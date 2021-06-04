import ctypes
from ctypes import *

import numpy as np
import pandas as pd
import scipy.signal as signal
import control as ctrl
import matplotlib.pyplot as plt
import scipy

class Etel_ILC:
    
    def __init__(self, trajectory_file_path, moving_mass, kt):
        
        #load the dll
        ilcDll = ctypes.WinDLL("C:\\Code\\ABR\\ILC\\ILC_WINDOWS_SIDE\\x64\\Debug\\ILC_DLL.dll");
        self.CExecuteTrajectory = ilcDll.CExecuteTrajectory
        self.CExecuteTrajectory.argtypes = POINTER(c_double), POINTER(c_double), POINTER(c_double), POINTER(c_double), POINTER(c_double), POINTER(c_int)
        self.CExecuteTrajectory.restype = c_int
        
        self.trajectory_file_path = trajectory_file_path
        self.sampling_time = 50e-6
        self.nb_sample = 0
        self.mass = moving_mass
        self.kt = kt
        
        self.learning_score = 0
        
        # filter 
        
        # python array data
        self.traj_theoPos_write = [] # array of double theo position
        self.traj_theoPos_read = []
        self.traj_realPos_read = []
        self.traj_acc = [] # array of doucle theo acceleration
        self.traj_tracking = [] 
        self.traj_time_stamp_read = []
        self.traj_currFfw_write = []
        self.traj_currFfw_ILC_debug = []
        
        # c_double array data (for communication with RTX)
        self.traj_theoPos_write_rtx = [] # c_double array theo position to be pass to RTX
        self.traj_theoPos_read_rtx = [] # c_double array theo position read from RTX
        self.traj_time_stamp_rtx = []
        
        self.traj_realPos_read_rtx = []
        self.traj_currFfw_write_rtx = []

        self.b = [] # Numerator polynomial coefficients
        self.a = [] # Denominator polynomial coefficients
        self.lowpass_filter = []
        self.L_filter = []
        self.axis_inverse_plant = []
                
        # load 
        self.load_traj()
        self.compute_acceleration_ffw()
        self.load_L_filter_parameter('Lfilter/A.txt','Lfilter/B.txt','Lfilter/C.txt','Lfilter/D.txt')
        
        
        
    def convert_to_c_double_type(self, array):
        return_c_double = (c_double * len(array))()
        for n in range(len(array)):
            return_c_double[n] = array[n]
        return return_c_double
    
    def convert_from_c_double_to_list_typ(self, c_double_input):
        return_list = [0] * len(c_double_input)
        for n in range(len(c_double_input)):
            return_list[n] = c_double_input[n]
        return return_list

    def load_traj(self):
        df = pd.read_csv(self.trajectory_file_path)
        
        self.traj_theoPos_write = df.M0.values
        self.traj_acc = df.M14.values
        self.nb_sample = df.M0.count()
        
        self.traj_theoPos_write_rtx = self.convert_to_c_double_type(self.traj_theoPos_write)
        self.traj_theoPos_read_rtx = (c_double * self.nb_sample)() # allocate memory for the returned data
        self.traj_time_stamp_rtx = (c_double * self.nb_sample)()
        self.traj_realPos_read_rtx = (c_double * self.nb_sample)()
        del(df)
        
    def compute_cps(self, data):
        N = len(data)
        T = self.sampling_time
        yf = scipy.fft.fft(data)
        
        xf = np.linspace(0.0,1.0//(2.0*T), N//2)
        yf_abs = (2.0/N * np.abs(yf[:N//2]))**2
        return np.sum(yf_abs)
    
    def load_L_filter_parameter(self,A_path,B_path,C_path,D_path):
        self.A = np.array(pd.read_csv(A_path,header=None,dtype=(float)).values)
        self.B = np.array(pd.read_csv(B_path,header=None,dtype=(float)).values)
        self.C = np.array(pd.read_csv(C_path,header=None,dtype=(float)).values)
        self.D = np.array(pd.read_csv(D_path,header=None,dtype=(float)).values)
        
    # xk is internal state of the state space filter
    # uk is the new state
    def stateSpaceFilter(self,uk,xk):
        xkp1= np.dot(self.A,xk) + np.dot(self.B,uk)
        yk = np.dot(self.C,xk) + self.D*uk

        return yk, xkp1
    

    def apply_L_filter(self,U):
        # xk = internal state space filter value (np.shape(xk) = (len(B),1))
        xk = np.zeros([len(self.B),1])
        y = np.zeros(len(U))
        
        for kk, ui in enumerate(U):
            y[kk], xk = self.stateSpaceFilter(ui, xk)        
        return y
    
    def compute_new_ILC_ffw(self):
        self.traj_currFfw_ILC_debug = self.traj_currFfw_write + np.multiply(self.apply_L_filter(self.traj_tracking),1e5)

        
    def compute_acceleration_ffw(self):
        self.traj_currFfw_write = np.multiply(self.traj_acc, (self.mass / self.kt)) 
        #self.traj_currFfw_write = np.multiply(self.traj_acc, 0.0) 
        self.traj_currFfw_write_rtx = self.convert_to_c_double_type(self.traj_currFfw_write)

        
    def execute_trajectory(self):
        self.CExecuteTrajectory(self.traj_time_stamp_rtx,
                                self.traj_theoPos_write_rtx,
                                self.traj_theoPos_read_rtx,
                                self.traj_currFfw_write_rtx,
                                self.traj_realPos_read_rtx,
                                byref(c_int(self.nb_sample)))
        
        self.traj_theoPos_read = self.convert_from_c_double_to_list_typ(self.traj_theoPos_read_rtx)
        self.traj_realPos_read = self.convert_from_c_double_to_list_typ(self.traj_realPos_read_rtx)
        self.traj_tracking = np.subtract(self.traj_theoPos_read, self.traj_realPos_read)
        self.learning_score = self.compute_cps(self.traj_tracking)
        
    def compute_new_ffw(self):
        self.traj_currFfw_write = np.multiply(self.traj_currFfw_write, 0.75)
        self.traj_currFfw_write_rtx = self.convert_to_c_double_type(self.traj_currFfw_write)
        
    def tune(self, max_iter=5):
        
        # Measure baseline
        self.execute_trajectory()
        self.plot('Baseline motion, score = ' + str(self.learning_score))
        
        for n in range(max_iter):
            self.compute_new_ffw()
            self.execute_trajectory()
            self.plot('iteration ' + str(n) + ', score = ' + str(self.learning_score))
   
    def plot(self, title=''):
        
        time = np.linspace(0,len(self.traj_theoPos_read)*self.sampling_time * 1e3,len(self.traj_theoPos_read))
        
        plt.figure()
        plt.suptitle(title)
        plt.subplot(211)
        plt.plot(time, np.multiply(self.traj_theoPos_read,1e3), c='C0')
        plt.ylabel('Theo pos [mm]', c='C0')
        plt.yticks(c='C0')
        plt.xlabel('time [ms]')
        plt.grid()
        plt.twinx()
        plt.plot(time, np.multiply(self.traj_tracking,1e6), c='C1')
        plt.ylabel('tracking error[um]', c='C1')
        plt.yticks(c='C1')
        plt.grid()
        
        plt.subplot(212)
        plt.plot(time, np.multiply(self.traj_theoPos_read,1e3), c='C0')
        plt.ylabel('Theo pos [mm]', c='C0')
        plt.yticks(c='C0')
        plt.xlabel('time [ms]')
        plt.grid()
        plt.twinx()
        plt.plot(time, self.traj_currFfw_write, c='C2')
        plt.ylabel('FFW current [A]', c='C2')
        plt.yticks(c='C2')
        
        plt.tight_layout()
        plt.draw()


toto = Etel_ILC('TheoTraj_for_ILC_1mmTrajVulcano1',0.0,25.7)