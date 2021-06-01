import ctypes
from ctypes import *

import numpy as np
import pandas as pd
import scipy.signal as signal
import control as ctrl

class Etel_ILC:
    
    def __init__(self, trajectory_file_path, moving_mass, kt):
        
        #load the dll
        ilcDll = ctypes.WinDLL("C:\\Code\\ABR\\ILC\\ILC_WINDOWS_SIDE\\x64\\Debug\\ILC_DLL.dll");
        self.CExecuteTrajectory = ilcDll.CExecuteTrajectory
        self.CExecuteTrajectory.argtypes = POINTER(c_double), POINTER(c_double), POINTER(c_double), POINTER(c_double), POINTER(c_int)
        self.CExecuteTrajectory.restype = c_int
        
        self.trajectory_file_path = trajectory_file_path
        self.sampling_time = 50e-6
        self.nb_sample = 0
        self.mass = moving_mass
        self.kt = kt
        
        # python array data
        self.traj_pos = [] # array of double theo position
        self.traj_acc = [] # array of doucle theo acceleration
        self.traj_real_pos = []
        self.traj_tracking = [] 
        self.traj_curr_ffw = []
        
        # c_double array data (for communication with RTX)
        self.traj_pos_rtx = [] # c_double array theo position to be pass to RTX
        self.traj_time_stamp_rtx = []
        self.traj_real_pos_rtx = []
        self.traj_curr_ffw_rtx = []

        self.b = [] # Numerator polynomial coefficients
        self.a = [] # Denominator polynomial coefficients
        self.lowpass_filter = []
        self.L_filter = []
        self.axis_inverse_plant = []
                
        self.load_traj()
        self.compute_acceleration_ffw()
        
    def convert_to_c_double_type(self, array):
        return_c_double = (c_double * len(array))()
        for n in range(len(array)):
            return_c_double[n] = array[n]
        return return_c_double
        
    def create_L_filter(self):
        self.L_filter = signal.tf2zpk(self.b, self.a)

    def load_traj(self):
        df = pd.read_csv(self.trajectory_file_path)
        
        self.traj_pos = df.M0.values
        self.traj_pos_rtx = (c_double * self.nb_sample)()
        self.traj_pos_rtx = self.convert_to_c_double_type(self.traj_pos)
        
        self.traj_acc = df.M14.values
        self.nb_sample = df.M0.count()
        self.traj_time_stamp_rtx = (c_double * self.nb_sample)()
        self.traj_real_pos_rtx = (c_double * self.nb_sample)()
        del(df)
        
    def compute_acceleration_ffw(self):
        self.traj_curr_ffw = np.multiply(self.traj_acc, (self.mass / self.kt))
        self.traj_curr_ffw_rtx = self.convert_to_c_double_type(np.multiply(self.traj_acc, (self.mass / self.kt)))
        
    def execute_trajectory(self):
        self.CExecuteTrajectory(self.traj_time_stamp_rtx, self.traj_pos_rtx, self.traj_curr_ffw_rtx, 
                           self.traj_real_pos_rtx, byref(c_int(self.nb_sample)))
        
    def debug_traj_pos_rtx(self):
        self.debug_val = np.empty(shape=(len(self.traj_pos_rtx)))
        for n in range(len(self.traj_real_pos_rtx)):
            self.debug_val[n]= self.traj_real_pos_rtx[n]
        

# #load the dll
# ilcDll = ctypes.WinDLL("C:\\ABR\\ILC\\ILC_WINDOWS_SIDE\\x64\\Debug\\ILC_DLL.dll");
# CExecuteTrajectory = ilcDll.CExecuteTrajectory
# CExecuteTrajectory.argtypes = POINTER(c_double), POINTER(c_double), POINTER(c_double), POINTER(c_double), POINTER(c_int)
# CExecuteTrajectory.restype = c_int

# #how long after theoretical mvt we want to monitor rtv values
# samplingTime = 50e-6
# nextraSecond = 0.2

# #prepare the trajectory data
# df = pd.read_csv('theoTrajLKT1mmVulc1_X.txt')
# forward = convert_to_c_double_type(df['M0 in meter'].values)
# ffwCurrentForward = convert_to_c_double_type(df['MF231 in A'].values)

# nTotal = len(df['M0 in meter'].values)
# timeStamps = (c_double * nTotal)()
# 	
# c_nTotal = c_int(nTotal)
# ml1 = (c_double * nTotal)()

# #prepare the plot window
# timeStampsData = []
# ml0Data = []
# ml1Data = []

# #line_ml0, line_ml1, axes = plot_iteration_results(timeStampsData, ml0Data, ml1Data)

# #plt.show()
# axes = plt.gca()

# line_ml0, = axes.plot(timeStampsData, ml0Data, 'g+')
# line_ml1, = axes.plot(timeStampsData, ml1Data, 'r+')
# #maxY = 0;

# nexecution = 2;
# for i in range(0,nexecution*2):
# 	print("forward: ",end = '')
# 	print(i)
# 	print(c_nTotal)

# 	CExecuteTrajectory(timeStamps, forward, ffwCurrentForward, ml1, byref(c_nTotal))
# 	for j in range(0,nTotal):
# 		timeStampsData.append(timeStamps[j]*samplingTime)
# 		ml0Data.append(forward[j])
# 		ml1Data.append(ml1[j])

# 	line_ml0.set_xdata(timeStampsData)
# 	line_ml0.set_ydata(ml0Data)
# 	line_ml1.set_xdata(timeStampsData)
# 	line_ml1.set_ydata(ml1Data)
# 		
# 	axes.set_xlim(min(timeStampsData), max(timeStampsData)) #set the new axis limit
# 	axes.set_ylim(min(ml1)-0.05*np.mean(ml1), max(ml1)+0.05*np.mean(ml1))
# 	plt.draw()
# 	plt.pause(1e-17)
# 	time.sleep(0.1);
# 	
# plt.show();
# print("Completed")

# #plt.plot(forward, timerange);
# #plt.plot(ml1, timerange);

# 	
