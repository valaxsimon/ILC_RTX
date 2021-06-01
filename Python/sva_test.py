import ctypes
from ctypes import *

import matplotlib.pyplot as plt
import time
import random
import numpy as np
import pandas as pd

def convert_to_c_double_type(array):
    return_c_double = (c_double * len(array))()
    for n in range(len(array)):
        return_c_double[n] = array[n]
    return return_c_double

# def plot_iteration_results(timeStampsData, ml0Data, ml1Data, axes=0, line_ml0=0, line_ml1=0):
#     # check if first iteration
#     if len(timeStampsData == 0):
#         line_ml0, = axes.plot(timeStampsData, ml0Data, 'g+', label='theo pos')
#         line_ml1, = axes.plot(timeStampsData, ml1Data, 'r+', label='real pos')
#         plt.show()
#         axes = plt.gca()
#     else:
#         line_ml0.set_xdata(timeStampsData)
#         line_ml0.set_ydata(ml0Data)
#         line_ml1.set_xdata(timeStampsData)
#         line_ml1.set_ydata(ml1Data)
        
#     	axes.set_xlim(min(timeStampsData), max(timeStampsData))
#     	axes.set_ylim(min(ml1)-0.05*np.mean(ml1), max(ml1)+0.05*np.mean(ml1))
#     	plt.draw()
#     	plt.pause(1e-17)
#     	time.sleep(0.1);
        
#     return line_ml0, line_ml1, axes
        

#load the dll
ilcDll = ctypes.WinDLL("C:\\Code\\ABR\\ILC\\ILC_WINDOWS_SIDE\\x64\\Debug\\ILC_DLL.dll");
CExecuteTrajectory = ilcDll.CExecuteTrajectory
CExecuteTrajectory.argtypes = POINTER(c_double), POINTER(c_double), POINTER(c_double), POINTER(c_double), POINTER(c_int)
CExecuteTrajectory.restype = c_int

#how long after theoretical mvt we want to monitor rtv values
samplingTime = 50e-6
nextraSecond = 0.2

#prepare the trajectory data
df = pd.read_csv('theoTrajLKT1mmVulc1_X.txt')
forward = convert_to_c_double_type(df['M0 in meter'].values)
ffwCurrentForward = convert_to_c_double_type(df['MF231 in A'].values)

nTotal = len(df['M0 in meter'].values)
timeStamps = (c_double * nTotal)()
	
c_nTotal = c_int(nTotal)
ml1 = (c_double * nTotal)()

#prepare the plot window
timeStampsData = []
ml0Data = []
ml1Data = []

#line_ml0, line_ml1, axes = plot_iteration_results(timeStampsData, ml0Data, ml1Data)

#plt.show()
axes = plt.gca()

line_ml0, = axes.plot(timeStampsData, ml0Data, 'g+')
line_ml1, = axes.plot(timeStampsData, ml1Data, 'r+')
#maxY = 0;

nexecution = 2;
for i in range(0,nexecution*2):
	print("forward: ",end = '')
	print(i)
	print(c_nTotal)

	CExecuteTrajectory(timeStamps, forward, ffwCurrentForward, ml1, byref(c_nTotal))
	for j in range(0,nTotal):
		timeStampsData.append(timeStamps[j]*samplingTime)
		ml0Data.append(forward[j])
		ml1Data.append(ml1[j])

	line_ml0.set_xdata(timeStampsData)
	line_ml0.set_ydata(ml0Data)
	line_ml1.set_xdata(timeStampsData)
	line_ml1.set_ydata(ml1Data)
		
	axes.set_xlim(min(timeStampsData), max(timeStampsData)) #set the new axis limit
	axes.set_ylim(min(ml1)-0.05*np.mean(ml1), max(ml1)+0.05*np.mean(ml1))
	plt.draw()
	plt.pause(1e-17)
	time.sleep(0.1);
	
plt.show();
print("Completed")

#plt.plot(forward, timerange);
#plt.plot(ml1, timerange);

	
