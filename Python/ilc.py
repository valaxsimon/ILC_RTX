import ctypes
from ctypes import *

import matplotlib.pyplot as plt
import time
import random

#load the dll
ilcDll = ctypes.WinDLL("C:\\ABR\\ILC\\ILC_WINDOWS_SIDE\\x64\\Debug\\ILC_DLL.dll");
CExecuteTrajectory = ilcDll.CExecuteTrajectory
CExecuteTrajectory.argtypes = POINTER(c_double), POINTER(c_double), POINTER(c_double), POINTER(c_int)
CExecuteTrajectory.restype = c_int

#how long after theoretical mvt we want to monitor rtv values
samplingTime = 100e-6;
nextraSecond = 0.2;

#prepare the trajectory data
nelem = 3500
nExtraElem = int(nextraSecond / samplingTime);

nTotal = nelem + nExtraElem

timeStamps = (c_double * nTotal)()
	
forward = (c_double * nTotal)()
for i in range(0,nelem):
	forward[i] = i / (nelem-1);
	
for i in range(nelem, nTotal):
	forward[i] = forward[nelem-1];

backward = (c_double * nTotal)()
for i in range(nelem-1,-1,-1):
	backward[nelem - i - 1] = i / (nelem-1);
	
for i in range(nelem,nTotal):
	backward[i] = backward[nelem-1];
	
c_nTotal = c_int(nTotal)
ml1 = (c_double * nTotal)()

#prepare the plot window
timeStampsData = []
ml0Data = []
ml1Data = []
plt.show()
axes = plt.gca()
axes.set_xlim(0, 100e-6*nTotal * 1)
axes.set_ylim(-0.2, 1.2)
line_ml0, = axes.plot(timeStampsData, ml0Data, 'g-')
line_ml1, = axes.plot(timeStampsData, ml1Data, 'r-')
maxY = 0;

nexecution = 5;
for i in range(0,nexecution*2):
	if (i % 2) == 0:
		print("forward: ",end = '')
		print(i)
		print(c_nTotal)
		CExecuteTrajectory(timeStamps, forward, ml1, byref(c_nTotal))
		for j in range(0,nTotal):
			timeStampsData.append(timeStamps[j])
			ml0Data.append(forward[j])
			ml1Data.append(ml1[j])
		line_ml0.set_xdata(timeStampsData)
		line_ml0.set_ydata(ml0Data)
		line_ml1.set_xdata(timeStampsData)
		line_ml1.set_ydata(ml1Data)
	else:
		print("backward: ",end = '')
		print(i)
		CExecuteTrajectory(timeStamps, backward, ml1, byref(c_nTotal))
		for j in range(0,nTotal):
			timeStampsData.append(timeStamps[j])
			ml0Data.append(backward[j])
			ml1Data.append(ml1[j])
		line_ml0.set_xdata(timeStampsData)
		line_ml0.set_ydata(ml0Data)
		line_ml1.set_xdata(timeStampsData)
		line_ml1.set_ydata(ml1Data)
	
	if (maxY < max(ml1)):
		maxY = max(ml1)
		
	axes.set_xlim(min(timeStampsData), max(timeStampsData)) #set the new axis limit
	axes.set_ylim(-0.2, maxY+0.2)
	plt.draw()
	plt.pause(1e-17)
	time.sleep(0.1);
	
plt.show();
print("Completed")

#plt.plot(forward, timerange);
#plt.plot(ml1, timerange);

	
