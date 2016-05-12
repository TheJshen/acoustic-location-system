#!/usr/bin/python

from todaFunctions import *
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import math

import scipy.io as sio


testCon = open('tester_conf', 'r')

f1 = float(testCon.readline().split(' #')[0])
f2 = float(testCon.readline().split(' #')[0])
f3 = float( testCon.readline().split(' #')[0])

Fs = float( testCon.readline().split(' #')[0])
tps = float( testCon.readline().split(' #')[0])

pi = np.pi

numCycles = float( testCon.readline().split(' #')[0])


T = float( testCon.readline().split(' #')[0])
S = float( testCon.readline().split(' #')[0])
D = float( testCon.readline().split(' #')[0])

xa = float( testCon.readline().split(' #')[0])
ya = float( testCon.readline().split(' #')[0])
za = float( testCon.readline().split(' #')[0])

xb = float( testCon.readline().split(' #')[0])
yb = float( testCon.readline().split(' #')[0])
zb = float( testCon.readline().split(' #')[0])

xc = float( testCon.readline().split(' #')[0])
yc = float( testCon.readline().split(' #')[0])
zc = float( testCon.readline().split(' #')[0])

#This is the actal position of ROV in meters. Change these to test performance with
#different ROV positions.
xs = float( testCon.readline().split(' #')[0]) #x location of ROV
ys = float( testCon.readline().split(' #')[0]) #y location of ROV
zs = D


pbw = float( testCon.readline().split(' #')[0]) # Passband width in hertz
fOrder = float( testCon.readline().split(' #')[0]) # Filter order number


smoothingCoeff = float( testCon.readline().split(' #')[0])

addedDelay = float( testCon.readline().split(' #')[0]) # The chosen set delay-time between pulses
detectionThreshold = float( testCon.readline().split(' #')[0]) # A threshold for how strong a signal needs to be on order to be detected as a recieved pulse.

testCon.close()
#print ("Starting postion acquisition test:")

#print ("Preparing Signals...")
#Creating test signals

#creating sines of length t with the different frequencies
#t = [0:1/Fs:1]
t = np.arange(0, 1, 1./Fs)
x1 = np.sin(2*pi*f1*t)
x2 = np.sin(2*pi*f2*t)
x3 = np.sin(2*pi*f3*t)

#Sets number of cycles to create for each pulse (command line arg)

#Chooses number of elements from x1, x2, and x2 to create depending on numCycles
#requested
x1Length = ( 1.0 / f1) * Fs * numCycles
x2Length = ( 1.0 / f2) * Fs * numCycles
x3Length = ( 1.0 / f3) * Fs * numCycles

temp = np.zeros(.1*Fs)

#Putting together a number of pulses and zero padding so all the signals
#has the same number of points
x1Sig = np.concatenate((x1[0:x1Length],temp), axis = 0);
x2Sig = np.concatenate((x2[0:x2Length],temp), axis = 0);
x3Sig = np.concatenate((x3[0:x3Length],temp), axis = 0);

#Plotting the pulses without zero padding
#plt.figure()
#plt.plot(x1Sig[0:x1Length])
#plt.title('X1 without zero padding')
#plt.figure()
#plt.plot(x2Sig[1:x2Length])
#plt.title('X2 without zero padding')
#plt.figure()
#plt.plot(x3Sig[1:x3Length])
#plt.title('X3 without zero padding')
#plt.show()

#Creating Recieved Signal
v = calculateSoundSpeedInWater(T, S, D);          #Sound speed in water

#Bouy positions relative to bouy A in meters. Change these to test
#performance for different configurations of bouy postions. 
#PS: For the algortihm to work, A has to be at 0.0, B has to be at x.0 and
#C has to be at 0.y. So change xb and yc only!!

#Cheating to get correct propagation times from bouy to ROV
#ROV position


#A whole bunch of math to reverse calculate the TDOA so the signals sent
#from bouys have the correct progagation delay
dsaTemp = np.sqrt(D*D + (xs - xa)*(xs - xa) + (ys - ya)*(ys - ya));
dsbTemp = np.sqrt(D*D + (xs - xb)*(xs - xb) + (ys - yb)*(ys - yb));
dscTemp = np.sqrt(D*D + (xs - xc)*(xs - xc) + (ys - yc)*(ys - yc));

#Calculate time to receive from distance / velocity of sound
tsa = dsaTemp/v;
tsb = dsbTemp/v;
tsc = dscTemp/v;

#Calculate delays based on time to receive
x1Delay = np.zeros(round(tsa * Fs));
x2Delay = np.zeros(round(tsb * Fs));
x3Delay = np.zeros(round(tsc * Fs));

#Concatenate calculated delays with original signals
x1Rec = np.concatenate((np.concatenate((np.concatenate((x1Delay, x1Sig),axis = 0), np.concatenate((x1Delay, x1[0:x1Length]), axis = 0)), axis = 0), temp), axis = 0)
x2Rec = np.concatenate((np.concatenate((np.concatenate((x2Delay, x2Sig),axis = 0), np.concatenate((x2Delay, x2[0:x2Length]), axis = 0)), axis = 0), temp), axis = 0)
x3Rec = np.concatenate((np.concatenate((np.concatenate((x3Delay, x3Sig),axis = 0), np.concatenate((x3Delay, x3[0:x3Length]), axis = 0)), axis = 0), temp), axis = 0)


#Determine the number of samples for the received signal. Why the nested min()?
recSigLength = min(min(len(x1Rec), len(x2Rec)), len(x3Rec));


#######################################################################################################
#	This is where the actual stuff starts to happen. All the way up until this is mostly to create the 
#	input signal
#######################################################################################################



#recSignal would be the signal recieved on the ROV in a practical case. This is
#all three bouy signals added together with the correct propagation delay.
#This signal would also (usually and hopefully) include 2 peaks from each
#bouy so that TDOA is possible to calculate
recSignal = x1Rec[1:recSigLength] + x2Rec[1:recSigLength] + x3Rec[1:recSigLength];


## Filter Design
#Here three filters are designed to extract the signal from each of the
#bouys. The filter design process is computationally heavy and is only
#necessary to perform if pbw, sbw or Fs is changed. 

#print ("Designing filters...")

b1 = frikkFilterDesigner( f1, pbw, fOrder, Fs );
b2 = frikkFilterDesigner( f2, pbw, fOrder, Fs );
b3 = frikkFilterDesigner( f3, pbw, fOrder, Fs );

#print ("Done")

## Filtering and smoothing
#Here each of bouy signals are abseda and filtered with their corresponding filter
#from above, and a moving average is performed in order to smooth/LP-filter
#the signals. smoothingCoeff determines how many adjacent datapoint are
#used in the smoothing process. 

#print ("Calculating ROV position")

extracted1 = filterAndSmooth(recSignal, b1, smoothingCoeff);
extracted2 = filterAndSmooth(recSignal, b2, smoothingCoeff);
extracted3 = filterAndSmooth(recSignal, b3, smoothingCoeff);


# This function calculates the x and y coordinates of the ROV.
#In reality, when implemented this would be the only necesary function to
#run as all other parameters should be set.
[x, y] = getPositionFromSignal( recSignal, b1, b2, b3, Fs, v, addedDelay, detectionThreshold, D, xa, ya, za, xb, yb, zb, xc, yc, zc )

#print ("Done")

#print ("")

#print ("Calculated position:")

#print ("D = " , D, "(Not calculated)")
print ("X = " , x)
print ("Y = " , y)


#plt.show()




