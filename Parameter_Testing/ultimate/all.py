#!/usr/bin/python

from todaFunctions import *
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import math

import scipy.io as sio


f1 = float(sys.argv[1])
f2 = float(sys.argv[2])
f3 = float(sys.argv[3])

print f1

#print ("Starting postion acquisition test:")

#print ("Preparing Signals...")
#Creating test signals
## Setup
pi = np.pi
Fs = 250000  #Samlpling frequency, Should be no problem to change this
tps = int(sys.argv[5])     #Transmits per second
#Fs = 2*f1

#creating sines of length t with the different frequencies
#t = [0:1/Fs:1]
t = np.arange(0, 1, 1./Fs)
x1 = np.sin(2*pi*f1*t)
x2 = np.sin(2*pi*f2*t)
x3 = np.sin(2*pi*f3*t)

#Choosing number of elements from x1, x2 and x3 to create pulse of certain
#number of cycles. Needs to be done in a nicer way.
#x1Length = 24*6
#x2Length = 21*6
#x3Length = 19*6

#Sets number of cycles to create for each pulse (command line arg)
#NOTE: 11 seems to work pretty accurately
numCycles = int(sys.argv[4])

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

#Creating Recieved Signal
T =  15;                                         #Temp in C
S = 5;                                          #Salinity in parts per thousand
D = int(sys.argv[9])                                            #Depth (0 - 100) in meters
v = calculateSoundSpeedInWater(T, S, D);          #Sound speed in water

#Bouy positions relative to bouy A in meters. Change these to test
#performance for different configurations of bouy postions. 
#PS: For the algortihm to work, A has to be at 0.0, B has to be at x.0 and
#C has to be at 0.y. So change xb and yc only!!
xa = 0;
ya = 0;
za = 0;

xb = 2;
yb = 0;
zb = 0;

xc = 0;
yc = 2;
zc = 0;

#Cheating to get correct propagation times from bouy to ROV
#ROV position

#This is the actal position of ROV in meters. Change these to test performance with
#different ROV positions.
xs = int(sys.argv[7])
ys = int(sys.argv[8])
zs = D;  

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



#FFT of recorded signal
#frikkFFT(recSignal, Fs); #frikkFFT is not yet implemented


## Filter Design
#Here three filters are designed to extract the signal from each of the
#bouys. The filter design process is computationally heavy and is only
#necessary to perform if pbw, sbw or Fs is changed. 

#print ("Designing filters...")

pbw = 1000; # Passband width in hertz
fOrder = int(sys.argv[6]) ; # Filter order number now the most optimal 
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
## Plotting filtered, absed and smoothed Signals
## Detection stuff
addedDelay = .1; # The chosen set delay-time between pulses
detectionThreshold = .05; # A threshold for how strong a signal needs to be on order to be detected as a recieved pulse.

# This function calculates the x and y coordinates of the ROV.
#In reality, when implemented this would be the only necesary function to
#run as all other parameters should be set.
[x, y] = getPositionFromSignal( recSignal, b1, b2, b3, Fs, v, addedDelay, detectionThreshold, D, xa, ya, za, xb, yb, zb, xc, yc, zc )


xerr = abs(xs - x) / xs
yerr = abs(ys - y) / ys


totale = abs(xerr + yerr)
fil = "A%i_B%i_C%i_NC%i_TPS%i_OF%i.txt" %(f1, f2, f3, numCycles, tps, fOrder) 

f = open (fil, "a")

string = "%f \n" %totale
f.write(string)

f.close()






