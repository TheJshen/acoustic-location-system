import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import math
from pylab import *
from scipy import fftpack
from scipy import signal

def movingAverage(input, movAvgParam):
	ilen = len(input)
	dataMovAvg = np.zeros(ilen)
	for i in range(0,ilen-1):	
		start = i - movAvgParam
		finish = i + movAvgParam
		if start < 1:	
			start = 1
		if finish > ilen:
			finish = ilen
		dataMovAvg[i] = np.mean(input[start:finish])

	return dataMovAvg


def calculateSoundSpeedInWater(T, S, Dmeters):
	D = Dmeters/1000
	t = T/10

	#Calculating spund speed
	v0 = 1449.05 + 45.7*t - 5.21*t*t + .23*t*t*t + (1.333 - .126*t + .009*t*t) * (S - 35);  	#initial water propagation speed
	v = v0 + (16.23 + .253*t)*D + (.213 - .1*t)*D*D + (.016 + .0002*(S - 35)) * (S - 35);   	#water propagation speed

	return v

#TODO Fix filteret!
def frikkFilterDesigner(centerFreq, pBandwidth, filterOrder, Fs):
	#This function creates a bandpass filter with center frequency at
	#centerFreq and with a passband with of pBandwidt(or pBandwidth/2 to each
	#side from centerFreq). filterOrder is the order of the filter to be designed. 
	#Fs is the sampling frequency.

	N = filterOrder
	pbw = pBandwidth/2;
	cf = centerFreq;


	a=signal.firwin( numtaps=N, cutoff=cf-pbw, nyq=Fs/2)
	b=-signal.firwin( numtaps=N, cutoff=cf+pbw, nyq=Fs/2)

	h = - (a+b)


	#Frequency response
	#mfreqz(h)
	#show()



	return h

def filterAndSmooth( X, b, smoothingCoeff ):
	Y = signal.lfilter(b, 1.0, X)
	YMA = movingAverage(np.absolute(Y), smoothingCoeff)
	return YMA


#TODO FIX!
def getPositionFromSignal( x, b1, b2, b3, Fs, v, addedDelay, detectionThreshold, D, xa, ya, za, xb, yb, zb, xc, yc, zc ):
	# Designed by Frikk H Solberg
	# This function calculates x and y cordinates of ROV from a bunch of inputs:
	# x: recieved signal containing all three pulses.
	# b1: filter coeffs for bouy A 
	# b2: filter coeffs for bouy B 
	# b3: filter coeffs for bouy C 
	# Fs: Sampling frequency
	# v: sound speed in water
	# addedDelay: The set delay between pulsees from the bouys
	# detectionThreshold: the threshold for what would be detected as a recieved
	# pulse
	# D: depth from depth sensor
	# xa: bouy A x position
	# ya: bouy A y position
	# xb: bouy B x position
	# yb: bouy B y position
	# xc: bouy C x position
	# yc: bouy C y position

	smoothingCoeff = 20; # Smoothing coeff is set here as this setting works best.

	#Extracting the individual signals from each bouy
	extracted1 = filterAndSmooth(x, b1, smoothingCoeff);
	extracted2 = filterAndSmooth(x, b2, smoothingCoeff);
	extracted3 = filterAndSmooth(x, b3, smoothingCoeff);

	#Calculating the TDOA for each of the different bouy signals
	[tdoa1, tdoaLength1] = TDOA( extracted1, Fs, v, addedDelay , detectionThreshold);
	[tdoa2, tdoaLength2] = TDOA( extracted2, Fs, v, addedDelay , detectionThreshold);
	[tdoa3, tdoaLength3] = TDOA( extracted3, Fs, v, addedDelay , detectionThreshold);

	#Calculating the x and y positions of ROV from the different TDOA signals
	[x, y] = calculateProjectedCoordinates( D, v, tdoa1, tdoa2, tdoa3, xa, ya, za, xb, yb, zb, xc, yc, zc );

	return [x, y]


def TDOA(x, Fs, v, addedDelay, detectionTreshold):
	#Designed by Frikk H Solberg
	#This funciton calclulates the TDOA for signal x, given that x is filtered
	#and only consist of two pulses. It also uses sampling rate Fs, sound speed
	#in water v, addedDelay, which is the added delay between pulses and the
	#detectionTreshold.


	#Maximum time traveled for sound in water over 100m range
	#TW = 0.07
	TW = 0;
	expectedDelay = round((TW + addedDelay)*Fs);
	calculationCoeff = 1; #This variable can be changed for tuning the performance of the detection algorithm.

	#plt.figure()
	#plt.plot(x[expectedDelay:len(x)-1])
	lastPeak = expectedDelay + singleThresholdDetection(x[expectedDelay:len(x)-1], detectionTreshold);
        print lastPeak
	firstPeakTemp = singleThresholdDetection(np.flipud(x[1:(lastPeak-5)]), detectionTreshold);
	firstPeak = lastPeak - firstPeakTemp - 5;
        #firstPeak = singleThresholdDetection(x, detectionTreshold);
        print firstPeak
	tdoa = ((lastPeak - firstPeak)/Fs - addedDelay) * calculationCoeff;
        tdoaLength = tdoa * v;
	return [tdoa, tdoaLength]

#TODO Fix!
def singleThresholdDetection( x, threshold ):
	position = 0;

	for i in range(0,len(x)-1):

		if x[i] >= threshold:
			position = i;
			break

	return position


def calculateProjectedCoordinates( D, v, tsa, tsb, tsc, xa, ya, za, xb, yb, zb, xc, yc, zc ):
	#Designed by Frikk H Solberg
	#funciton calculates the projected coordinates using a simplified method
	#from the paper XX. 

	dsa = v*tsa;
	dsb = v*tsb;
	dsc = v*tsc;

        print tsa
        print tsb
        print tsc

	#Projecting to plane to move from 3D to 2D
	dsaProjected = sqrt((dsa*dsa - D*D));
	dsbProjected = sqrt(dsb*dsb - D*D);
	dscProjected = sqrt(dsc*dsc - D*D);

	#Calculating positions
	x = (dsaProjected*dsaProjected - dsbProjected*dsbProjected + xb*xb )/(2*xb);
	y = ((dsaProjected*dsaProjected - dscProjected*dscProjected + xc*xc + yc*yc)/(2*yc)) - ((xc)/(yc))*x;

	return [x, y]



def mfreqz(b,a=1):
    w,h = signal.freqz(b,a)
    h_dB = 20 * log10 (abs(h))
    subplot(211)
    plot(w/max(w),h_dB)
    ylim(-150, 5)
    ylabel('Magnitude (db)')
    xlabel(r'Normalized Frequency (x$\pi$rad/sample)')
    title(r'Frequency response')
    subplot(212)
    h_Phase = unwrap(arctan2(imag(h),real(h)))
    plot(w/max(w),h_Phase)
    ylabel('Phase (radians)')
    xlabel(r'Normalized Frequency (x$\pi$rad/sample)')
    title(r'Phase response')
    subplots_adjust(hspace=0.5)
