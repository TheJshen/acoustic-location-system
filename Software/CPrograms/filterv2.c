/*******************************************************************************
 * 	filter.c
 * by Jason Shen, James Smith and Luis Sanchez
 * 
 * Acoustic Location System Filtering
 * 
 * This C module implements the DSP filtering of an input signal for the 
 * STM33F446RE Nucleo board.. The input signal will be passed in through 
 * on board ADC connected to a transponder that receives accoustic waves 
 * delivered by three transponders at 3 different frequencies. The coeffecients 
 * used for the filter are defined in main.h and implement  bandpass filters for
 * 35kHz, 40kHz and 45kHz with a pass bandwith of 500Hz. 
 * 
 * This program will output the time difference of arrival of the three
 * varying frequency signals. the default for for output is:
 *
 * [ 35 vs 40 kHz time difference, 
 *   40 vs 45 kHz time difference,
 *   35 vs 45 kHz time difference ]
 *
 * Currently the algorithm will first FIR filter the data for each of the 3 
 * frequencies., Then it will use an in place moving average function to 
 * smooth the data. Finally, it will run a peak detection function and find the 
 * peak of each of the 3 filtered signals which will then be placed on an output
 * pin to be available for beagle bone on board OpenROV.
 *
 *
*******************************************************************************/

/* Includes */
#include <stdio.h> //fprintf, FILE
#include <stdlib.h> //
#include <math.h> //sqrt

#include "mbed.h"
#include "main.h"
#include "float.h"

#define BLOCK_SIZE      (32)
#define MOV_AVG_WIND  20


//Fir filter variables 
static float firStateF[BLOCK_SIZE + NUM_TAPS - 1];
unsigned int blockSize = BLOCK_SIZE;
unsigned int numBlocks = NUM_SAMPLES/BLOCK_SIZE;


/* Function Prototypes */
float calculateSoundSpeedInWater(float temp, float salinity, float depth);

int singleThresholdDetection(const float sampleData[NUM_SAMPLES], int startPosition);

void filterAndSmooth(float signal[NUM_SAMPLES], float filterCoeffs[NUM_TAPS], int smoothingCoeff, float * filteredSignal);

void movAvg(float signal[]) {

//------------------------------------
// Hyperterminal configuration
// 9600 bauds, 8-bit data, no parity
//------------------------------------
Serial pc(SERIAL_TX, SERIAL_RX);

DigitalOut myled(LED1);

int main(void)
{

	printf("Running Filter... ");
	printf("Number of Samples: %d\n", NUM_SAMPLES);
	printf("Moving Average window: %d samples\n", MOV_AVG_WIND); 
	
	/*
    float velocity = calculateSoundSpeedInWater(temp, salinity, depth);
    pc.printf("The velocity is %lf", velocity);
	*/
   
	// Fir instance variables 
    arm_fir_instance_f32 S1, S2, S3;
    arm_status status;
    float32_t *input, *output;
    
	// Fir Coefficients
    float coeffsA_f32[NUM_TAPS];
    float output_f32[NUM_SAMPLES];
    
	/* Input is the signal with all 3 frequencies present. */    
    input = &signalABC[0];
    output = &output_f32[0];
    
    /* Initialize FIR instance structure */
    arm_fir_init_f32(&S1, NUM_TAPS, (float *) &coeffsA[0], 
		&firStateF[0], blockSize);
    
	int i = 0;
    
    /* FIR Filtering */
    for( i = 0; i < numBlocks; i++) {
        arm_fir_f32(&S1, input + (i * blockSize), 
						output + (i * blockSize), blockSize);
    }
    
    /* Take the absolute value of the filtered signal */
    for(i = 0; i < NUM_SAMPLES; i++ ) {
        if(output[i] < 0) {
            output[i] = -1 * output[i];    
        }
    } 
	
	/* Print before moving average */ 
	printf("----------Before moving average-------------\n");
    for(i = 0; i < NUM_SAMPLES; i++) {
        printf("%lf\n", output[i]);
    }
    
    float * avgSignal[NUM_SAMPLES];
   	// Calculate moving average and do peak detection; 
    movAvg(output);
   
	/* Print signal after moving average */ 
	printf("----------After moving average-------------\n");
    for(i = 0; i < NUM_SAMPLES; i++) {
        printf("%lf\n", output[i]);
    }
}

/*
 * Function: singleThresholdDetection()
 * Description: calculates the first occurrence of a value over the threshold
 * value and returns it.
 *
 * Params:
 *  arg1: sampleData - sample data to find position of first peak
 *  arg2: startPosition - threshold value to search for
 *
 * Return Value: position where threshold is first reached
 *
 */
int singleThresholdDetection(const float sampleData[], int startPosition)
{
    int i;
":
    for(i = startPosition; i < NUM_SAMPLES; i++) {
        if (sampleData[i] >= DETECT_THRESH) {
            return i;
        }
    }
    
    return  -1;
}


/* Function Name: movAvg();
 * Function Declaration: void movAvg(float signal[]);
 * Function Description: This function takes an input signal and does an in
 * place moving average algorithm. The window size is defined by MOV_AVG_WIND
 * in main.h. 
 *
 * This function serves a dual purpose of finding the position of the largest 
 * peak and returning that position.
 *
 */
int movAvg(float signal[]) {
     int i = 0;
    
    int start = 0;
    int finish = 0;
    float total = 0;

	int maxIndex = 0;
	float max = 0;
	
	// Buffer to hold the most recent samples
    float buffer[MOV_AVG_WIND*2];
    
	// Go through signal array and calculate moving average
    for(i = 0; i < NUM_SAMPLES; i++) {
        start = i - MOV_AVG_WIND;
        finish = i + MOV_AVG_WIND;
        
        if(start < 0) {
            start = 0;
        }

        if(finish > NUM_SAMPLES) {
            finish = NUM_SAMPLES;
        }
        
        total = 0;
        for(int j = start; j < finish; j++) {
            total += signal[j];
        }
        
        if(i > MOV_AVG_WIND*2 - 1) {
            signal[i-(MOV_AVG_WIND*2)] = buffer[i % (MOV_AVG_WIND*2)];
			if(buffer[i % (MOV_AVG_WIND * 2)] > max) {
				maxPosition = i - MOV_AVG_WIND * 2;
				max = 
        }

        // Rotating buffer to save the avg
        buffer[i%(MOV_AVG_WIND*2)] = total / (finish - start); 
			
    }
    for(int i = NUM_SAMPLES-MOV_AVG_WIND*2; i < NUM_SAMPLES; i++) 
        signal[i] = buffer[i %(MOV_AVG_WIND*2)];
	
    for(int i = 0; i < NUM_SAMPLES; i++)
        printf("averagedSignal[i] = %f\n", signal[i]);
	}
}


