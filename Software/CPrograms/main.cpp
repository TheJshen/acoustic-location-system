/*******************************************************************************
 * main.cpp 
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
 * [ A vs B time difference, B vs C time difference, A vs C time difference ]
 *
 * Currently the algorithm will first FIR filter the data for each of the 3 
 * frequencies. Then it will use an in place moving average function to 
 * smooth the data. Finally, it will run a peak detection function and find the 
 * peak of each of the 3 filtered signals which will then be placed on an output
 * pin to be available for beagle bone on OpenROV.
 *
 *
 *
 *
 * TODO: 
 * 1.Test on different location  inputs from python script,
 * 2. Integrate ADC and filter code
 * 3. If current filtering ans smoothing is innaccurate develop new algorithm
 * 4. Clean up code
*******************************************************************************/

/* Includes */
#include "main.h"




/* FIR filter variables */
static float firStateF[BLOCK_SIZE + NUM_TAPS - 1];
unsigned int blockSize = BLOCK_SIZE;
unsigned int numBlocks = NUM_SAMPLES/BLOCK_SIZE;


/* Function Prototypes */
float calculateSoundSpeedInWater(float temp, float salinity, float depth);

int singleThresholdDetection(const float sampleData[], int startPosition);

void filterAndSmooth(float signal[], float filterCoeffs[], int smoothingCoeff, float * filteredSignal);

void movAvg(float signal[]);

int findMax(float signal[]);

/*------------------------------------
 * Hyperterminal configuration
 * 9600 bauds, 8-bit data, no parity
 ************************************/
Serial pc(SERIAL_TX, SERIAL_RX);

DigitalOut myled(LED1);  //Debug LED

int main(void)
{
    printf("---------------------------------------------------------------- ");
    printf("                                                                 ");
    printf("    Underwater Acoustic Location System DSP\n");
    printf("                                                                 ");
    printf("---------------------------------------------------------------- ");
    printf("    Running Filter... ");
    printf("---------------------------------------------------------------- ");
    printf("    Number of Samples: %d\n", NUM_SAMPLES);
    printf("    Moving Average window: %d samples\n", MOV_AVG_WIND); 
    
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
    
    /* The array of time differences to place on output pin */
    int toReturn[3]; 
        
    /***************************************************************************
    ****************** Filtering for f = 35kHz (from buoy A) *******************
    ***************************************************************************/
    if(UALS_DEBUG) {
        printf("Beginning filtering for f = 35kHz (buoy A)\n");
    }

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

    /* Find the maximum value of the filtered signal */
    int maxA = findMax(output);
    
    /***************************************************************************
    ********************* Filtering for f = 40kHz (from buoy B) ****************
    ***************************************************************************/
    if(UALS_DEBUG) {
        printf("Beginning filtering for f = 40kHz (buoy B)\n");
    }
    /* Initialize FIR instance structure */
    arm_fir_init_f32(&S2, NUM_TAPS, (float *) &coeffsB[0], 
        &firStateF[0], blockSize);
    
    /* FIR Filtering */
    for( i = 0; i < numBlocks; i++) {
        arm_fir_f32(&S2, input + (i * blockSize), 
                        output + (i * blockSize), blockSize);
    }
    
    /* Take the absolute value of the filtered signal */
    for(i = 0; i < NUM_SAMPLES; i++ ) {
        if(output[i] < 0) {
            output[i] = -1 * output[i];    
        }
    } 
    
    /* Print before moving average */ 
    if(UALS_DEBUG) {
        printf("----------Before moving average-------------\n");
        for(i = 0; i < NUM_SAMPLES; i++) {
            printf("%lf\n", output[i]);
        }
    }
    
    
    /* Perform moving average */
    movAvg(output);
   
    /* Print signal after moving average */ 
    if(UALS_DEBUG) {
        printf("----------After moving average-------------\n");
        for(i = 0; i < NUM_SAMPLES; i++) {
            printf("%lf\n", output[i]);
        }
    }
    
    /* Find the maximum value of the filtered signal */
    int maxB = findMax(output);


    /***************************************************************************
    ******************** Filtering for f = 45kHz (from buoy C) *****************
    ***************************************************************************/
    if(UALS_DEBUG) {
        printf("Beginning filtering for f = 45kHz (buoy C)\n");
    }
    /* Initialize FIR instance structure */
    arm_fir_init_f32(&S3, NUM_TAPS, (float *) &coeffsC[0], 
        &firStateF[0], blockSize);
    
    /* FIR Filtering */
    for( i = 0; i < numBlocks; i++) {
        arm_fir_f32(&S3, input + (i * blockSize), 
                        output + (i * blockSize), blockSize);
    }
    
    /* Take the absolute value of the filtered signal */
    for(i = 0; i < NUM_SAMPLES; i++ ) {
        if(output[i] < 0) {
            output[i] = -1 * output[i];    
        }
    } 
    
    /* Print before moving average */ 
    if(UALS_DEBUG) {
        printf("----------Before moving average-------------\n");
        for(i = 0; i < NUM_SAMPLES; i++) {
            printf("%lf\n", output[i]);
        }
    }
    
    
    /* Perform moving average */
    movAvg(output);
   
    /* Print signal after moving average */ 
    if(UALS_DEBUG) {
        printf("----------After moving average-------------\n");
        for(i = 0; i < NUM_SAMPLES; i++) {
            printf("%lf\n", output[i]);
        }
    }
    
    /* Find the maximum value of the filtered signal */
    int maxC = findMax(output);

    /* The time differences to be returned */
    /* TODO: should we take the absolute value? What does TDOA take in */
    float tdAB = maxB - maxA;
    float tdBC = maxB - maxC;
    float tdAC = maxC - maxA;

    printf("tdAB = %f\n", tdAB);
    printf("tdBC = %f\n", tdBC);
    printf("tdAC = %f\n", tdAC);

    toReturn[0] = tdAB;
    toReturn[1] = tdBC;
    toReturn[2] = tdAC;
}

/*
 * Function Name: singleThresholdDetection()
 * Function Declaration: int singleThresholdDetection(const float sampleData[],
															int startPosition)
 * Function Description: Calculates the first occurrence of a value over the 
 * threshold value and returns the position.
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
    
    for(i = startPosition; i < NUM_SAMPLES; i++) {
        if (sampleData[i] >= DETECT_THRESH) {
            return i;
        }
    }
    
    return  -1;
}


/* 
 * Function Name: movAvg();
 * Function Declaration: void movAvg(float signal[]);
 * Function Description: This function takes an input signal and does an in
 * place moving average algorithm. The window size is defined by MOV_AVG_WIND
 * in main.h. 
 *
 * This function serves a dual purpose of finding the position of the largest 
 * peak and returning that position.
 * 
 * Return Value: None
 *
 */
void movAvg(float signal[]) {
     int i = 0;
    
    int start = 0;
    int finish = 0;
    float total = 0;
    
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
        }

        // Rotating buffer to save the avg
        buffer[i%(MOV_AVG_WIND*2)] = total / (finish - start); 
            
    }
    for(int i = NUM_SAMPLES-MOV_AVG_WIND*2; i < NUM_SAMPLES; i++) { 
        signal[i] = buffer[i %(MOV_AVG_WIND*2)];
    }
    
    for(int i = 0; i < NUM_SAMPLES; i++) {
        printf("averagedSignal[i] = %f\n", signal[i]);
    }
}


/* 
 * Function Name: findMax();
 * Function Declaration: int findMax(float signal[]);
 * Function Description: This function takes an input signal and returns the
 * position of the maximum valued sample.
 *
 */
int findMax(float signal[]) {
    float maxVal = 0.0;
    int maxPosition = -1;
    int i;

	/* Go through each element searching for maximum */
    for(i = 0; i < NUM_SAMPLES; i++) {
        if(signal[i] > maxVal) {
			/* We've found a new max, replace */
            maxVal = signal[i];
            maxPosition = i;
        }
    }

    if(maxPosition > 0) {
        printf("Max position found at %d\n", maxPosition);
        return maxPosition;
    }
    else {
        printf("Error: no max found\n");
        return -1;
    }
}
