/* Includes */
#include <stdio.h> //fprintf, FILE
#include <stdlib.h> //
#include <math.h> //sqrt

#include "mbed.h"
#include "main.h"

#include "arm_math.h"
//#include "math_helper.h"

#include "float.h"

//#include "tiny_printf.c"

#define BLOCK_SIZE      (32)

static float firStateF[BLOCK_SIZE + NUM_TAPS - 1];

unsigned int blockSize = BLOCK_SIZE;
unsigned int numBlocks = NUM_SAMPLES/BLOCK_SIZE;


/* Private macro */


/* Private variables */
//float signalA[NUM_SAMPLES];
//float signalB[NUM_SAMPLES];
//float signalC[NUM_SAMPLES];

FILE* logFile;

typedef struct coordinate {
    float x;
    float y;
} coord;

/* Function Prototypes */
float calculateSoundSpeedInWater(float temp, float salinity, float depth);

void createSignalsFromFile(void);

void movingAverage(float signal[NUM_SAMPLES], float * averagedSignal[NUM_SAMPLES]);

int singleThresholdDetection(const float sampleData[NUM_SAMPLES], int startPosition);

void getPositionFromSignal(
    float signal[NUM_SAMPLES], coord* projCoord,
    float velocity, float depth,
    float xa, float ya, float za,
    float xb, float yb, float zb,
    float xc, float yc, float zc);

void filterAndSmooth(float signal[NUM_SAMPLES], float filterCoeffs[NUM_TAPS], int smoothingCoeff, float * filteredSignal);

void calculateProjectedCoordinates(
    coord * projCoord,
    float depth, float velocity,
    float tsa, float tsb, float tsc,
    float xa, float ya, float za,
    float xb, float yb, float zb,
    float xc, float yc, float zc);
    

//------------------------------------
// Hyperterminal configuration
// 9600 bauds, 8-bit data, no parity
//------------------------------------

Serial pc(SERIAL_TX, SERIAL_RX);

DigitalOut myled(LED1);

int main(void)
{
    int i = 0;

    /* TODO - Add your application code here */
    //FILE* fp = fopen("log.txt", "w");
    //if(fp == 0) 
    //   pc.printf("NULL\n");
    //fclose(fp);

    //createSignalsFromFile();

    /*
    float temp = TEMP;
    float depth = DEPTH;
    float salinity = SALINITY;

    float velocity = calculateSoundSpeedInWater(temp, salinity, depth);

    pc.printf("The velocity is %lf", velocity);


    int firstPeak = singleThresholdDetection(signalA, 0);
    
    pc.printf("First peak found at position %d\n", firstPeak);
    pc.printf("Time of first peak (in ms): %lf", firstPeak / 250.0);
    */
    
    arm_fir_instance_f32 S;
    arm_status status;
    float *input, *output;
    
    input = &signalABC[0];
    
    /* Initialize FIR instance structure */
    arm_fir_init_f32(&S, NUM_TAPS, (float *) &coeffsA[0], &firStateF[0], blockSize);
    
    /* FIR Filtering */
    for( i = 0; i < numBlocks; i++) {
        arm_fir_f32(&S, input + (i * blockSize), output + (i * blockSize), blockSize);
    }
    
    /* Take the absolute value of the filtered signal */
    for(i = 0; i < NUM_SAMPLES; i++ ) {
        printf("Index: %d, Amplitude: %e\n", i, output[i]);
        if(output[i] < 0.0f) {
            printf("Negative sample!\n");
            output[i] = -1 * output[i];    
        }
    }
    
    /*
    int firstFilteredPeak = singleThresholdDetection(output, 0);
    
    pc.printf("First filtered peak found at position %d\n", firstFilteredPeak);
    pc.printf("Time of first peak (in ms): %lf", firstFilteredPeak / 250.0);
    */
    
    
    float * avgSignal[NUM_SAMPLES];
    
    movingAverage(output, avgSignal);
    
    for(i = 0; i < NUM_SAMPLES; i++) {
        printf("%lf\n", avgSignal[i]);
    }
}

void movingAverage(float signal[NUM_SAMPLES], float ** averagedSignal) {
    int i = 0;
    
    int start = 0;
    int finish = 0;
    float total = 0;
    
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
        for(int j = start; j < finish; i++) {
            total += signal[j];
        }
        
        *averagedSignal[i] = total / (finish - start);        
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
int singleThresholdDetection(const float sampleData[NUM_SAMPLES], int startPosition)
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
 * Function: getPositionFromSignal()
 * Description: TODO
 *
 * Params: TODO
 *
 *
 * Return Value: none
 *
 */
 /*
void getPositionFromSignal(
    float signal[NUM_SAMPLES], coord* projCoord,
    float velocity, float depth,
    float xa, float ya, float za,
    float xb, float yb, float zb,
    float xc, float yc, float zc)
{

    //TODO uncomment the following lines of code once filtering is working
    //TODO can be tuned
    //int smoothingCoeff = 20;

    //float extracted1[NUM_SAMPLES];
    //float extracted2[NUM_SAMPLES];
    //float extracted3[NUM_SAMPLES];


    //filterAndSmooth(signal, filter1Coeffs, smoothingCoeff, &extracted1);
    //filterAndSmooth(signal, filter2Coeffs, smoothingCoeff, &extracted2);
    //filterAndSmooth(signal, filter3Coeffs, smoothingCoeff, &extracted3);

    //int tsa = singleThresholdDetection(extracted1, 0);
    //int tsb = singleThresholdDetection(extracted2, 0);
    //int tsc = singleThresholdDetection(extracted3, 0);


    int tsa = singleThresholdDetection(signalA, 0);
    int tsb = singleThresholdDetection(signalB, 0);
    int tsc = singleThresholdDetection(signalC, 0);

    if(tsa != -1 )

        calculateProjectedCoordinates(projCoord, depth, velocity, tsa, tsb, tsc,
                                      xa, ya, za,
                                      xb, yb, zb,
                                      xc, yc, zc);
}
*/

void filterAndSmooth(float signal[NUM_SAMPLES], float filterCoeffs[NUM_TAPS], int smoothingCoeff, float * filteredSignal)
{
    //Low pass filter
    //arm_fir_instance_f32 * firInstance;

    //arm_fir_init_f32(firInstance, NUM_TAPS,
    //Moving average

    //return signal

}


/*
 * Function: calculateProjectedCoordinates()
 * Description: TODO
 *
 * Params: TODO
 *
 *
 * Return Value: none
 *
 */
void calculateProjectedCoordinates(
    coord * projCoord,
    float depth, float velocity,
    float tsa, float tsb, float tsc,
    float xa, float ya, float za,
    float xb, float yb, float zb,
    float xc, float yc, float zc)
{

    float dsa = velocity * tsa;
    float dsb = velocity * tsb;
    float dsc = velocity * tsc;

    float dsaProjected = sqrt(dsa * dsa - depth * depth);
    float dsbProjected = sqrt(dsb * dsb - depth * depth);
    float dscProjected = sqrt(dsc * dsc - depth * depth);

    float xCoord =  (dsaProjected * dsaProjected - dsbProjected * dsbProjected + xb * xb)/(2 * xb);
    float yCoord = ((dsaProjected * dsaProjected - dscProjected * dscProjected + xc * xc + yc * yc)/(2 * yc)) - ((xc) / (yc)) * xCoord;

    projCoord->x = xCoord;
    projCoord->y = yCoord;
}

