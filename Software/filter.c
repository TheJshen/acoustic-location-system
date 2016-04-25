#define SAMP_FREQ   	(300000)  //300kHz
#define TRANS_PERIOD 	(100)     //100ms
#define NUM_SAMPLES     ((SAMP_FREQ) * (TRANS_PERIOD) / 1000)


/* Function: singleThresholdDetection()
 * Description: calculates the first occurence of a value over the threshold 
 * value and returns it.
 *
 * Params:
 * 	arg1: sample_data - sample data to find position of first peak
 * 	arg2: peak_threshold - threshold value to search for
 *
 * Return Value: position where threshold is first reached
 *
 */
int singleThresholdDetection( float sample_data[NUM_SAMPLES], float peak_threshold) { 
	int i;
	int position = 0;

	for(i = 0; i < NUM_SAMPLES; i++) {
		if (sample_data[i] >= peakThreshold) {
			position = i;
			break;
		}
	}
	
	return position;
}
