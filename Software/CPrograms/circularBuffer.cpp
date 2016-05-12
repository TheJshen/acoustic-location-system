#include <stdio.h>
#include <stdlib.h>


typedef struct {
	float *buffer;
	int length;
	int start;
	int end;
	int count;
} CircBuffer;


CircBuffer * circBufferCreate(int num_samples) {
	// Allocate space for circular buffer
	CircBuffer * cb = calloc(1, sizeof(CircBuffer));
	
	// Set buffer parameters
	cb->buffer = calloc(1, num_samples);
	cb->length = num_samples;
	cb->start = 0;
	cb->end = 0;
	cb->count = 0;

	return cb;
}

void circBufferDelete(CircBuffer * cb) {
	if(cb) {
		// Free memory associated with circular buffer
		free(cb->buffer);
		free(cb);
	}
}

void circBufferWrite(CircBuffer * cb, float sample, int length) {
	
	if((cb->count) < length) {
		// Buffer not full, set data and increment count
		cb->buffer[cb->count] = sample;
		cb->count++;
	} else {
		// Buffer full, restart start
		cb->count = 0;
		cb->buffer[cb->count] = sample;
	}
}

int main() {
	CircBuffer* cb = circBufferCreate(10);
	
	for(int i = 0; i < 15; i++) {
		circBufferWrite(cb, i * 1.0f, 10);
	}
	
	for(int i = 0; i < 10; i++) {
		printf("CircBuff[%d] = %lf\n", i, cb->buffer[i]);
	}
}
