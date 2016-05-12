#include "mbed.h"

AnalogOut my_output(PA_4);

#define PI        (3.141592653589793238462)
#define AMPLITUDE (0.7)    // x * 3.3V
#define PHASE     (PI * 1) // 2*pi is one period
#define RANGE     (0x7FFF)
#define OFFSET    (0x7FFF)

// Configuration for sinewave output
#define BUFFER_SIZE (360) // 360 samples per cycles

uint16_t buffer[BUFFER_SIZE];

void calculate_sinewave(void);

// The wave that should be generate should be 2777Hz

//Will need to modify DAC to use the higher performance version to be able to
//generate the high frequency sine waves needed
int main() {
    printf("Sinewave example\n"); // example sine wave
    calculate_sinewave();
    while(1) {      
        // sinewave output
        for (int i = 0; i < BUFFER_SIZE; i++) {
            my_output.write_u16(buffer[i]);
            wait_us(1); // 1 sample update per microsecond
        }
    }
}

// Create the sinewave buffer
void calculate_sinewave(void){
  for (int i = 0; i < BUFFER_SIZE; i++) {
     double rads = (PI * i)/180.0; // Convert degree in radian
     //double rads = (PI * i)/8.0;
     buffer[i] = (uint16_t)(AMPLITUDE * (RANGE * (cos(rads + PHASE))) + OFFSET);
     // fills the buffer with the 360 distinct points on the sine graph
  }
}
