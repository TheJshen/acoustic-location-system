#include "mbed.h"
#include "Stopwatch.h"
 
 
#define SAMPLES 1000

AnalogIn analog_value(A0);


int main() {
  
    int i;   
    int count = 0;

    Stopwatch sw;
    sw.start();
    Timer timer;
    
    timer.start();

    for(int i = 0; i < SAMPLES; i++) {
        printf("%lf\n",analog_value.read());
    }

	timer.stop();
}
