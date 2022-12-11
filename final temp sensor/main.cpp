#include "mbed.h"
#include <stdint.h>
#include "DS18B20.h"

DigitalInOut sensor(D8);     // sensor connected to pin D8
DigitalOut green(D2);
DigitalOut red(D3);

Ticker timer;                // used for our microsec timing
Serial pc(D1, D0);     // serial comms back to console

float FtoC(float f){
    return (5 * (f - 32) / 9);
}

int main() {
    pc.printf("\n\r===Liquid Temperature Sensor===\n\r");
    sensor.mode(PullUp);

    ROM_Code_t ROM_Code = ReadROM();

    pc.printf("\nCelsius or Farenheit(Enter: C or F):\t");
    char c;
    pc.scanf("%c", &c);
    pc.printf("%c", c);

    pc.printf("\nEnter Desired Temperature:\t");
    float r;
    pc.scanf("%f", &r);
    pc.printf("%f", r);

    if(c == 'F' || c == 'f'){//convert to celcius
        r = FtoC(r);
    }
     pc.printf("\n");

    while (1) {
        uint32_t temp = GetTemperature();
        float f = (temp & 0x0F) * 0.0625;    // calculate .4 part
        f += (temp >> 4);
       
        displayTemperature(pc);

        if(f > 47){
            green = 0;
            red = 1;
        }
        else if (int(f) == int(r)){
            green = 1;
            red = 0;
        }
        else if ((f > r && f < r + 2.0) || (f < r && f > r - 2.0)){
            green = 1;
            red = 1;
        }
        else{
            green = 0;
            red = 1;
        }

        wait(.5);
    }
}
