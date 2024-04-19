#include <Arduino.h>
#include <SPI.h>

// put function declarations here:
byte *buf;
int loc = 0;

void setup() {

    buf = new byte[300000];   // 300k bytes
    Serial1.begin(460800);     // 460800 baud
    Serial.begin(115200);

}

void loop()
{

    if (Serial1.available() > 0) {
        buf[loc] = Serial1.read();
        loc++;
    }

    // Serial.print(millis());
    // Serial.print("--------");
    // Serial.println(buf[loc-1]);


    // SPI interrupt
    if (loc == interrupt) {
        // send the buffer over SPI
        SPI.transfer(buf, loc);
        loc = 0;
    }

}
