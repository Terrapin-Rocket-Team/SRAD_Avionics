#include <Arduino.h>
#include "APRSMsg.h"
#include "RFM69HCW.h"
#include "Radio.h"

APRSConfig config = {"KC3UTM", "APRS", "WIDE1-1", '[', '/'};
RadioSettings settings = {433.775, false, false, &hardware_spi, 10, 31, 32};
RadioSettings settings2 = {915.775, false, false, &hardware_spi, 10, 31, 32};
RadioSettings settings3 = {915.555, false, false, &hardware_spi, 10, 31, 32};
RFM69HCW radio1 = {settings, config};
RFM69HCW radio2 = {settings2, config};
RFM69HCW radio3 = {settings2, config};

int lastCycle = 1;
RFM69HCW *curSystem = &radio1;

void setup() {

    Serial.begin(6000000);      // 6Mbps baud rate

    radio1.begin();
    radio2.begin();
    radio3.begin();

    lastCycle = Serial.read();  

}

void loop() {

    int phase = Serial.read();

    if (phase != -1 && phase != lastCycle) {
        lastCycle = phase;

        if (phase == 1) {
            curSystem = &radio1;
        } else if (phase == 2) {
            curSystem = &radio2;
        } else {
            curSystem = &radio3;
        }
    }

    const char *msg = curSystem->rx();

    if (strlen(msg) > 0) {

        // call .decode() if needed
        Serial.print(msg);
    }



}
