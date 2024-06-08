#include <Arduino.h>
#include "RFM69HCW.h"

#define CALLSIGN "KC3UTM"
#define TOCALL "APRS"
#define PATH "WIDE1-1"

APRSConfig config = {CALLSIGN, TOCALL, PATH, '[', '/'};
RadioSettings settings = {433.775, true, false, &hardware_spi, 10, 2, 9, 8, 4, 3};
RFM69HCW transmit = {&settings, &config};

void setup()
{
    Serial.begin(9600);
    while (!Serial)
        ;

    if (!transmit.begin())
        Serial.println("Transmitter failed to begin");

    Serial.println("RFM69t began");
}

void loop()
{
    delay(2000);
    transmit.send("39.336896667,-77.337067833,480,0,31,H,0,11:40:51", ENCT_TELEMETRY);
}