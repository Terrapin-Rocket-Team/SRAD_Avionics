#include <Arduino.h>
#include "RFM69HCW.h"

#define CALLSIGN "KC3UTM"
#define TOCALL "APRS"
#define PATH "WIDE1-1"

APRSConfig config = {CALLSIGN, TOCALL, PATH, '[', '/'};
RadioSettings settings = {433.775, false, false, &hardware_spi, 10, 2, 9, 0, 8, 3};
RFM69HCW receive = {&settings, &config};

void setup()
{
    Serial.begin(9600);
    while (!Serial)
        ;

    if (!receive.begin())
        Serial.println("Reciever failed to begin");

    Serial.println("RFM69r began");
}

void loop()
{
    if (receive.available())
    {
        Serial.println(receive.receive(ENCT_GROUNDSTATION));
        receive.rxs();
    }
}