#include <Arduino.h>

#include "RFM69HCW.h"

#define MSG_SIZE 37500
#define TX_ADDR 0x02
#define RX_ADDR 0x01
#define SYNC1 0x37
#define SYNC2 0x69

// Serial communication with Pi
char buf[MSG_SIZE * 2];
unsigned int num = 0;
int loc = 0;

// radio variables
bool hasTransmission = false;
char msg[MSG_SIZE] = {0};
unsigned int pos = 0;

bool hasL = false;
bool hasA = false;

// testing
// uint32_t timer = millis();

// radio
APRSConfig config = {"KC3UTM", "APRS", "WIDE1-1", '[', '/'};
RadioSettings settings = {433.775, false, true, &hardware_spi, 10, 15, 14, 7, 8, 9};
RFM69HCW receive = {&settings, &config};

void setup()
{
    Serial.begin(460800);

    if (!receive.begin())
        Serial.println("Receiver failed to begin");

    // Serial.println("RFM69r began");

    receive.radio.setModeIdle();
}

void loop()
{
    if (receive.radio.mode() != RHGenericDriver::RHModeRx)
        receive.radio.setModeRx();

    // writing to serial
    if (num > 0)
    {
        // Serial.write(buf[loc]);
        loc++;
        num--;
    }

    // Refill fifo here
    if (hasTransmission && receive.radio.mode() == RHGenericDriver::RHModeRx)
    {
        // Start spi transaction
        settings.spi->beginTransaction();
        // Select the radio
        digitalWrite(settings.cs, LOW);

        // Send the fifo address with the write mask off
        settings.spi->transfer(RH_RF69_REG_00_FIFO);
        while (pos < MSG_SIZE && receive.FifoNotEmpty())
        {
            msg[pos] = settings.spi->transfer(0);
            Serial.write(msg[pos]);
            pos++;
        }

        // reset msgLen and toAddr, end transaction, and clear fifo through entering idle mode
        digitalWrite(settings.cs, HIGH);
        settings.spi->endTransaction();
        // go back to idle mode if addr and len did not match
        if (pos == MSG_SIZE)
        {
            // Serial.println("finished");
            pos = 0;
            hasTransmission = false;
            receive.radio.setModeIdle();
            memcpy(buf, buf + loc, num);
            loc = 0;
            memcpy(buf + num, msg, MSG_SIZE);
            // Serial.write(msg[MSG_SIZE - 1]);
            // Serial.flush();
            num += MSG_SIZE;
        }
    }

    // Start a new transmission
    if (receive.FifoNotEmpty() && !hasTransmission && receive.radio.mode() == RHGenericDriver::RHModeRx)
    {
        // timer = millis();
        // Start spi transaction
        settings.spi->beginTransaction();
        // Select the radio
        digitalWrite(settings.cs, LOW);

        // Send the fifo address with the write mask off
        settings.spi->transfer(RH_RF69_REG_00_FIFO);
        // pos = 0;

        // while (pos < MSG_SIZE)
        // {
        //   if (receive.FifoNotEmpty())
        //   {
        //     Serial.write(settings.spi->transfer(0));
        //     pos++;
        //   }
        // }

        // Preserve the state of msgLen and toAddr in case we don't care about this message
        // Get the message length
        if (!hasL)
            hasL = settings.spi->transfer(0) == SYNC1;
        // Get the radio type that is supposed to receive this message
        if (!hasA && hasL && receive.FifoNotEmpty())
        {
            hasA = settings.spi->transfer(0) == SYNC2;
            if (!hasA)
            {
                hasL = false;
                receive.radio.setModeIdle();
            }
        }

        // check address
        if (hasL && hasA)
        {

            // set up for receiving the message
            pos = 0;
            hasTransmission = true;
            hasL = hasA = false;

            while (pos < MSG_SIZE && receive.FifoNotEmpty())
            {
                msg[pos] = settings.spi->transfer(0);
                Serial.write(msg[pos]);
                pos++;
            }
        }
        // reset msgLen and toAddr, end transaction, and clear fifo through entering idle mode
        digitalWrite(settings.cs, HIGH);
        settings.spi->endTransaction();

        if (pos == MSG_SIZE)
        {
            // Serial.println("finished 1");
            pos = 0;
            hasTransmission = false;
            receive.radio.setModeIdle();
            memcpy(buf, buf + loc, num);
            loc = 0;
            memcpy(buf + num, msg, MSG_SIZE);
            num += MSG_SIZE;
        }
    }

    // if (num > MSG_SIZE + MSG_SIZE / 2)
    //   Serial.println("Buffer getting too long");
}