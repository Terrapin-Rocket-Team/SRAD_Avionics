#include <Arduino.h>

#include "RFM69HCW.h"

#define MSG_SIZE 1250
#define TX_ADDR 0x02
#define RX_ADDR 0x01

// Serial communication with Pi
char buf[MSG_SIZE * 2];
unsigned int num = 0;
int loc = 0;
int newBytes = 0;

// radio variables
bool hasTransmission = false;
char msg[MSG_SIZE] = {0};
unsigned int pos = 0;

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
        if (loc == MSG_SIZE * 2)
            loc = 0;
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
            Serial.write(settings.spi->transfer(0));
            // newBytes++;
            // if (newBytes == MSG_SIZE * 2)
            //   newBytes = 0;
            pos++;
            // num++;
            Serial.flush();
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
            // receive.radio.setModeIdle();
            // memcpy(buf, buf + loc, num);
            // loc = 0;
            // memcpy(buf + num, msg, MSG_SIZE);
            // Serial.write(msg[MSG_SIZE - 1]);
            // Serial.flush();
            // num += MSG_SIZE;
        }
    }

    // Start a new transmission
    if (receive.FifoNotEmpty() && !hasTransmission && receive.radio.mode() == RHGenericDriver::RHModeRx)
    {
        // Serial.println("started receiving");
        // timer = millis();
        // Start spi transaction
        settings.spi->beginTransaction();
        // Select the radio
        digitalWrite(settings.cs, LOW);

        // Send the fifo address with the write mask off
        settings.spi->transfer(RH_RF69_REG_00_FIFO);

        // Preserve the state of msgLen and toAddr in case we don't care about this message
        // Get the message length
        int l = settings.spi->transfer(0);

        if (l * 10 == MSG_SIZE) // do this one at a time so we don't miss anything
        {
            // Get the radio type that is supposed to receive this message
            while (!receive.FifoNotEmpty()) // temporary solution, wait here for address
                ;
            int a = settings.spi->transfer(0); // do this one at a time so we don't miss anything

            // check address
            if (a == RX_ADDR)
            {
                // set up for receiving the message
                pos = 0;
                hasTransmission = true;

                while (pos < MSG_SIZE && receive.FifoNotEmpty())
                {
                    Serial.write(settings.spi->transfer(0));
                    // newBytes++;
                    // if (newBytes == MSG_SIZE * 2)
                    //   newBytes = 0;
                    pos++;
                    // num++;
                    Serial.flush();
                }
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
            // memcpy(buf, buf + loc, num);
            // loc = 0;
            // memcpy(buf + num, msg, MSG_SIZE);
            // num += MSG_SIZE;
        }
        // if (!hasTransmission)
        //   receive.radio.setModeIdle();
    }

    // if (num > MSG_SIZE + MSG_SIZE / 2)
    //   Serial.println("Buffer getting too long");
}