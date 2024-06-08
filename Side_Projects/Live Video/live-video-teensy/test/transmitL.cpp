#include <Arduino.h>

#include "RFM69HCW.h"

#define MSG_SIZE 1250
#define TX_ADDR 0x02
#define RX_ADDR 0x01

// Serial communication with Pi
char *buf;
unsigned int loc = 0;

// radio variables
bool hasTransmission = false;
char msg[MSG_SIZE] = {0};
unsigned int pos = 0;

bool continueTx = false;

// testing
// uint32_t timer = millis();
// end testing

// radio
APRSConfig config = {"KC3UTM", "APRS", "WIDE1-1", '[', '/'};
RadioSettings settings = {433.775, true, true, &hardware_spi, 10, 15, 14, 8, 4, 9};
RFM69HCW transmit = {&settings, &config};

void setup()
{
    Serial.begin(9600);

    buf = new char[MSG_SIZE * 30]; // 30 frames
    Serial1.begin(460800);         // 460800 baud

    if (!transmit.begin())
        Serial.println("Transmitter failed to begin");

    Serial.println("RFM69t began");

    transmit.radio.setModeIdle();
}

void loop()
{

    // reading from Raspi
    if (Serial1.available() > 0)
    {
        buf[loc] = Serial1.read();
        loc++;
    }

    // Refill fifo here
    if (hasTransmission && transmit.radio.mode() == RHGenericDriver::RHModeTx)
    {

        // Start spi transaction
        settings.spi->beginTransaction();
        // Select radio
        digitalWrite(settings.cs, LOW);

        settings.spi->transfer(RH_RF69_REG_00_FIFO | RH_RF69_SPI_WRITE_MASK);

        int sentBytes = 0;

        // Send the next section of the payload
        while (pos < MSG_SIZE && !transmit.FifoFull())
        {
            settings.spi->transfer(buf[sentBytes]);
            sentBytes++;
            pos++;
        }

        // Deselect radio
        digitalWrite(settings.cs, HIGH);
        // End spi transaction
        settings.spi->endTransaction();

        memcpy(buf, buf + sentBytes, loc - sentBytes);
        loc = loc - sentBytes;

        // Serial.println(loc);

        // Check if message is finished
        if (pos == MSG_SIZE)
        {
            pos = 0;
            hasTransmission = false;
            if (loc < MSG_SIZE)
            {
                while (!digitalRead(settings.irq))
                    ;
                transmit.radio.setModeIdle();
                // Serial.println("finished");
            }
            else
            {
                continueTx = true;
            }
        }
    }

    // Start a new transmission
    if ((loc >= MSG_SIZE /*|| millis() - timer > 35*/) && !hasTransmission && (transmit.radio.mode() != RHGenericDriver::RHModeTx || continueTx))
    {
        continueTx = false;
        // Setup
        Serial.println(loc);
        // memcpy(msg, buf, bytesToTx);
        hasTransmission = true;

        // Start spi transaction
        settings.spi->beginTransaction();
        // Select radio
        digitalWrite(settings.cs, LOW);

        // Select the fifo for writing
        settings.spi->transfer(RH_RF69_REG_00_FIFO | RH_RF69_SPI_WRITE_MASK);
        // Send message length
        settings.spi->transfer(MSG_SIZE / 10U);
        // Send address for receving radio
        settings.spi->transfer(RX_ADDR);

        int sentBytes = 0;

        // Send the payload
        while (pos < MSG_SIZE && !transmit.FifoFull())
        {
            settings.spi->transfer(buf[sentBytes]);
            sentBytes++;
            pos++;
        }

        // Deselect radio
        digitalWrite(settings.cs, HIGH);
        // End spi transaction
        settings.spi->endTransaction();

        // Start the transmitter
        if (pos != 0 && transmit.radio.mode() != RHGenericDriver::RHModeTx)
        {
            transmit.radio.setModeTx();
        }
        else
        {
            Serial.println("No bytes transfered");
        }

        memcpy(buf, buf + sentBytes, loc - sentBytes);
        loc = loc - sentBytes;

        // Serial.println(loc);

        // Check if message is finished
        if (pos == MSG_SIZE)
        {
            pos = 0;
            hasTransmission = false;
            if (loc < MSG_SIZE)
            {
                while (!digitalRead(settings.irq))
                    ;
                transmit.radio.setModeIdle();
                // Serial.println("finished");
            }
            else
            {
                continueTx = true;
            }
        }
    }
}