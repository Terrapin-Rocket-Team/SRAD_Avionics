#include <Arduino.h>

#include "RFM69HCW.h"

#define MSG_SIZE 25000
#define TX_ADDR 0x02
#define RX_ADDR 0x01
#define SYNC1 0x00
#define SYNC2 0xff

void radioIdle();
void radioRx();

// radio variables
bool hasTransmission = false;
bool modeReady = false;
bool modeIdle = true;

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
    // Serial1.begin(460800);

    if (!receive.begin())
        Serial.println("Receiver failed to begin");

    // Serial.println("RFM69r began");
}

void loop()
{

    // Refill fifo here
    if (receive.FifoNotEmpty() && hasTransmission)
    {
        // Start spi transaction
        settings.spi->beginTransaction();
        // Select the radio
        digitalWrite(settings.cs, LOW);

        // Send the fifo address with the write mask off
        settings.spi->transfer(RH_RF69_REG_00_FIFO);

        // data
        while (pos < MSG_SIZE && receive.FifoNotEmpty())
        {
            Serial.write(settings.spi->transfer(0));
            pos++;
        }

        // reset msgLen and toAddr, end transaction, and clear fifo through entering idle mode
        digitalWrite(settings.cs, HIGH);
        settings.spi->endTransaction();

        if (pos == MSG_SIZE)
        {
            // Serial.println("finished");
            pos = 0;
            hasTransmission = false;
            radioIdle();
        }
    }

    // Start a new transmission
    if (receive.FifoNotEmpty() && !hasTransmission && !modeIdle && modeReady)
    {
        // Serial.println("started receiving");
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

        if (!hasL)
        {
            hasL = settings.spi->transfer(0) == SYNC1;
        }

        if (!hasA && hasL && receive.FifoNotEmpty())
        {
            hasL = hasA = settings.spi->transfer(0) == SYNC2;
            if (!hasA)
            {
                // radioIdle();
            }
        }

        // if found sync bytes
        if (hasL && hasA)
        {
            // set up for receiving the message
            hasTransmission = true;
            hasL = hasA = false;

            while (pos < MSG_SIZE && receive.FifoNotEmpty())
            {
                Serial.write(settings.spi->transfer(0));
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
            radioIdle();
        }
    }

    if ((receive.radio.spiRead(RH_RF69_REG_27_IRQFLAGS1) & RH_RF69_IRQFLAGS1_MODEREADY) && !modeReady)
    {
        modeReady = true;
        if (modeIdle)
            radioRx();
    }
}

// adapted from radioHead functions
void radioIdle()
{
    modeReady = false;
    modeIdle = true;
    receive.radio.spiWrite(RH_RF69_REG_5A_TESTPA1, RH_RF69_TESTPA1_NORMAL);
    receive.radio.spiWrite(RH_RF69_REG_5C_TESTPA2, RH_RF69_TESTPA2_NORMAL);
    uint8_t opmode = receive.radio.spiRead(RH_RF69_REG_01_OPMODE);
    opmode &= ~RH_RF69_OPMODE_MODE;
    opmode |= (RH_RF69_OPMODE_MODE_STDBY & RH_RF69_OPMODE_MODE);
    receive.radio.spiWrite(RH_RF69_REG_01_OPMODE, opmode);
}

void radioRx()
{
    modeReady = false;
    modeIdle = false;
    receive.radio.spiWrite(RH_RF69_REG_5A_TESTPA1, RH_RF69_TESTPA1_NORMAL);
    receive.radio.spiWrite(RH_RF69_REG_5C_TESTPA2, RH_RF69_TESTPA2_NORMAL);
    receive.radio.spiWrite(RH_RF69_REG_25_DIOMAPPING1, RH_RF69_DIOMAPPING1_DIO0MAPPING_01); // Set interrupt line 0 PayloadReady
    uint8_t opmode = receive.radio.spiRead(RH_RF69_REG_01_OPMODE);
    opmode &= ~RH_RF69_OPMODE_MODE;
    opmode |= (RH_RF69_OPMODE_MODE_RX & RH_RF69_OPMODE_MODE);
    receive.radio.spiWrite(RH_RF69_REG_01_OPMODE, opmode);
}