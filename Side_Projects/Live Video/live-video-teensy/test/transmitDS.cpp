#include <Arduino.h>

#include "RFM69HCW.h"

#define MSG_SIZE 1250
#define TX_ADDR 0x02
#define RX_ADDR 0x01
#define SYNC1 0x37
#define SYNC2 0x69

#define DS_DEBUG

// Serial communication with Pi
char buf[MSG_SIZE * 3];
int bytes = 0;
int bytesToSend = 0;
int bytesSent = 0;

bool sendFlag = false;

uint32_t txTimeout = millis();

#ifdef DS_DEBUG
uint32_t debugTimer = millis();
#endif

// radio
APRSConfig config = {"KC3UTM", "APRS", "WIDE1-1", '[', '/'};
RadioSettings settings = {433.775, true, true, &hardware_spi, 10, 15, 14, 8, 4, 9};
RFM69HCW transmit = {&settings, &config};

void setup()
{
    Serial.begin(9600);
    Serial1.begin(460800); // 460800 baud

    if (!transmit.begin())
        Serial.println("Transmitter failed to begin");

    Serial.println("RFM69t began");

    // Get initial message
    while (bytes < MSG_SIZE)
    {
        if (Serial1.available() > 0)
        {
            buf[bytes] = Serial.read();
            bytes++;
        }
    }

    // Send initial message

    bytesToSend = bytes;
    sendFlag = true;

    // Start spi transaction
    settings.spi->beginTransaction();
    // Select radio
    digitalWrite(settings.cs, LOW);

    settings.spi->transfer(RH_RF69_REG_00_FIFO | RH_RF69_SPI_WRITE_MASK);

    // add our own sync values so the receiver knows data is coming
    settings.spi->transfer(SYNC1);
    settings.spi->transfer(SYNC2);

    // start sending data
    while (bytesToSend != 0 && !transmit.FifoFull())
    {
        settings.spi->transfer(buf[MSG_SIZE - bytesToSend]);
        bytesToSend--;
        bytesSent++;
    }

    // Deselect radio
    digitalWrite(settings.cs, HIGH);
    // End spi transaction
    settings.spi->endTransaction();

    // set tx mode, but only need to do so the first time around
    if (transmit.radio.mode() != RHGenericDriver::RHModeTx)
        transmit.radio.setModeTx();

    // Begin another tranaction to reload fifo
    settings.spi->beginTransaction();
    digitalWrite(settings.cs, LOW);

    settings.spi->transfer(RH_RF69_REG_00_FIFO | RH_RF69_SPI_WRITE_MASK);
}

void loop()
{
    // read bytes into buf from serial
    if (Serial1.available() > 0 && bytes < MSG_SIZE * 3)
    {
        txTimeout = millis();
        buf[bytes] = Serial1.read();
        bytes++;
    }
    else if (bytes == MSG_SIZE * 3)
    {
        Serial.println("Possible buffer overrun!!");
    }

    // if we have MSG_SIZE bytes ready, add them to be sent
    if (bytes - bytesToSend >= MSG_SIZE)
        bytesToSend += MSG_SIZE;

    // send bytes if we have some to send
    if (sendFlag && bytesToSend != 0)
    {
        if (!transmit.FifoFull())
        {
            settings.spi->transfer(buf[MSG_SIZE - bytesToSend]);
            bytesToSend--;
            bytesSent++;
        }

        if (bytesSent == MSG_SIZE)
        {
            sendFlag = false;
            memcpy(buf, buf + bytesSent, bytesToSend);
            if (bytes > MSG_SIZE)
                bytes -= bytesSent;
            else
                bytes = 0;
            bytesSent = 0;
        }
    }

    // start sending after not sending data for a while
    if ((bytes >= MSG_SIZE || (millis() - txTimeout > 100 && bytes != 0)) && !sendFlag)
    {
        sendFlag = true;
        bytesToSend = MSG_SIZE;

        if (bytes < MSG_SIZE)
            memset(buf + bytes, 0, MSG_SIZE - bytes);

        // add our own sync values so the receiver knows data is coming
        settings.spi->transfer(SYNC1);
        settings.spi->transfer(SYNC2);

        // start sending data
        if (!transmit.FifoFull())
        {
            settings.spi->transfer(buf[MSG_SIZE - bytesToSend]);
            bytesToSend--;
            bytesSent++;
        }
    }

    // if we don't have anything to send
    if (bytesToSend == 0 && bytes < MSG_SIZE && !transmit.FifoFull())
    {
        // this switches between sending 0 and 1 bits
        settings.spi->transfer(0b01010101);
    }

#ifdef DS_DEBUG
    if (millis() - debugTimer > 100)
    {
        debugTimer = millis();
        Serial.println("\nBuffer state:");
        Serial.print("bytesToSend\t");
        Serial.println(bytesToSend);
        Serial.print("bytesSent\t");
        Serial.println(bytesSent);
        Serial.print("bytes\t\t");
        Serial.println(bytes);
    }
#endif
}