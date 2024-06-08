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
int bytesToSend = 0;
int bytesSent = 0;
int bytesReceived = 0;

bool receiveFlag = false;
bool s1Flag = false;
bool s2Flag = false;

#ifdef DS_DEBUG
uint32_t debugTimer = millis();
#endif

// radio
APRSConfig config = {"KC3UTM", "APRS", "WIDE1-1", '[', '/'};
RadioSettings settings = {433.775, false, true, &hardware_spi, 10, 15, 14, 8, 4, 9};
RFM69HCW receive = {&settings, &config};

void setup()
{
    // Serial.begin(460800); // 460800 baud
    Serial.begin(9600);

    if (!receive.begin())
        Serial.println("Receiver failed to begin");

    Serial.println("RFM69r began");

    receive.radio.setModeRx();

    settings.spi->beginTransaction();
    digitalWrite(settings.cs, LOW);

    settings.spi->transfer(RH_RF69_REG_00_FIFO);
}

void loop()
{
    // send bytes into serial from buf
    if (bytesToSend > 0)
    {
        // Serial.write(buf[bytesSent]);
        bytesToSend--;
        bytesSent++;

        if (bytesSent == MSG_SIZE)
        {
            memcpy(buf, buf + bytesSent, bytesToSend);
            if (bytesReceived > 0)
                bytesReceived -= bytesSent;
            bytesSent = 0;
        }
    }

    // read bytes if we have some to read
    if (receiveFlag && receive.FifoNotEmpty())
    {
        if (s1Flag && s2Flag)
        {
            buf[bytesReceived] = settings.spi->transfer(0);
            bytesReceived++;
        }

        if (bytesReceived == MSG_SIZE)
        {
            bytesToSend += MSG_SIZE;
            bytesReceived = 0;
            receiveFlag = s1Flag = s2Flag = false;
        }
    }

    // start reading after not sending data for a while
    Serial.println(receive.FifoNotEmpty());
    if (receive.FifoNotEmpty() && !receiveFlag)
    {
        // do this in reverse order so we don't end up reading too many bytes from the fifo
        if (s1Flag && s2Flag)
        {
            bytesReceived = bytesSent + bytesToSend; // pointer to where new data should go
            buf[bytesReceived] = settings.spi->transfer(0);
            bytesReceived++;
        }

        if (s1Flag && !s2Flag)
        {
            s2Flag = settings.spi->transfer(0) == SYNC2;
            s1Flag = s2Flag; // if s2flag is false, we want s1flag to also be false and vis versa
        }
        else if (!s1Flag)
        {
            s1Flag = settings.spi->transfer(0) == SYNC1;
        }

        Serial.print(s1Flag);
        Serial.print(" ");
        Serial.println(s2Flag);
    }

#ifdef DS_DEBUG
    if (millis() - debugTimer > 1000)
    {
        debugTimer = millis();
        Serial.println("\nBuffer state:");
        Serial.print("bytesToSend\t");
        Serial.println(bytesToSend);
        Serial.print("bytesSent\t");
        Serial.println(bytesSent);
        Serial.print("bytesReceived\t");
        Serial.println(bytesReceived);
    }
#endif
}