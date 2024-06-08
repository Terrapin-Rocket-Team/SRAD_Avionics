// rf69_server.cpp
//
// Example program showing how to use RH_RF69 on Raspberry Pi
// Uses the bcm2835 library to access the GPIO pins to drive the RFM69 module
// Requires bcm2835 library to be already installed
// http://www.airspayce.com/mikem/bcm2835/
// Use the Makefile in this directory:
// cd example/raspi/rf69
// make
// sudo ./rf69_server
//
// Contributed by Charles-Henri Hallard based on sample RH_NRF24 by Mike Poublon

// NOTE: must run with sudo or segmentation fault happens

#include <bcm2835.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>

#include "RFM69HCW.h"

#define MSG_SIZE 25000
#define MSG_THRESH 25000
#define TX_ADDR 0x02
#define RX_ADDR 0x01
#define SYNC1 0x00
#define SYNC2 0xff

void radioIdle();
void radioTx();

// Serial communication with Pi
char buf[MSG_SIZE * 3];

// radio variables
bool hasTransmission = false;
bool modeReady = true;

int top = 0;
int toSend = 0;
int sent = 0;
int bytesThisMessage = 0;

uint32_t txTimeout = millis();

// testing
uint32_t debugTimer = millis();
// end testing

// radio
APRSConfig config = {"KC3UTM", "APRS", "WIDE1-1", '[', '/'};
RadioSettings settings = {433.775, true, true, &hardware_spi, 17, 22, 27, 7, 8, 25};
RFM69HCW transmit = {&settings, &config};

// Flag for Ctrl-C
volatile sig_atomic_t force_exit = false;

void sig_handler(int sig)
{
    printf("\n%s Break received, exiting!\n", __BASEFILE__);
    force_exit = true;
}

// Main Function
int main(int argc, const char *argv[])
{

    // setup
    signal(SIGINT, sig_handler);
    printf("%s\n", __BASEFILE__);

    if (!bcm2835_init())
    {
        fprintf(stderr, "%s bcm2835_init() Failed\n\n", __BASEFILE__);
        return 1;
    }

    if (!transmit.begin())
        fprintf(stderr, "Transmitter failed to begin\n");

    char ch;

    // loop
    while (!force_exit)
    {
        while (read(STDIN_FILENO, &ch, 1) > 0 && top < MSG_SIZE * 3)
        {
            txTimeout = millis();
            buf[top] = ch;
            top++;

            if (bytesThisMessage + toSend < MSG_SIZE && hasTransmission)
            {
                toSend++;
            }
        }

        // Refill fifo here
        if (hasTransmission && toSend > 0 && !transmit.FifoFull())
        {
            // Start spi transaction
            // settings.spi->beginTransaction();
            // Select radio
            digitalWrite(settings.cs, LOW);

            settings.spi->transfer(RH_RF69_REG_00_FIFO | RH_RF69_SPI_WRITE_MASK);

            // Send the next section of the payload
            sent = 0;
            while (toSend > 0 && !transmit.FifoFull())
            {
                settings.spi->transfer(buf[sent]);
                sent++;
                bytesThisMessage++;
                toSend--;
            }
            memcpy(buf, buf + sent, top - sent);
            top -= sent;

            // Deselect radio
            digitalWrite(settings.cs, HIGH);
            // End spi transaction
            // settings.spi->endTransaction();
        }
        else if (hasTransmission && toSend <= 0 && bytesThisMessage < MSG_SIZE)
        {
            printf("Ran out of bits!\tbytesThisMessage ");
            printf("%i", bytesThisMessage);
            printf("\ttop ");
            printf("%i\n", top);
        }

        // Start a new transmission
        if ((top >= MSG_THRESH || (millis() - txTimeout > 100 && top > 0)) && !hasTransmission && modeReady)
        {
            hasTransmission = true;
            toSend = (top > MSG_SIZE) ? MSG_SIZE : top;

            // Start spi transaction
            // settings.spi->beginTransaction();
            // Select radio
            digitalWrite(settings.cs, LOW);

            // Select the fifo for writing
            settings.spi->transfer(RH_RF69_REG_00_FIFO | RH_RF69_SPI_WRITE_MASK);
            // Send message length
            settings.spi->transfer(SYNC1);
            // Send address for receving radio
            settings.spi->transfer(SYNC2);

            // Send the payload
            sent = 0;
            while (toSend > 0 && !transmit.FifoFull())
            {
                settings.spi->transfer(buf[sent]);
                sent++;
                bytesThisMessage++;
                toSend--;
            }
            memcpy(buf, buf + sent, top - sent);
            top -= sent;

            // Deselect radio
            digitalWrite(settings.cs, HIGH);
            // End spi transaction
            // settings.spi->endTransaction();

            radioTx();
        }

        if (millis() - txTimeout > 100 && top > 0)
        {
            top = 0;
            sent = 0;
        }

        if ((bytesThisMessage == MSG_SIZE || (millis() - txTimeout > 100 && toSend == 0)) && digitalRead(settings.irq))
        {
            bytesThisMessage = 0;
            hasTransmission = false;
            radioIdle();
            // printf("finished");
        }

        if (transmit.radio.spiRead(RH_RF69_REG_27_IRQFLAGS1) & RH_RF69_IRQFLAGS1_MODEREADY)
        {
            modeReady = true;
        }

        if (millis() - debugTimer > 100)
        {
            debugTimer = millis();
        }
        printf("\rBuffer state:\ttop %i\tsent %i\ttoSend %i\tbytesThisMessage %i", top, sent, toSend, bytesThisMessage);
    }
    // fclose(video);
    printf("\n%s Ending\n", __BASEFILE__);
    radioIdle();
    bcm2835_close();
    return 0;
}

// adapted from radioHead functions
void radioIdle()
{
    modeReady = false;
    transmit.radio.spiWrite(RH_RF69_REG_5A_TESTPA1, RH_RF69_TESTPA1_NORMAL);
    transmit.radio.spiWrite(RH_RF69_REG_5C_TESTPA2, RH_RF69_TESTPA2_NORMAL);
    uint8_t opmode = transmit.radio.spiRead(RH_RF69_REG_01_OPMODE);
    opmode &= ~RH_RF69_OPMODE_MODE;
    opmode |= (RH_RF69_OPMODE_MODE_STDBY & RH_RF69_OPMODE_MODE);
    transmit.radio.spiWrite(RH_RF69_REG_01_OPMODE, opmode);
}

void radioTx()
{
    modeReady = false;
    transmit.radio.spiWrite(RH_RF69_REG_5A_TESTPA1, RH_RF69_TESTPA1_BOOST);
    transmit.radio.spiWrite(RH_RF69_REG_5C_TESTPA2, RH_RF69_TESTPA2_BOOST);
    transmit.radio.spiWrite(RH_RF69_REG_25_DIOMAPPING1, RH_RF69_DIOMAPPING1_DIO0MAPPING_00); // Set interrupt line 0 PacketSent
    uint8_t opmode = transmit.radio.spiRead(RH_RF69_REG_01_OPMODE);
    opmode &= ~RH_RF69_OPMODE_MODE;
    opmode |= (RH_RF69_OPMODE_MODE_TX & RH_RF69_OPMODE_MODE);
    transmit.radio.spiWrite(RH_RF69_REG_01_OPMODE, opmode);
}