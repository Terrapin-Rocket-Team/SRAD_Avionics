#include "EncodeAPRSForSerial.h"
#include "Adafruit_BNO055.h"
#include <Arduino.h>
#include "../Radio/RFM69HCW.h"
#include "../Radio/APRS/APRSCmdMsg.h"
#include "LiveRFM69HCW.h"
#include <SD.h>
#include <SPI.h>

#define SERIAL_BAUD 1000000 // 1M Baud => 8us per byte

#define RADIO1_HEADER 0x01
#define RADIO2_HEADER 0x02
#define RADIO3_HEADER 0xfe

// in bytes
#define VID_PACKET_SIZE 400
#define PACKET3_SIZE 60
#define COMMAND_SIZE 4

// makes them 30 times as large
#define BUFF1_SIZE (VID_PACKET_SIZE * 250)
#define BUFF2_SIZE (VID_PACKET_SIZE * 250)
#define BUFF3_SIZE (PACKET3_SIZE * 30)

// 1, 2 are for live radio, 3 is for telemetry, 4 is for input
uint8_t buff1[BUFF1_SIZE];
uint8_t buff2[BUFF2_SIZE];
uint8_t buff3[BUFF3_SIZE];

// live video stuff

#define MSG_SIZE 5000
#define TX_ADDR 0x02
#define RX_ADDR 0x01
#define SYNC1 0x00
#define SYNC2 0xff

struct LiveSettings
{
    bool hasTransmission;
    bool modeReady;
    bool modeIdle = true;
    unsigned int pos;
    bool hasL;
    bool hasA;
};

struct RadioObject
{
    APRSTelemMsg *telem;
    APRSCmdMsg *cmd;
    RFM69HCW *radio;
    const int packetSize;
    uint8_t *buffer;
    int buffsize;
    int bufftop; // index of the next byte to be written
};

struct LiveRadioObject
{
    Live::RFM69HCW *radio;
    const int packetSize;
    byte *buffer;
    int buffsize;
    int bufftop; // index of the next byte to be written
    LiveSettings settings;
};

void radioIdle(LiveRadioObject *rad);
void radioRx(LiveRadioObject *rad);
void readLiveRadio(LiveRadioObject *rad);
void readLiveRadioX(LiveRadioObject *rad);

Live::APRSConfig config1 = {"KC3UTM", "APRS", "WIDE1-1", '[', '/'};
Live::RadioSettings settings1 = {433.775, false, true, &hardware_spi, 21, 22, 20, 26, 25, 24};
Live::RFM69HCW radio1(&settings1, &config1);

Live::APRSConfig config2 = {"KC3UTM", "APRS", "WIDE1-1", '[', '/'};
Live::RadioSettings settings2 = {915.0, false, true, &hardware_spi, 10, 15, 14, 7, 8, 9};
Live::RFM69HCW radio2(&settings2, &config2);

// Let radio3 be the telemetry receiver
APRSHeader header = {"KC3UTM", "APRS", "WIDE1-1", '/', 'o'};
APRSTelemMsg msg3(header);
APRSCmdMsg cmd3(header);
RadioSettings settings3 = {433.775, 0x02, 0x01, &hardware_spi, 10, 31, 32};
RFM69HCW radio3(&settings3);

LiveSettings lset1 = {false, false, true, 0, false, false};
LiveSettings lset2 = {false, false, true, 0, false, false};
LiveRadioObject r1 = {&radio1, VID_PACKET_SIZE, buff1, BUFF1_SIZE, 0, lset1};
LiveRadioObject r2 = {&radio2, VID_PACKET_SIZE, buff2, BUFF2_SIZE, 0, lset2};
RadioObject r3 = {&msg3, &cmd3, &radio3, PACKET3_SIZE, buff3, BUFF3_SIZE, 0};

// make an array of pointers to the radio objects
// void *radios[] = {&r1, &r2, &r1, &r2, &r1, &r2, &r1, &r2, &r1, &r2, &r1, &r2, &r1, &r2, &r1, &r2, &r1, &r2, &r1, &r2, &r1, &r2, &r1, &r2, &r1, &r2, &r1, &r2, &r1, &r2, &r3};

int counter = 0;
int radioIndex = 1;
uint16_t curPacketSize = 0;
uint16_t sentHeader = 0;

void setup()
{
    // Serial.begin(SERIAL_BAUD);
    delay(1000);

    Serial.println("Starting Ground Station Receiver");

    if (!radio1.begin())
        Serial.println("Radio1 failed to initialize");
    else
        Serial.println("Radio1 initialized");

    if (!radio2.begin())
        Serial.println("Radio2 failed to initialize");
    else
        Serial.println("Radio2 initialized");

    if (!radio3.init())
        Serial.println("Radio3 failed to initialize");
    else
        Serial.println("Radio3 initialized");

    // Serial.print("mux started");

    // Test Case for Demo Purposes
    // cmd3.data.MinutesUntilPowerOn = 0;
    // cmd3.data.MinutesUntilDataRecording = 1;
    // cmd3.data.MinutesUntilVideoStart = 2;
    // cmd3.data.Launch = false;

    // sdjkfhkjsd(&r3);
}

void loop()
{

    // delay(10);

    if (radio3.update())
    {
        if (radio3.dequeueReceive(&msg3))
        {
            char buffer[PACKET3_SIZE];
            aprsToSerial::encodeAPRSForSerial(msg3, buffer, 80, radio3.RSSI());
            // write to circular buffer for radio3 (assuming PACKET3_SIZE bytes of data)
            for (int i = 0; i < PACKET3_SIZE && r3.bufftop < r3.buffsize; i++)
            {
                buff3[r3.bufftop] = buffer[i];
                r3.bufftop = (r3.bufftop + 1);
                // delay(15);
            }
        }
    }

    // read live radio
    readLiveRadio(&r1);
    readLiveRadio(&r2);

    if (radioIndex <= 30)
    {

        // radio1
        if (radioIndex % 2 == 0)
        {
            // Serial.print("Bufftop: ");
            // Serial.println("SENDI");
            // check if start of packet
            if (sentHeader == 0)
            {
                // Serial.print("e");
                // Serial.println(r1.bufftop);

                if (r1.bufftop > VID_PACKET_SIZE)
                {
                    Serial.write(RADIO1_HEADER);
                    Serial.write(r1.bufftop >> 8);
                    Serial.write(r1.bufftop & 0xff);
                    sentHeader++;
                }
                else
                {
                    radioIndex++;
                }
            }

            if (sentHeader != 0)
            {
                // Serial.print("Dumping");
                // Serial.println(r1.bufftop);
                // if sent header for this packet, write block of data to serial
                for (int i = 0; i < r1.bufftop; i++)
                {
                    Serial.write(buff1[i]);
                    sentHeader++;
                }
                if (r1.bufftop > VID_PACKET_SIZE)
                    sentHeader = 0;

                // check if end of packet
                radioIndex++;
                curPacketSize = 0;
                r1.bufftop = 0;
            }
        }
        else // radio 2
        {
            // check if start of packet
            if (sentHeader == 0)
            {

                if (r2.bufftop > VID_PACKET_SIZE)
                {
                    Serial.write(RADIO2_HEADER);
                    Serial.write(r2.bufftop >> 8);
                    Serial.write(r2.bufftop & 0xff);
                    sentHeader++;
                }
                else
                {
                    radioIndex++;
                }
            }
            else
            {
                // write block of data to serial
                for (int i = 0; i < r2.bufftop; i++)
                {
                    Serial.write(buff2[i]);
                    sentHeader++;
                }

                // check if end of packet
                radioIndex++;
                if (r2.bufftop > VID_PACKET_SIZE)
                    sentHeader = 0;
                curPacketSize = 0;
                r2.bufftop = 0;
            }
        }
    }
    // radio3
    else
    {
        // Serial.println("Hit 3");
        // check if start of packet
        if (sentHeader == 0)
        {

            if (r3.bufftop > PACKET3_SIZE)
            {
                Serial.write(RADIO3_HEADER);
                Serial.write(r3.bufftop >> 8);
                Serial.write(r3.bufftop & 0xff);
                sentHeader++;
            }
            else
            {
                radioIndex = 1;;
            }
        }
        else
        {
            // write block of data to serial
            for (int i = 0; i < r3.bufftop; i++)
            {
                Serial.write(buff3[i]);
                sentHeader++;
            }

            // check if end of packet
            radioIndex = 1;
            if (r3.bufftop > PACKET3_SIZE)
                sentHeader = 0;
            curPacketSize = 0;
            r3.bufftop = 0;
        }
    }

    // Serial.println(r1.bufftop);
}

// read from serial
// if (Serial.available())
// {
//     // first byte is minutesUntilPowerOn, second byte is minutesUntilDataRecording, third byte is minutesUntilVideoStart, fourth byte is launch
//     if (Serial.available() >= COMMAND_SIZE)
//     {
//         byte command[COMMAND_SIZE];
//         for (int i = 0; i < COMMAND_SIZE; i++)
//         {
//             command[i] = Serial.read();
//         }

//         // set the command data
//         r3.cmd->data.MinutesUntilPowerOn = command[0];
//         r3.cmd->data.MinutesUntilDataRecording = command[1];
//         r3.cmd->data.MinutesUntilVideoStart = command[2];
//         r3.cmd->data.Launch = command[3];

//         // send the command
//         radio3.enqueueSend(r3.cmd);
//     }
//     else {
//         // reset the buffer
//         Serial.clear();
//     }
// }

// adapted from radioHead functions
void radioIdle(LiveRadioObject *rad)
{
    rad->settings.modeReady = false;
    rad->settings.modeIdle = true;

    rad->radio->radio.spiWrite(RH_RF69_REG_5A_TESTPA1, RH_RF69_TESTPA1_NORMAL);
    rad->radio->radio.spiWrite(RH_RF69_REG_5C_TESTPA2, RH_RF69_TESTPA2_NORMAL);
    uint8_t opmode = rad->radio->radio.spiRead(RH_RF69_REG_01_OPMODE);
    opmode &= ~RH_RF69_OPMODE_MODE;
    opmode |= (RH_RF69_OPMODE_MODE_STDBY & RH_RF69_OPMODE_MODE);
    rad->radio->radio.spiWrite(RH_RF69_REG_01_OPMODE, opmode);
}

void radioRx(LiveRadioObject *rad)
{
    rad->settings.modeReady = false;
    rad->settings.modeIdle = false;
    rad->radio->radio.spiWrite(RH_RF69_REG_5A_TESTPA1, RH_RF69_TESTPA1_NORMAL);
    rad->radio->radio.spiWrite(RH_RF69_REG_5C_TESTPA2, RH_RF69_TESTPA2_NORMAL);
    rad->radio->radio.spiWrite(RH_RF69_REG_25_DIOMAPPING1, RH_RF69_DIOMAPPING1_DIO0MAPPING_01); // Set interrupt line 0 PayloadReady
    uint8_t opmode = rad->radio->radio.spiRead(RH_RF69_REG_01_OPMODE);
    opmode &= ~RH_RF69_OPMODE_MODE;
    opmode |= (RH_RF69_OPMODE_MODE_RX & RH_RF69_OPMODE_MODE);
    rad->radio->radio.spiWrite(RH_RF69_REG_01_OPMODE, opmode);
}

void readLiveRadio(LiveRadioObject *rad)
{

    using namespace Live;
    // Refill fifo here
    if (rad->radio->FifoNotEmpty() && rad->settings.hasTransmission)
    {
        // Start spi transaction
        rad->radio->settings.spi->beginTransaction();

        // Select the radio
        digitalWrite(rad->radio->settings.cs, LOW);

        // Send the fifo address with the write mask off
        rad->radio->settings.spi->transfer(RH_RF69_REG_00_FIFO);

        // data
        while (rad->settings.pos < MSG_SIZE && rad->radio->FifoNotEmpty() && ((rad->bufftop) < (rad->buffsize)) && counter < VID_PACKET_SIZE)
        {
            rad->buffer[rad->bufftop] = rad->radio->settings.spi->transfer(0);
            rad->bufftop = (rad->bufftop + 1);

            // Serial.write(rad->radio->settings.spi->transfer(0));
            rad->settings.pos++;
            // Serial.println(rad->bufftop);
            counter++;
        }
        counter = 0;
        // Serial.println(rad->settings.pos);

        // rad->settings.firstTime = true;

        // reset msgLen and toAddr, end transaction, and clear fifo through entering idle mode
        digitalWrite(rad->radio->settings.cs, HIGH);
        rad->radio->settings.spi->endTransaction();

        if (rad->settings.pos == MSG_SIZE)
        {
            // Serial.println("finished");
            rad->settings.pos = 0;
            rad->settings.hasTransmission = false;
            radioIdle(rad);
        }
    }

    // Start a new transmission
    if (rad->radio->FifoNotEmpty() && !(rad->settings.hasTransmission) && !(rad->settings.modeIdle) && (rad->settings.modeReady))
    {
        // Serial.println("started receiving");
        // timer = millis();
        // Start spi transaction
        rad->radio->settings.spi->beginTransaction();
        // Select the radio
        digitalWrite(rad->radio->settings.cs, LOW);

        // Send the fifo address with the write mask off
        rad->radio->settings.spi->transfer(RH_RF69_REG_00_FIFO);
        // rad->settings.pos = 0;

        // while (rad->settings.pos < MSG_SIZE)
        // {
        //   if (rad->radio->FifoNotEmpty())
        //   {
        //     Serial.write(settings.spi->transfer(0));
        //     rad->settings.pos++;
        //   }
        // }

        if (!(rad->settings.hasL))
        {
            rad->settings.hasL = rad->radio->settings.spi->transfer(0) == SYNC1;
        }

        if (!(rad->settings.hasA) && rad->settings.hasL && rad->radio->FifoNotEmpty())
        {
            rad->settings.hasL = rad->settings.hasA = rad->radio->settings.spi->transfer(0) == SYNC2;
            if (!(rad->settings.hasA))
            {
                // radioIdle();
            }
        }

        // if found sync bytes
        if (rad->settings.hasL && rad->settings.hasA)
        {
            // set up for receiving the message
            rad->settings.hasTransmission = true;
            rad->settings.hasL = rad->settings.hasA = false;

            while (rad->settings.pos < MSG_SIZE && rad->radio->FifoNotEmpty() && rad->bufftop < rad->buffsize && counter < VID_PACKET_SIZE)
            {
                rad->buffer[rad->bufftop] = rad->radio->settings.spi->transfer(0);
                rad->bufftop = (rad->bufftop + 1);

                // Serial.write(rad->radio->settings.spi->transfer(0));
                rad->settings.pos++;
                // Serial.println(rad->bufftop);
                counter++;
            }
            // Serial.println(rad->settings.pos);
            // rad->settings.firstTime = true;
            counter = 0;
        }
        // reset msgLen and toAddr, end transaction, and clear fifo through entering idle mode
        digitalWrite(rad->radio->settings.cs, HIGH);
        rad->radio->settings.spi->endTransaction();

        if (rad->settings.pos == MSG_SIZE)
        {
            // Serial.println("finished 1");
            rad->settings.pos = 0;
            rad->settings.hasTransmission = false;
            radioIdle(rad);
        }
    }

    if ((rad->radio->radio.spiRead(RH_RF69_REG_27_IRQFLAGS1) & RH_RF69_IRQFLAGS1_MODEREADY) && !(rad->settings.modeReady))
    {
        rad->settings.modeReady = true;
        if (rad->settings.modeIdle)
            radioRx(rad);
    }
}

// filler funcs

void readLiveRadioX(LiveRadioObject *rad)
{
    // randomly pick a letter, and update the circular buffer with 50-100 of that letter
    char c = 'A' + (random(0, 26));

    int num = random(50, 100);
    for (int i = 0; i < num; i++)
    {
        rad->buffer[rad->bufftop] = c;
        rad->bufftop = (rad->bufftop + 1) % rad->buffsize;
    }
}