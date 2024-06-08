#include <Arduino.h>

#include "RFM69HCW.h"

#define MSG_SIZE 37500
#define TX_ADDR 0x02
#define RX_ADDR 0x01
#define SYNC1 0x37
#define SYNC2 0x69

// Serial communication with Pi
char *buf;
unsigned int loc = 0;

// radio variables
bool hasTransmission = false;
char msg[MSG_SIZE] = {0};
unsigned int pos = 0;

// testing
// uint32_t timer = millis();
// end testing

// radio
APRSConfig config = {"KC3UTM", "APRS", "WIDE1-1", '[', '/'};
RadioSettings settings = {433.775, true, true, &hardware_spi, 10, 15, 14, 8, 4, 9};
RFM69HCW transmit = {&settings, &config};

void setup()
{
  Serial.begin(250000);

  buf = new char[MSG_SIZE * 2]; // 60 frames
  Serial1.begin(460800);        // 460800 baud

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
  if (hasTransmission && transmit.radio.mode() == RHGenericDriver::RHModeTx && pos < MSG_SIZE)
  {

    // Start spi transaction
    settings.spi->beginTransaction();
    // Select radio
    digitalWrite(settings.cs, LOW);

    settings.spi->transfer(RH_RF69_REG_00_FIFO | RH_RF69_SPI_WRITE_MASK);

    // Send the next section of the payload
    while (pos < MSG_SIZE && !transmit.FifoFull())
    {
      settings.spi->transfer(msg[pos]);
      Serial.write(msg[pos]);
      pos++;
    }

    // Deselect radio
    digitalWrite(settings.cs, HIGH);
    // End spi transaction
    settings.spi->endTransaction();
  }

  // Start a new transmission
  if ((loc >= MSG_SIZE /*|| millis() - timer > 35*/) && !hasTransmission && transmit.radio.mode() != RHGenericDriver::RHModeTx)
  {
    // testing
    // timer = millis();
    // for (int i = 0; i < 1250 / 50; i++)
    // memcpy(buf + i * 50, "Test test test test test test test test test test ", 50);
    // loc = MSG_SIZE;
    // end testing
    // Setup
    // Serial.println(loc);
    // Serial.write(buf[0]);
    // Serial.write(buf[MSG_SIZE - 1]);
    // Serial.println("here");
    memcpy(msg, buf, MSG_SIZE);
    // Serial.write(msg[0]);
    // Serial.write(msg[MSG_SIZE - 1]);
    // Serial.flush();
    hasTransmission = true;

    // Start spi transaction
    settings.spi->beginTransaction();
    // Select radio
    digitalWrite(settings.cs, LOW);

    // Select the fifo for writing
    settings.spi->transfer(RH_RF69_REG_00_FIFO | RH_RF69_SPI_WRITE_MASK);
    // Send message length
    settings.spi->transfer(SYNC1);
    // Send address for receving radio
    settings.spi->transfer(SYNC2);

    // Send the payload
    while (pos < MSG_SIZE && !transmit.FifoFull())
    {
      settings.spi->transfer(msg[pos]);
      Serial.write(msg[pos]);
      pos++;
    }

    // Deselect radio
    digitalWrite(settings.cs, HIGH);
    // End spi transaction
    settings.spi->endTransaction();

    // Start the transmitter
    if (pos != 0)
    {
      transmit.radio.setModeTx();
    }
    else
    {
      Serial.println("No bytes transfered");
    }

    // clean
    // Serial.println(loc);
    if (loc - MSG_SIZE > 0)
      memcpy(buf, buf + MSG_SIZE, loc - MSG_SIZE);
    loc = loc - MSG_SIZE;
    // Serial.println(loc);
  }

  if (pos == MSG_SIZE && digitalRead(settings.irq))
  {
    pos = 0;
    hasTransmission = false;
    transmit.radio.setModeIdle();
    // Serial.println("\nfinished");
  }
}