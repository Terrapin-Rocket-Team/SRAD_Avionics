#ifndef RFM69HCW_H
#define RFM69HCW_H

#if defined(ARDUINO)
#define MSG_LEN 200
#elif defined(TEENSYDUINO)
#define MSG_LEN 10 * 1024
#elif defined(RASPBERRY_PI)
#define MSG_LEN 10 * 1024
#endif

#include "Radio.h"
#include "APRSMsg.h"
#include "RH_RF69.h"
#include "APRSEncodeFunctions.h"

/*
Settings:
- double frequency in Mhz
- bool transmitter
- bool highBitrate
- RHGenericSPI *spi pointer to radiohead SPI object
- uint8_t cs CS pin
- uint8_t irq IRQ pin
- uint8_t rst RST pin
*/
struct RadioSettings
{
    double frequency;
    bool transmitter;
    bool highBitrate;
    RHGenericSPI *spi;
    uint8_t cs;
    uint8_t irq;
    uint8_t rst;
};

class RFM69HCW : public Radio
{
public:
    RFM69HCW(const RadioSettings s, const APRSConfig config);
    bool begin() override;
    bool tx(const char *message) override;
    const char *rx() override;
    bool encode(char *message, EncodingType type) override;
    bool decode(char *message, EncodingType type) override;
    bool send(const char *message, EncodingType type) override;
    const char *receive(EncodingType type) override;
    int RSSI() override;
    bool available();
    void set300KBPS();

private:
    RH_RF69 radio;
    // all radios should have the same networkID
    const uint8_t networkID = 0x01;
    // default to the highest transmit power
    const int txPower = 20;
    // set by constructor
    uint8_t addr;
    uint8_t toAddr;
    uint8_t id;
    RadioSettings settings;
    // for sending/receiving data
    // stores messages sent to radio, length determined by max radio message length
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN + 1];
    uint8_t bufSize = RH_RF69_MAX_MESSAGE_LEN;
    APRSConfig cfg;
    bool avail;
    int rssi;
    int incomingMsgLen;
    // stores full messages, max length determined by platform
    char msg[MSG_LEN + 1];
};

#endif // RFM69HCW_H