#ifndef RFM69HCW_H
#define RFM69HCW_H

#if defined(ARDUINO)
#define MSG_LEN 200
#elif defined(TEENSYDUINO)
#define MSG_LEN 10 * 1024
#elif defined(RASPBERRY_PI) || defined(__unix__)
#define MSG_LEN 10 * 1024
#endif

#include "Radio.h"
#include "APRSMsg.h"
#include "RH_RF69.h"

/*
Settings:
- double frequency in Mhz
- bool transmitter
- bool highBitrate
- RHGenericSPI *spi pointer to radiohead SPI object
- uint8_t cs CS pin
- uint8_t irq IRQ pin
- uint8_t rst RST pin
- uint8_t ff FifoFull pin
- uint8_t fne FifoNotEmpty pin
- uint8_t fl FifoLevel pin
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
    uint8_t ff;
    uint8_t fne;
    uint8_t fl;
};

class RFM69HCW : public Radio
{
public:
    RFM69HCW(const RadioSettings *s, const APRSConfig *config);
    bool begin() override;
    bool tx(const char *message, int len = -1) override;
    bool sendBuffer();
    void endtx();
    bool txs(const char *message, int len = -1); // tx start
    bool txI();                                  // tx Interrupt
    void txe();                                  // tx end
    const char *rx() override;
    void rxI(); // rx interrupt
    void rxs(); // rx end
    bool idle();
    bool encode(char *message, EncodingType type, int len = -1) override;
    bool decode(char *message, EncodingType type, int len = -1) override;
    bool send(const char *message, EncodingType type, int len = -1) override;
    const char *receive(EncodingType type) override;
    int RSSI() override;
    bool available();
    void set300KBPS();

    // stores full messages, max length determined by platform
    char msg[MSG_LEN + 1];
    // length of msg for recieving binary messages
    int msgLen = 0;

private:
    static void i0();
    static void i1();
    static void i2();
    static void i3();
    static void itr0();
    static void itr1();
    static void itr2();
    static void itr3();
    bool FifoFull();
    bool FifoNotEmpty();
    bool FifoLevel();

    RH_RF69 radio;
    // all radios should have the same networkID
    const uint8_t networkID = 0x01;
    const uint8_t sw[4] = {0xff, 0x00, 0x2d, 0xd4};
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
    bool busy;
    int rssi;
    int totalPackets;
    int msgIndex = 0;
    RHGenericDriver::RHMode mode = RHGenericDriver::RHModeIdle;

    static RFM69HCW *devices[];
    static int numInts;
};

#endif // RFM69HCW_H