#ifndef PSRAM_H
#define PSRAM_H

#include "sdCard.h"
#include "BlinkBuzz.h"

extern "C" uint8_t external_psram_size;
class PSRAM
{
public:
    PSRAM();
    bool init();
    void print(const char *data, bool isFlightData = true);
    void println(const char *data, bool isFlightData = true);
    void write(const uint8_t data);
    bool isReady();
    bool dumpFlightData();
    bool dumpLogData();
    int getFreeSpace();

private:
    bool ready;
    bool dumped;
    char *cursorStart;//for flight data
    char *cursorEnd;//for log data
    char *memBegin;//start of memory
    char *memEnd;//end of memory

};

#endif