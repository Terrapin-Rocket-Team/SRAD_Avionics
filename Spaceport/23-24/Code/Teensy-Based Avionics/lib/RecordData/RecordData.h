#ifndef RECORD_DATA_H
#define RECORD_DATA_H

#include "psram.h"
#include "sdCard.h"

extern PSRAM *ram;


enum LogType
{
    LOG_,
    ERROR_,
    WARNING_,
    INFO_
};
enum Dest
{
    BOTH,
    TO_USB,
    TO_FILE
};
enum Mode
{
    FLIGHT,
    GROUND
};
void recordFlightData(char *data); //0 is preflight, 5 is postflight.
void recordLogData(LogType type, const char *data, Dest dest = BOTH);
void recordLogData(double timeStamp, LogType type, const char *data, Dest dest = BOTH);
void setRecordMode(Mode mode);//Will enable or disable the PSRAM based on the mode. If mode is GROUND, PSRAM will be disabled and all data in PSRAM will be written to the SD card. If mode is FLIGHT, PSRAM will be enabled.
#endif