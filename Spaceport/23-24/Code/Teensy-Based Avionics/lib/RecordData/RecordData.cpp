#include "RecordData.h"

static const char *logTypeStrings[] = {"LOG", "ERROR", "WARNING", "INFO"};
static Mode mode = GROUND;
void recordFlightData(char *data)
{
    if(!isSDReady())
        return;
    if (mode == GROUND)
    {
        flightDataFile = sd.open(flightDataFileName, FILE_WRITE); // during preflight, print to SD card constantly. ignore PSRAM for this stage.
        if (flightDataFile)
        {
            flightDataFile.println(data);
            flightDataFile.close();
        }
    }
    else if(ram->isReady())
        ram->println(data); // while in flight, print to PSRAM for later dumping to SD card.
}

void recordLogData(LogType type, const char *data, Dest dest)
{
    recordLogData(millis() / 1000.0, type, data, dest);
}

void recordLogData(double timeStamp, LogType type, const char *data, Dest dest)
{
    int size = 15 + 7; // 15 for the timestamp and extra chars, 7 for the log type
    char logPrefix[size];
    snprintf(logPrefix, size, "%.3f - [%s] ", timeStamp, logTypeStrings[type]);

    if (dest == BOTH || dest == TO_USB)
    {
        Serial.print(logPrefix);
        Serial.println(data);
    }
    if ((dest == BOTH || dest == TO_FILE) && isSDReady())
    {
        if (mode == GROUND)
        {
            logFile = sd.open(logFileName, FILE_WRITE); // during preflight, print to SD card constantly. ignore PSRAM for this stage. With both files printing, may be bad...
            if (logFile)
            {
                logFile.print(logPrefix);
                logFile.println(data);
                logFile.close();
            }
        }
        else if (ram->isReady()) // while in flight, print to PSRAM for later dumping to SD card.
        {
            ram->print(logPrefix, false);
            ram->println(data, false);
        }
    }
}

void setRecordMode(Mode m)
{
    if(mode == FLIGHT && m == GROUND){
        ram->dumpFlightData();
        ram->dumpLogData();
        digitalWrite(LED_BUILTIN, HIGH);
        digitalWrite(33, HIGH);
        delay(2000);
        digitalWrite(LED_BUILTIN, LOW);
        digitalWrite(33, LOW);
    }
    mode = m;
}