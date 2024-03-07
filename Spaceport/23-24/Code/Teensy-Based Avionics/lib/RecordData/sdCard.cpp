#include "sdCard.h"

static constexpr int NAME_SIZE = 24;
// SdFs sd;
// FsFile logFile;  // File object to use for logging
char logFileName[NAME_SIZE];        // Name of the log file
char flightDataFileName[NAME_SIZE]; // Name of the flight data file
bool sdReady = false;               // Whether the SD card has been initialized

// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 3
/*
  Change the value of SD_CS_PIN if you are using SPI and
  your hardware does not use the default value, SS.
  Common values are:
  Arduino Ethernet shield: pin 4
  Sparkfun SD shield: pin 8
  Adafruit SD shields and modules: pin 10
*/

// SDCARD_SS_PIN is defined for the built-in SD on some boards.
#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SS;
#else  // SDCARD_SS_PIN
// Assume built-in SD is used.
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif // SDCARD_SS_PIN

// Try max SPI clock for an SD. Reduce SPI_CLOCK if errors occur.
#define SPI_CLOCK SD_SCK_MHZ(50)

// Try to select the best SD card configuration.
#if HAS_SDIO_CLASS
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#elif ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
#else // HAS_SDIO_CLASS
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK)
#endif // HAS_SDIO_CLASS

#if SD_FAT_TYPE == 0
SdFat sd;
File logFile;
File flightDataFile;
#elif SD_FAT_TYPE == 1
SdFat32 sd;
File32 logFile;
File32 flightDataFile;
#elif SD_FAT_TYPE == 2
SdExFat sd;
ExFile logFile;
ExFile flightDataFile;
#elif SD_FAT_TYPE == 3
SdFs sd;
FsFile logFile;
FsFile flightDataFile;
#else // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif // SD_FAT_TYPE

// Initializes the sensor
bool setupSDCard()
{
    int rdy = 0;
    sdReady = false;
    if (sd.begin(SD_CONFIG))
    {
        // Find file name
        int fileNo = 0;
        bool exists = true;
// Disable the warning for the following line, because seeing the warning all the time is annoying
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-truncation"
        while (exists)
        {
            snprintf(logFileName, NAME_SIZE, "%d_Log.txt", ++fileNo);
            exists = sd.exists(logFileName);
        }
        snprintf(flightDataFileName, NAME_SIZE, "%d_FlightData.csv", fileNo); // will overwrite the previous file if it exists
#pragma GCC diagnostic pop

        // Setup files
        logFile = sd.open(logFileName, FILE_WRITE);
        if (logFile)
        {
            logFile.close();
            rdy++;
        }
        flightDataFile = sd.open(flightDataFileName, FILE_WRITE);
        if (flightDataFile)
        {
            flightDataFile.close();
            rdy++;
        }
    }
    sdReady = rdy == 2;

    return sdReady;
}
void sendSDCardHeader(const char *csvHeader)
{
    if (sdReady)
    {
        flightDataFile = sd.open(flightDataFileName, FILE_WRITE);
        if (flightDataFile)
        {
            flightDataFile.println(csvHeader);
            flightDataFile.close();
        }
    }
}
// Returns whether the sensor is initialized
bool isSDReady()
{
    return sdReady;
}