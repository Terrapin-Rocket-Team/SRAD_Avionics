#ifndef RADIO_H
#define RADIO_H

#if defined(ARDUINO)
#include <Arduino.h>
#elif defined(_WIN32) || defined(_WIN64) // Windows
#include <cstdint>
#include <string>
#include <cstring>
#elif defined(__unix__)  // Linux
// TODO
#elif defined(__APPLE__) // OSX
// TODO
#endif

/*
Types:
- ENCT_TELEMETRY: latitude,longitude,altitude,speed,heading,precision,stage,t0 <-> APRS message
- ENCT_VIDEO: char* filled with raw bytes <-> Raw byte array
- ENCT_GROUNDSTATION: Source:Value,Destination:Value,Path:Value,Type:Value,Body:Value <-> APRS message
- ENCT_NONE: no encoding is applied, same as using tx()
*/
enum EncodingType
{
    ENCT_TELEMETRY,
    ENCT_VIDEO,
    ENCT_GROUNDSTATION,
    ENCT_NONE
};

class Radio
{
public:
    virtual ~Radio(){}; // Virtual descructor. Very important
    virtual bool begin() = 0;
    virtual bool tx(const char *message, int len = -1) = 0;
    virtual const char *rx() = 0;
    virtual bool encode(char *message, EncodingType type, int len = -1) = 0;
    virtual bool decode(char *message, EncodingType type, int len = -1) = 0;
    virtual bool send(const char *message, EncodingType type, int len = -1) = 0;
    virtual const char *receive(EncodingType type) = 0;
    virtual int RSSI() = 0;
};

#endif // RADIO_H