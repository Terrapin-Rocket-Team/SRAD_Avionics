/*
MIT License

Copyright (c) 2020 Peter Buchegger

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef APRSMSG_H
#define APRSMSG_H

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

class APRSMessageType
{
public:
    enum Value : uint8_t
    {
        PositionWithoutTimestamp, // = and !
        PositionWithTimestamp,    // @ and /
        Status,                   // >
        Query,                    // ?
        Message,                  // :
        Weather,                  // _
        Telemetry,                // T
        CurrentMicEData,          // `
        // you can add more types ;)
        Error,
    };

    APRSMessageType() = default;
    // cppcheck-suppress noExplicitConstructor
    APRSMessageType(char type)
    {
        switch (type)
        {
        case '=':
        case '!':
            value = PositionWithoutTimestamp;
            break;
        case '@':
        case '/':
            value = PositionWithTimestamp;
            break;
        case '>':
            value = Status;
            break;
        case '?':
            value = Query;
            break;
        case ':':
            value = Message;
            break;
        case '_':
            value = Weather;
            break;
        case 'T':
            value = Telemetry;
            break;
        case '`':
            value = CurrentMicEData;
            break;
        default:
            value = Error;
        }
    }
    // cppcheck-suppress noExplicitConstructor
    constexpr APRSMessageType(Value aType) : value(aType) {}
    constexpr bool operator==(APRSMessageType a) const { return value == a.value; }
    constexpr bool operator!=(APRSMessageType a) const { return value != a.value; }
    explicit operator bool() const { return value != Error; }

    const char *toString() const
    {
        switch (value)
        {
        case PositionWithoutTimestamp:
            return "Position Without Timestamp";
        case PositionWithTimestamp:
            return "Position With Timestamp";
        case Status:
            return "Status";
        case Query:
            return "Query";
        case Message:
            return "Message";
        case Weather:
            return "Weather";
        case Telemetry:
            return "Telemetry";
        case CurrentMicEData:
            return "Current Mic-E Data";
        default:
            return "Error";
        }
    }

private:
    Value value = Error;
};

class APRSBody
{
public:
    APRSBody();
    virtual ~APRSBody();

    const char *getData();
    void setData(const char data[80]);

    virtual bool decode(char *message);
    virtual const char *encode();
    virtual void toString(char *str);

private:
    char _data[80]{0};
};

class APRSMsg
{
public:
    APRSMsg();
    APRSMsg(APRSMsg &otherMsg);
    APRSMsg &operator=(APRSMsg &otherMsg);
    virtual ~APRSMsg();

    const char *getSource();
    void setSource(const char source[8]);

    const char *getDestination();
    void setDestination(const char destination[8]);

    const char *getPath();
    void setPath(const char path[10]);

    APRSMessageType getType();

    const char *getRawBody();
    APRSBody *getBody();

    virtual bool decode(char *message);
    virtual void encode(char *message);
    virtual void toString(char *str);

    static void formatLat(char *lat, bool hp);
    static void formatLong(char *lng, bool hp);
    static void formatDao(char *lat, char *lng, char *dao);
    static void padding(unsigned int number, unsigned int width, char *output, int offset = 0);

private:
    char _source[8]{0};
    char _destination[8]{0};
    char _path[10]{0};
    APRSMessageType _type;
    char _rawBody[80]{0};
    APRSBody _body;
};

/*
APRS Configuration
- CALLSIGN
- TOCALL
- PATH
- SYMBOL
- OVERLAY
*/
struct APRSConfig
{
    char CALLSIGN[8];
    char TOCALL[8];
    char PATH[10];
    char SYMBOL;
    char OVERLAY;
};

/*
APRS Telemetry Data
- lat
- lng
- alt
- spd
- hdg
- precision
- stage
- t0
- dao
*/
struct APRSData
{
    char lat[16];
    char lng[16];
    char alt[10];
    char spd[4];
    char hdg[4];
    char precision;
    char stage[3];
    char t0[9];
    char dao[6];
};

#endif // RADIO_H