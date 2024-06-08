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

#include "APRSMsg.h"

char *s_min_nn(uint32_t min_nnnnn, int high_precision);
int numDigits(unsigned int num);

APRSMsg::APRSMsg() : _body()
{
}

APRSMsg::APRSMsg(APRSMsg &otherMsg)
    : _type(otherMsg.getType()), _body()
{
    strcpy(_source, otherMsg.getSource());
    strcpy(_destination, otherMsg.getDestination());
    strcpy(_path, otherMsg.getPath());
    strcpy(_rawBody, otherMsg.getRawBody());
    _body.setData(otherMsg.getBody()->getData());
}

APRSMsg &APRSMsg::operator=(APRSMsg &otherMsg)
{
    if (this != &otherMsg)
    {
        setSource(otherMsg.getSource());
        setDestination(otherMsg.getDestination());
        setPath(otherMsg.getPath());
        _type = otherMsg.getType();
        strcpy(_rawBody, otherMsg.getRawBody());
        _body.setData(otherMsg.getBody()->getData());
    }
    return *this;
}

APRSMsg::~APRSMsg()
{
}

const char *APRSMsg::getSource()
{
    return _source;
}

void APRSMsg::setSource(const char source[8])
{
    strcpy(_source, source);
}

const char *APRSMsg::getDestination()
{
    return _destination;
}

void APRSMsg::setDestination(const char destination[8])
{
    strcpy(_destination, destination);
}

const char *APRSMsg::getPath()
{
    return _path;
}

void APRSMsg::setPath(const char path[10])
{
    strcpy(_path, path);
}

APRSMessageType APRSMsg::getType()
{
    return _type;
}

const char *APRSMsg::getRawBody()
{
    return _rawBody;
}

APRSBody *APRSMsg::getBody()
{
    return &_body;
}

bool APRSMsg::decode(char *message)
{
    int len = strlen(message);
    int posSrc = -1, posDest = -1, posPath = -1;
    for (int i = 0; i < len; i++)
    {
        if (message[i] == '>' && posSrc == -1)
            posSrc = i;
        if (message[i] == ',' && posDest == -1)
            posDest = i;
        if (message[i] == ':' && posPath == -1)
            posPath = i;
    }
    if (posSrc >= 8)
        return false;
    if (posDest - (posSrc + 1) >= 8)
        return false;
    if (posPath - (posDest + 1) >= 10)
        return false;

    if (posSrc >= 0)
    {
        strncpy(_source, message, posSrc);
        _source[posSrc] = '\0';
    }
    else
    {
        _source[0] = '\0';
    }

    if (posDest != -1 && posDest < posPath)
    {
        strncpy(_path, message + posDest + 1, posPath - (posDest + 1));
        strncpy(_destination, message + posSrc + 1, posDest - (posSrc + 1));
        _path[posPath - (posDest + 1)] = '\0';
        _destination[posDest - (posSrc + 1)] = '\0';
    }
    else
    {
        _path[0] = '\0';
        if (posSrc >= 0 && posPath >= 0)
        {

            strncpy(_destination, message + posSrc + 1, posPath - (posSrc + 1));
            _destination[posPath - (posSrc + 1)] = '\0';
        }
        else
        {
            _destination[0] = '\0';
        }
    }
    strcpy(_rawBody, message + posPath + 1);
    _rawBody[strlen(message + posPath + 1)] = '\0';
    _type = APRSMessageType(_rawBody[0]);
    _body.decode(_rawBody);
    return bool(_type);
}

void APRSMsg::encode(char *message)
{
    sprintf(message, "%s>%s", _source, _destination);
    if (strlen(_path) > 0)
    {
        sprintf(message + strlen(message), ",%s", _path);
    }
    sprintf(message + strlen(message), ":%s", _body.encode());
}

void APRSMsg::toString(char *str)
{
    char body[87];
    _body.toString(body);
    sprintf(str, "Source:%s,Destination:%s,Path:%s,Type:%s,%s", _source, _destination, _path, _type.toString(), body);
}

APRSBody::APRSBody()
{
}

APRSBody::~APRSBody()
{
}

const char *APRSBody::getData()
{
    return _data;
}

void APRSBody::setData(const char data[80])
{
    strcpy(_data, data);
}

bool APRSBody::decode(char *message)
{
    strcpy(_data, message);
    return true;
}

const char *APRSBody::encode()
{
    return _data;
}

void APRSBody::toString(char *str)
{
    sprintf(str, "Data:%s", _data);
}

// creates the latitude string for the APRS message based on whether the GPS coordinates are high precision
void APRSMsg::formatLat(char *lat, bool hp)
{
    bool negative = lat[0] == '-';

    int len = strlen(lat);
    int decimalPos = 0;
    // use for loop in case there is no decimal
    for (int i = 0; i < len; i++)
    {
        if (lat[i] == '.')
        {
            decimalPos = i;
            break;
        }
    }
    int decLen = strlen(lat + decimalPos + 1);
    int dec = atoi(lat + decimalPos + 1);

    // make sure there are nine digits
    int missingDigits = 9 - decLen;
    if (missingDigits < 0)
        for (int i = 0; i < missingDigits * -1; i++)
            dec /= 10;
    if (missingDigits > 0)
        for (int i = 0; i < missingDigits; i++)
            dec *= 10;

    // we like sprintf's float up-rounding.
    // but sprintf % may round to 60.00 -> 5360.00 (53° 60min is a wrong notation
    // ;)
    sprintf(lat, "%02d%s%c", atoi(lat) * (negative ? -1 : 1), s_min_nn(dec, hp), negative ? 'S' : 'N');
}

// creates the longitude string for the APRS message based on whether the GPS coordinates are high precision
void APRSMsg::formatLong(char *lng, bool hp)
{
    bool negative = lng[0] == '-';

    int len = strlen(lng);
    int decimalPos = 0;
    // use for loop in case there is no decimal
    for (int i = 0; i < len; i++)
    {
        if (lng[i] == '.')
        {
            decimalPos = i;
            break;
        }
    }

    int decLen = strlen(lng + decimalPos + 1);
    int dec = atoi(lng + decimalPos + 1);

    // make sure there are nine digits
    int missingDigits = 9 - decLen;
    if (missingDigits < 0)
        for (int i = 0; i < missingDigits * -1; i++)
            dec /= 10;
    if (missingDigits > 0)
        for (int i = 0; i < missingDigits; i++)
            dec *= 10;

    // we like sprintf's float up-rounding.
    // but sprintf % may round to 60.00 -> 5360.00 (53° 60min is a wrong notation
    // ;)
    sprintf(lng, "%02d%s%c", atoi(lng) * (negative ? -1 : 1), s_min_nn(dec, hp), negative ? 'W' : 'E');
}

// creates the dao at the end of aprs message based on latitude and longitude
void APRSMsg::formatDao(char *lat, char *lng, char *dao)
{
    // !DAO! extension, use Base91 format for best precision
    // /1.1 : scale from 0-99 to 0-90 for base91, int(... + 0.5): round to nearest
    // integer https://metacpan.org/dist/Ham-APRS-FAP/source/FAP.pm
    // http://www.aprs.org/aprs12/datum.txt

    int len = strlen(lat);
    int decimalPos = 0;
    // use for loop in case there is no decimal
    for (int i = 0; i < len; i++)
    {
        if (lat[i] == '.')
            decimalPos = i;
    }
    sprintf(dao, "!w%s", s_min_nn((int)(lat + decimalPos + 1), 2));

    len = strlen(lng);
    decimalPos = 0;
    for (int i = 0; i < len; i++)
    {
        if (lng[i] == '.')
            decimalPos = i;
    }
    sprintf(dao + 3, "%s!", s_min_nn((int)(lng + decimalPos + 1), 2));
}

// adds a specified number of zeros to the begining of a number
void APRSMsg::padding(unsigned int number, unsigned int width, char *output, int offset)
{
    unsigned int numLen = numDigits(number);
    if (numLen > width)
    {
        width = numLen;
    }
    for (unsigned int i = 0; i < width - numLen; i++)
    {
        output[offset + i] = '0';
    }
    sprintf(output + offset + width - numLen, "%u", number);
}

// takes in decimal minutes and converts to MM.dddd
char *s_min_nn(uint32_t min_nnnnn, int high_precision)
{
    /* min_nnnnn: RawDegrees billionths is uint32_t by definition and is n'telth
     * degree (-> *= 6 -> nn.mmmmmm minutes) high_precision: 0: round at decimal
     * position 2. 1: round at decimal position 4. 2: return decimal position 3-4
     * as base91 encoded char
     */

    static char buf[6];

    min_nnnnn = min_nnnnn * 0.006;

    if (high_precision)
    {
        if ((min_nnnnn % 10) >= 5 && min_nnnnn < 6000000 - 5)
        {
            // round up. Avoid overflow (59.999999 should never become 60.0 or more)
            min_nnnnn = min_nnnnn + 5;
        }
    }
    else
    {
        if ((min_nnnnn % 1000) >= 500 && min_nnnnn < (6000000 - 500))
        {
            // round up. Avoid overflow (59.9999 should never become 60.0 or more)
            min_nnnnn = min_nnnnn + 500;
        }
    }

    if (high_precision < 2)
        sprintf(buf, "%02u.%02u", (unsigned int)((min_nnnnn / 100000) % 100), (unsigned int)((min_nnnnn / 1000) % 100));
    else
        sprintf(buf, "%c", (char)((min_nnnnn % 1000) / 11) + 33);
    // Like to verify? type in python for i.e. RawDegrees billions 566688333: i =
    // 566688333; "%c" % (int(((i*.0006+0.5) % 100)/1.1) +33)
    return buf;
}

int numDigits(unsigned int num)
{
    if (num < 10)
        return 1;
    return 1 + numDigits(num / 10);
}