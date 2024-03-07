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

#include "APRSEncodeFunctions.h"

// takes in decimal minutes and converts to MM.dddd
char *sMin_nn(uint32_t min_nnnnn, int highPrecision)
{
    /* min_nnnnn: RawDegrees billionths is uint32_t by definition and is n'telth
     * degree (-> *= 6 -> nn.mmmmmm minutes) highPrecision: 0: round at decimal
     * position 2. 1: round at decimal position 4. 2: return decimal position 3-4
     * as base91 encoded char
     */

    static char buf[6];

    min_nnnnn = min_nnnnn * 0.006;

    if (highPrecision)
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

    if (highPrecision < 2)
        sprintf(buf, "%02u.%02u", (unsigned int)((min_nnnnn / 100000) % 100), (unsigned int)((min_nnnnn / 1000) % 100));
    else
        sprintf(buf, "%c", (char)((min_nnnnn % 1000) / 11) + 33);
    // Like to verify? type in python for i.e. RawDegrees billions 566688333: i =
    // 566688333; "%c" % (int(((i*.0006+0.5) % 100)/1.1) +33)
    return buf;
}

// creates the latitude string for the APRS message based on whether the GPS coordinates are high precision
void createLatAprs(char *lat, bool hp)
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
    // we like sprintf's float up-rounding.
    // but sprintf % may round to 60.00 -> 5360.00 (53° 60min is a wrong notation
    // ;)

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
    sprintf(lat, "%02d%s%c", atoi(lat) * (negative ? -1 : 1), sMin_nn(dec, hp), negative ? 'S' : 'N');
}

// creates the longitude string for the APRS message based on whether the GPS coordinates are high precision
void createLongAprs(char *lng, bool hp)
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
    // we like sprintf's float up-rounding.
    // but sprintf % may round to 60.00 -> 5360.00 (53° 60min is a wrong notation
    // ;)

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
    sprintf(lng, "%02d%s%c", atoi(lng) * (negative ? -1 : 1), sMin_nn(dec, hp), negative ? 'W' : 'E');
}

// creates the dao at the end of aprs message based on latitude and longitude
void createDaoAprs(char *lat, char *lng, char *dao)
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
    sprintf(dao, "!w%s", sMin_nn(atoi(lat + decimalPos + 1), 2));

    len = strlen(lng);
    decimalPos = 0;
    for (int i = 0; i < len; i++)
    {
        if (lng[i] == '.')
            decimalPos = i;
    }
    sprintf(dao + 3, "%s!", sMin_nn(atoi(lng + decimalPos + 1), 2));
}

// adds a specified number of zeros to the begining of a number
void padding(unsigned int number, unsigned int width, char *output, int offset)
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

int numDigits(unsigned int num)
{
    if (num < 10)
        return 1;
    return 1 + numDigits(num / 10);
}