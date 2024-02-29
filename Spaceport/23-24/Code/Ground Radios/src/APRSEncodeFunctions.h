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

#ifndef APRS_ENCODE_FUNCTIONS_H
#define APRS_ENCODE_FUNCTIONS_H

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

char *s_min_nn(uint32_t min_nnnnn, int high_precision);
void create_lat_aprs(char *lat, bool hp);
void create_long_aprs(char *lng, bool hp);
void create_dao_aprs(char *lat, char *lng, char *dao);
void padding(unsigned int number, unsigned int width, char *output, int offset = 0);
int numDigits(unsigned int num);

#endif