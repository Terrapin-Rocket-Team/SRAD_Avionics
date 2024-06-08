/*
 * serialTest.c:
 *	Very simple program to test the serial port. Expects
 *	the port to be looped back to itself
 *
 * Copyright (c) 2012-2013 Gordon Henderson.
 ***********************************************************************
 * This file is part of wiringPi:
 *      https://github.com/WiringPi/WiringPi
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <termios.h>
#include <cstdint>

#include <wiringPi.h>
#include <wiringSerial.h>

volatile sig_atomic_t force_exit = false;

void sig_handler(int sig)
{
    printf("\nexiting...\n");
    force_exit = true;
}

int main()
{
    int fd;
    FILE *logFile = fopen("teensy-interface.log", "w");
    char data[1250] = {0};
    int count = 0;

    signal(SIGINT, sig_handler);

    if ((fd = serialOpen("/dev/serial0", 460800)) < 0)
    {
        fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
        return 1;
    }

    if (wiringPiSetup() == -1)
    {
        fprintf(stdout, "Unable to start wiringPi: %s\n", strerror(errno));
        return 1;
    }

    printf("Starting transmission\n");

    int packetCount = 0;
    long bytes = 0;
    uint64_t timer = micros();
    char logMsg[50];
    long long us = 0;
    while (!force_exit)
    {
        if (micros() - timer > 2100)
        {
            us += micros() - timer;
            // the way logMsg is formatted is a little weird, but should generate a nice looking log file
            if (us >= 1000000)
            {
                printf("\nAvg Bitrate: %lf\n", us, (bytes + 75) * 8, ((double(bytes + 75) * 8) / us) * 1000);
                snprintf(logMsg, 50, "\nElapsed Time: %llu Bitrate: %lf \nElapsed Time: ", micros() - timer, ((double(bytes + 75) * 8) / us) * 1000);
                us = 0;
                bytes = 0;
            }
            else
            {
                snprintf(logMsg, 50, "%llu\t", micros() - timer);
            }
            fwrite(logMsg, sizeof(char), strlen(logMsg), logFile);
            timer = micros();
            count += fread(data + count, sizeof(char), 75 - count, stdin);
            if (count == 75)
            {
                printf("\rTransmitting");
                printf(" %i", packetCount++);
                // write(1, data, count);
                fflush(stdout);
                // Serial goes here
                write(fd, data, count);
                count = 0;
                bytes += 75;
            }
            // this controls max bitrate
            // delayMicroseconds(2100);
            // bitrate = (75*8)/delay
            // delay = (75*8)/bitrate
        }
    }
    fclose(logFile);
    serialClose(fd);
    return 0;
}
