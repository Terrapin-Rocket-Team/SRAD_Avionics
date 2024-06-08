#
# Makefile:
#       wiringPi - A "wiring" library for the Raspberry Pi
#       https://github.com/wiringPi/wiringPi
#
#       Copyright (c) 2012-2015 Gordon Henderson
#################################################################################
# This file is part of wiringPi:
#       Wiring Compatable library for the Raspberry Pi
#
#    wiringPi is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    wiringPi is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
#################################################################################

ifneq ($V,1)
Q ?= @
endif

SRC     =       main.cpp

OBJ     =       $(SRC:.cpp=.o)

BINS    =       $(SRC:.cpp=)

DEBUG   = -O3
CC      ?= gcc
INCLUDE = -I/usr/local/include
CFLAGS  = $(DEBUG) -Wall $(INCLUDE) -Winline -pipe $(EXTRA_CFLAGS)
BUILDDIR = build

LDFLAGS = -L/usr/local/lib
LDLIBS    = -lwiringPi -lwiringPiDev -lpthread -lm -lcrypt -lrt

all: premake    main

really-all:     $(BINS)

premake:
        mkdir -p $(BUILDDIR)

main:   $(BUILDDIR)/main.o
        $Q echo [link]
        $Q $(CC) $^ -o $(BUILDDIR)/$@ $(LDFLAGS) $(LDLIBS)

$(BUILDDIR)/%.o: %.cpp
        $Q echo [CC] $<
        $Q $(CC) -c $(CFLAGS) $< -o $@

clean:
        $Q echo "[Clean]"
        $Q rm -f $(OBJ) *~ core tags $(BINS)

tags:   $(SRC)
        $Q echo [ctags]
        $Q ctags $(SRC)

depend:
        makedepend -Y $(SRC)