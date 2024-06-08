# Makefile
# Sample for RH_RF69 (client and server) on Raspberry Pi
# Caution: requires bcm2835 library to be already installed
# http://www.airspayce.com/mikem/bcm2835/

CC            = g++
CFLAGS        = -DRASPBERRY_PI -DBCM2835_NO_DELAY_COMPATIBILITY -D__BASEFILE__=\"$*\"
LIBS          = -lbcm2835
RADIOHEADBASE = ../RadioHead
BUILDDIR      = build
INCLUDE       = -I$(RADIOHEADBASE)

all: rf69v

$(BUILDDIR)/RasPi.o: $(RADIOHEADBASE)/RHutil/RasPi.cpp
				$(CC) $(CFLAGS) -c $(RADIOHEADBASE)/RHutil/RasPi.cpp $(INCLUDE) -o $@

$(BUILDDIR)/rf69v.o: rf69v.cpp
				$(CC) $(CFLAGS) -c $(INCLUDE) $< -o $@

$(BUILDDIR)/APRSMsg.o: APRSMsg.cpp
				$(CC) $(CFLAGS) -c $(INCLUDE) $< -o $@

$(BUILDDIR)/RFM69HCW.o: RFM69HCW.cpp
				$(CC) $(CFLAGS) -c $(INCLUDE) $< -o $@

$(BUILDDIR)/RH_RF69.o: $(RADIOHEADBASE)/RH_RF69.cpp
				$(CC) $(CFLAGS) -c $(INCLUDE) $< -o $@

$(BUILDDIR)/RHDatagram.o: $(RADIOHEADBASE)/RHDatagram.cpp
				$(CC) $(CFLAGS) -c $(INCLUDE) $< -o $@

$(BUILDDIR)/RHHardwareSPI.o: $(RADIOHEADBASE)/RHHardwareSPI.cpp
				$(CC) $(CFLAGS) -c $(INCLUDE) $< -o $@

$(BUILDDIR)/RHSPIDriver.o: $(RADIOHEADBASE)/RHSPIDriver.cpp
				$(CC) $(CFLAGS) -c $(INCLUDE) $< -o $@

$(BUILDDIR)/RHGenericDriver.o: $(RADIOHEADBASE)/RHGenericDriver.cpp
				$(CC) $(CFLAGS) -c $(INCLUDE) $< -o $@

$(BUILDDIR)/RHGenericSPI.o: $(RADIOHEADBASE)/RHGenericSPI.cpp
				$(CC) $(CFLAGS) -c $(INCLUDE) $< -o $@

rf69v: $(BUILDDIR)/rf69v.o $(BUILDDIR)/RH_RF69.o $(BUILDDIR)/RasPi.o $(BUILDDIR)/RHHardwareSPI.o $(BUILDDIR)/RHGenericDriver.o $(BUILDDIR)/RHGenericSPI.o $(BUILDDIR)/RHSPIDriver.o $(BUILDDIR)/APRSMsg.o $(BUILDDIR)/RFM69HCW.o
				mkdir -p build
				$(CC) $^ $(LIBS) -o $(BUILDDIR)/rf69v

clean:
				rm -rf $(BUILDDIR)/*.o $(BUILDDIR)/rf69v
