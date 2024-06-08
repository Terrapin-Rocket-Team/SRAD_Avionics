#!/bin/bash

echo " --------------------------------------------------"
echo " | Starting Avionics Live Video Setup Script v1.0 |"
echo " --------------------------------------------------"

if [ $EUID -ne 0 ]
then 
  echo "Error: Please run as root"
  exit 1
fi

echo "Checking dependencies..."
sleep 1

echo -n "Checking for git..."
if ! type git &> /dev/null
then
    echo "Not Found"
    echo "Installing git..."
    apt install git -y
else
    echo "Found"
fi

echo -n "Checking for cmake..."
if ! type cmake &> /dev/null
then
    echo "Not Found"
    echo "Installing cmake..."
    apt install cmake -y
else
    echo "Found"
fi

echo -n "Checking for ffmpeg..."
if ! type ffmpeg &> /dev/null
then
    echo "Not Found"
    echo "Installing ffmpeg..."
    apt install ffmpeg -y
else
    echo "Found"
fi

echo -n "Checking for perl..."
if ! type perl &> /dev/null
then
    echo "Not Found"
    echo "Installing perl..."
    apt install perl -y
else
    echo "Found"
fi

echo -n "Checking for libcamera..."
if ! type libcamera-vid &> /dev/null
then
    echo "Not Found"
    echo "Installing libcamera..."
    apt install libcamera-apps -y
else
    echo "Found"
fi

echo -n "Checking for netcat..."
if ! type nc &> /dev/null
then
    echo "Not Found"
    echo "Installing netcat..."
    apt install netcat-traditional -y
else
    echo "Found"
fi

echo -n "Checking for Boost 1.74..."
if [ ! -d /usr/include/boost ] || [ ! -f /usr/lib/aarch64-linux-gnu/libboost_program_options.so ]
then
    echo "Not Found"
    echo "Installing Boost 1.74"
    apt install libboost1.74-all-dev -y
else
    echo "Found"
fi

echo "Getting AV1 encoder..."
sleep 1

if [ -d aom ]
then
    cd aom
    if [ ! git rev-parse --is-inside-work-tree &> /dev/null ]
    then
        cd ..
        rm -rf aom
        git clone https://aomedia.googlesource.com/aom
    else
        cd ..
        echo "Found git repository, assuming it is the right one"
    fi
else
    git clone https://aomedia.googlesource.com/aom
fi

cd aom
mkdir -p build && cd build

echo "Building AV1 encoder..."
sleep 1

cmake ..
make

echo "Installing encoder..."
sleep 1

make install

cd ../../

echo "Building teensy interface..."
sleep 1

cd teensy-interface
make

echo "Finishing up..."
sleep 1

cd ../

chmod +x tests/localsave.sh
chmod +x tests/localsave1080p.sh
chmod +x tests/localsave1080pv2.sh
chmod +x tests/networkstream.sh

crontab -u ${SUDO_USER} crontab-config

sudo -u ${SUDO_USER} bash -c "mkdir -p ~/video"

echo "Setup complete!"