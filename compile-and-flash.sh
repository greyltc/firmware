#!/usr/bin/env bash

# one time:
#git clone this_repo
#cd this_repo
#yay -Syyu platformio
#mkdir -p pio
#pushd pio
#platformio init --board megaatmega2560
#platformio lib install 872 # install Ethernet library
#ln -s ../../microC/firmware.ino src/.
#popd

# compile and flash
#pio device list # to figure out where to upload
DEVICE_PORT=/dev/ttyACM0
cd pio && pio run --target upload --upload-port ${DEVICE_PORT}
#pio device monitor -b 115200 -p ${DEVICE_PORT}

