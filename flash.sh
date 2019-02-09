#!/usr/bin/env bash

# typical usage is `./flash.sh /dev/ttyACM0`

# the first (only) argument to this script must be the arduino's port
# find it by running `arduino-cli board list`

# this must have been done once at some point before running the flash command below:
#arduino-cli core update-index
#arduino-cli core install arduino:avr
#arduino-cli lib install "Ethernet"

# user must have write permissions for the arduino port (probably enough to be in the uucp group)

cd mainArduino/build
avrdude -C/etc/avrdude.conf -v -patmega2560 -cwiring -P"$1" -b115200 -D -Uflash:w:mutovis_firmware.hex:i
