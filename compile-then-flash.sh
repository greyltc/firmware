#!/usr/bin/env bash

# typical usage is `./compile-then-flash.sh /dev/ttyACM0`

# the first (only) argument to this script must be the arduino's port
# find it by running `arduino-cli board list`

# this must have been done once at some point before running the flash command below:
#arduino-cli core update-index
#arduino-cli core install arduino:avr
#arduino-cli lib install "Ethernet"

# user must have write permissions for the arduino port (probably enough to be in the uucp group)

echo "======COMPILING======"
./compile.sh
echo "======FLASHING======"
./flash.sh "$1"
