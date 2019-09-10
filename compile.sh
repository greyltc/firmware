#!/usr/bin/env bash

# one time:
#arduino-cli core update-index
#arduino-cli core install arduino:avr
#arduino-cli lib install "Ethernet"

cd mainArduino
arduino-cli compile -v -o build/mutovis_firmware -b arduino:avr:mega .
#arduino-cli compile --show-properties -v -o build/mutovis_firmware -b arduino:avr:mega .
rm -rf /tmp/arduino-*
