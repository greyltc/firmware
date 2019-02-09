#!/usr/bin/env bash

# one time:
#arduino-cli core update-index
#arduino-cli core install arduino:avr
#arduino-cli lib install "Ethernet"

arduino-cli compile -o build/mutovis_firmware -b arduino:avr:mega . 
