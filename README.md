# firmware
Firmware design files for controlling the hardware from https://github.com/mutovis/hardware

## Compiling
Via [arduino-cli](https://github.com/arduino/arduino-cli):
```
# you only need to do the following three lines once
arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli lib install "Ethernet"
arduino-cli lib install "SD"

cd mainArduino
arduino-cli compile -v -o build/mutovis_firmware -b arduino:avr:mega .
# now you have the firmware in build/mutovis_firmware.hex
```

## Flashing
Via [avrdude](http://www.nongnu.org/avrdude/):
```
# first, figure out what port your arduino is on:
arduino-cli board list
# then flash the firmware you compiled above (after editing in the correct value for -P you just discovered)
cd mainArduino
avrdude -v -C/etc/avrdude.conf -patmega2560 -cwiring -P/dev/ttyACMX -b115200 -D -Uflash:w:build/mutovis_firmware.hex:i
```
