# firmware
Firmware design files for controlling the hardware from https://github.com/mutovis/hardware

## Compiling
Via [arduino-cli](https://github.com/arduino/arduino-cli):
```
arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli lib install "Ethernet"

arduino-cli compile -o mainArduino/build/mutovis_firmware -b arduino:avr:mega mainArduino/
```

## Flashing
Via [arduino-cli](https://github.com/arduino/arduino-cli):
```
# first, figure out what port your arduino is on:
arduino-cli board list
# then flash the firmware you compiled above
arduino-cli upload -p /dev/ttyACMX -b arduino:avr:mega mainArduino/
```
