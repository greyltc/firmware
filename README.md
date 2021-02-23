# firmware
Firmware design files for controlling the hardware from https://github.com/greyltc/electronics and/or https://github.com/greyltc/hardware

## Prepare
Install platformio (`yay -Syyu platformio` perhaps?)
```
git clone ${this_project}
git clone https://github.com/greyltc/ArduinoCore-avr.git # for my wire library mods
cd ${this_project}

# make sure we have the framework&package we need
pio platform install --with-package framework-arduino-avr atmelavr

mkdir -p pio-main
platformio init --board megaatmega2560 -d pio-main
pio lib -d pio-main install 872  # Arduino's Ethernet library
#pio lib -d pio-main install 342 # Adafruit's Arduino library for ADS1015/1115 ADCs (only needed for old hardware)
(cd pio-main/src && ln -sf ../../main-uC/* .)

mkdir -p pio-stepper
# currently needed until my wire mods are in MCUdude/MiniCore
platformio init --board ATmega328PB -d pio-stepper --project-option "lib_deps = framework-arduino-avr/Wire" --project-option "lib_extra_dirs = \$PROJECT_CORE_DIR/packages/framework-arduino-avr/libraries"
(cd pio-stepper/src && ln -sf ../../stepper-uC/* .)
```

## Usage
```
./compile-and-flash.sh pio-main # Build for main uC
./compile-and-flash.sh pio-stepper # Build for stepper uC

./compile-and-flash.sh pio-main fadsfdsafsad # list detected targets

./compile-and-flash.sh pio-main /dev/ttyUSBX # flash main
./compile-and-flash.sh pio-stepper /dev/ttyUSBX # flash stepper
```

### Misc PIO commands:
build:
```
# for the stepper uC:
pio run -d pio-stepper

# for the main uC:
pio run -d pio-main
```

flash:
```
#pio device list # to figure out where to upload
MAIN_PORT=/dev/ttyACMX
STEPPER_PORT=/dev/ttyACMX

# for the stepper uC:
pio run -d pio-stepper --target upload --upload-port ${STEPPER_PORT}

# for the main uC:
pio run -d pio-main --target upload --upload-port ${MAIN_PORT}
```

debug: 
```
# for the stepper uC:
device monitor -b 115200 -p ${STEPPER_PORT}

# for the main uC:
device monitor -b 115200 -p ${MAIN_PORT}
```

## AVRDUDE
### Flashing a .hex
Flashing a raw .hex firmware file for the stepper driver might look something like this:
```
~/.platformio/packages/tool-avrdude/avrdude -v -p atmega328pb -C ~/.platformio/packages/tool-avrdude/avrdude.conf -c arduino -b 115200 -D -P "/dev/ttyUSBX" -U flash:w:.pio/build/ATmega328PB/firmware.hex:i
```
Main controller flash might look like this:
```
~/.platformio/packages/tool-avrdude/avrdude -v -p atmega2560 -C ~/.platformio/packages/tool-avrdude/avrdude.conf -c wiring -b 115200 -D -P "/dev/ttyUSBX" -U flash:w:.pio/build/megaatmega2560/firmware.hex:i
```
### Dumping a .hex
Dumping a .hex firmware file for the stepper driver might look like this:
```
~/.platformio/packages/tool-avrdude/avrdude -v -p atmega328pb -C ~/.platformio/packages/tool-avrdude/avrdude.conf -c arduino -b 115200 -D -P "/dev/ttyUSBX" -U flash:r:stepper_firmware_dump.hex:i
```
Consider repeating the read and write commands replacing `flash:` once each with `:eeprom`, `:hfuse`, `:lfuse`, `:efuse` though `man avrdude` shows many more memory types.  
Main controller dump might look like this:
```
~/.platformio/packages/tool-avrdude/avrdude -v -p atmega2560 -C ~/.platformio/packages/tool-avrdude/avrdude.conf -c wiring -b 115200 -D -P "/dev/ttyUSBX" -U flash:r:main_firmware_dump.hex:i
```
### Verifying
You can verify a `:flash` firmware file on disk matches that in a device with:
```
~/.platformio/packages/tool-avrdude/avrdude -v -p atmega328pb -C ~/.platformio/packages/tool-avrdude/avrdude.conf -c arduino -b 115200 -D -P "/dev/ttyUSBX" -U flash:v:file_on_disk.hex:i
```
