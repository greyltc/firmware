# firmware
Firmware design files for controlling the hardware from https://github.com/greyltc/electronics and/or https://github.com/greyltc/hardware

## Prepare
Install platformio (yay -Syyu platformio)
```
git clone ${this_project}
git clone https://github.com/greyltc/ArduinoCore-avr.git # for my wire library mods
cd ${this_project}

mkdir -p pio-main
platformio init --board megaatmega2560 -d pio-main
pio lib -d pio-main install 872  # Arduino's Ethernet library
#pio lib -d pio-main install 342 # Adafruit's Arduino library for ADS1015/1115 ADCs (only needed for old hardware)
(cd pio-main/src && ln -sf ../../../ArduinoCore-avr/libraries/Wire/src/* .) # for my wire library mods)
(cd pio-main/src && ln -sf ../../main-uC/* .)

mkdir -p pio-stepper
platformio init --board ATmega328PB -d pio-stepper
(cd pio-stepper/src && ln -sf ../../../ArduinoCore-avr/libraries/Wire/src/* .) # currently needed until my my wire library mods are merged
(cd pio-stepper/src && ln -sf ../../stepper-uC/* .)
```

## Build
```
# for the stepper uC:
pio run -d pio-stepper

# for the main uC:
pio run -d pio-main
```

## Flash
```
#pio device list # to figure out where to upload
MAIN_PORT=/dev/ttyACMX
STEPPER_PORT=/dev/ttyACMX

# for the stepper uC:
pio run -d pio-stepper --target upload --upload-port ${STEPPER_PORT}

# for the main uC:
pio run -d pio-main --target upload --upload-port ${MAIN_PORT}
```

## Debug
```
# for the stepper uC:
device monitor -b 115200 -p ${STEPPER_PORT}

# for the main uC:
device monitor -b 115200 -p ${MAIN_PORT}
```

