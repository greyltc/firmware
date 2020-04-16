# firmware
Firmware design files for controlling the hardware from https://github.com/greyltc/electronics and/or https://github.com/greyltc/hardware

## Compile and flash
Via [platformio](https://github.com/arduino/arduino-cli)  
Install platformio (yay -Syyu platformio)
```
git clone ${this_project} && cd ${this_project}
mkdir -p pio
cd pio
platformio init --board megaatmega2560
pio lib install 872 # Arduino's Ethernet library
#pio lib install 342 # Adafruit's Arduino library for ADS1015/1115 ADCs (only needed for old hardware)
ln -sf ../../microC/firmware.ino src/.
#pio device list # to figure out where to upload
DEVICE_PORT=/dev/ttyACMX
pio run --target upload --upload-port ${DEVICE_PORT}

# for debug
#device monitor -b 115200 -p ${DEVICE_PORT}

```
