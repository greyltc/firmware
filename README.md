[![CI](https://github.com/greyltc/firmware/actions/workflows/build.yml/badge.svg)](https://github.com/greyltc/firmware/actions/workflows/build.yml)
# firmware
Firmware design files for controlling the hardware from https://github.com/greyltc/electronics and/or https://github.com/greyltc/hardware

## Usage
### Build
```
git clone https://github.com/greyltc/firmware.git && cd firmware
# optionally do hacking on the source code here
docker buildx build --progress plain --target compile --tag fwbuilt --load .  # compile and save all build artifacts in a docker image
docker buildx build --progress plain --target export --output type=local,dest=. .  # export built firmware to local fs
```
You should now have various compiled firmware .hex files that are ready to flash.
### Flash to hardware
Follow this if you'd like to program something with the firmware you've built.
#### From docker
If you've built with docker as described above, you could flash your hardware with the build artifacts in the fwbuilt image like this:
```
docker run --network=none --device=/dev/ttyACM0 fwbuilt pio run --project-dir /megaatmega2560_adc --target upload
```
where `/dev/ttyACM0` is the serial port device associated with the hardware you're flashing and `megaatmega2560_adc` is the name of the previously built firmware. You can list the names of the previously built firmwares with `docker run fwbuilt cat /revs.txt`.
#### Via docker
If all you have is a firmware blob, you might be able to flash it via a docker image with something like (for atmega2560):
```
cat megaatmega2560_adc.hex | docker run --rm --interactive --network=none --device=/dev/ttyACM0 ghcr.io/greyltc-org/firmware-builder:20220219.0.115 /root/.platformio/packages/tool-avrdude/avrdude -v -p atmega2560 -C /root/.platformio/packages/tool-avrdude/avrdude.conf -c wiring -b 115200 -D -P "/dev/ttyACM0" -U flash:w:-:i
```
where `/dev/ttyACM0` is the serial port device associated with the hardware you're flashing and `megaatmega2560_adc.hex` is the firmware blob you have. Or (for ATmega328PB):
```
cat ATmega328PB_ax0.hex | docker run --rm --interactive --network=none --device=/dev/ttyUSB1 ghcr.io/greyltc-org/firmware-builder:20220219.0.115 /root/.platformio/packages/tool-avrdude/avrdude -v -p atmega328pb -C /root/.platformio/packages/tool-avrdude/avrdude.conf -c arduino -b 115200 -D -P "/dev/ttyUSB1" -U flash:w:-:i
```
