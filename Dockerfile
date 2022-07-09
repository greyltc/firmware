# syntax=docker.io/docker/dockerfile:1.4.2
FROM ghcr.io/greyltc-org/firmware-builder:20220709.0.148 as compile
#FROM ghcr.io/greyltc-org/firmware-builder:20220220.0.117 as compile
COPY main-uC /megaatmega2560/src
COPY squirrel /megaatmega2560sq/src
COPY stepper-uC /ATmega328PB/src
RUN --network=none <<EOF
#!/usr/bin/env bash
set -e
set -o pipefail

mkdir -p /out
touch /revs.txt

BOARD=megaatmega2560
grep -r 'define FIRMWARE_VER' /${BOARD}

REV=baseline
BREV=${BOARD}_${REV}
echo ${BREV} >> /revs.txt
cp -a /${BOARD} /${BREV}
pio run -d /${BREV}
cp /${BREV}/.pio/build/${BOARD}/firmware.hex /out/${BREV}.hex

export PLATFORMIO_BUILD_FLAGS=-DNO_ADC
REV=noadc
BREV=${BOARD}_${REV}
echo ${BREV} >> /revs.txt
cp -a /${BOARD} /${BREV}
pio run -d /${BREV}
cp /${BREV}/.pio/build/${BOARD}/firmware.hex /out/${BREV}.hex
unset PLATFORMIO_BUILD_FLAGS

export PLATFORMIO_BUILD_FLAGS=-DADS1015
REV=ads1015
BREV=${BOARD}_${REV}
echo ${BREV} >> /revs.txt
cp -a /${BOARD} /${BREV}
pio run -d /${BREV}
cp /${BREV}/.pio/build/${BOARD}/firmware.hex /out/${BREV}.hex
unset PLATFORMIO_BUILD_FLAGS

REV=no_spi_bb
BREV=${BOARD}_${REV}
echo ${BREV} >> /revs.txt
cp -a /${BOARD} /${BREV}
sed --in-place 's,^#define BIT_BANG_SPI,//#define BIT_BANG_SPI,g' /${BREV}/src/firmware.cpp
pio run -d /${BREV}
cp /${BREV}/.pio/build/${BOARD}/firmware.hex /out/${BREV}.hex


rm -r /${BOARD}/src
cp -a /${BOARD}sq/src /${BOARD}/src
grep -r 'define FIRMWARE_VER' /${BOARD}

REV=squirrel
BREV=${BOARD}_${REV}
echo ${BREV} >> /revs.txt
cp -a /${BOARD} /${BREV}
pio run -d /${BREV}
cp /${BREV}/.pio/build/${BOARD}/firmware.hex /out/${BREV}.hex


BOARD=ATmega328PB
grep -r 'define FIRMWARE_VER' /${BOARD}

REV=ax0
BREV=${BOARD}_${REV}
echo ${BREV} >> /revs.txt
cp -a /${BOARD} /${BREV}
sed --in-place 's,^#define I2C_SLAVE_ADDRESS.*,#define I2C_SLAVE_ADDRESS 0x50,g' /${BREV}/src/firmware.cpp
pio run -d /${BREV}
cp /${BREV}/.pio/build/${BOARD}/firmware.hex /out/${BREV}.hex

REV=ax1
BREV=${BOARD}_${REV}
echo ${BREV} >> /revs.txt
cp -a /${BOARD} /${BREV}
sed --in-place 's,^#define I2C_SLAVE_ADDRESS.*,#define I2C_SLAVE_ADDRESS 0x51,g' /${BREV}/src/firmware.cpp
pio run -d /${BREV}
cp /${BREV}/.pio/build/${BOARD}/firmware.hex /out/${BREV}.hex

REV=ax2
BREV=${BOARD}_${REV}
echo ${BREV} >> /revs.txt
cp -a /${BOARD} /${BREV}
sed --in-place 's,^#define I2C_SLAVE_ADDRESS.*,#define I2C_SLAVE_ADDRESS 0x52,g' /${BREV}/src/firmware.cpp
pio run -d /${BREV}
cp /${BREV}/.pio/build/${BOARD}/firmware.hex /out/${BREV}.hex

EOF

FROM scratch AS export
COPY --from=compile /out/* /
