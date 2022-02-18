# syntax=docker/dockerfile:1.3-labs
FROM ghcr.io/greyltc-org/firmware-builder:20220218.0.109 as compile
COPY main-uC /megaatmega2560/src
COPY stepper-uC /ATmega328PB/src
RUN --network=none <<EOF
#!/usr/bin/env bash
mkdir -p /out
touch /revs.txt

BOARD=megaatmega2560

REV=adc
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


BOARD=ATmega328PB

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
