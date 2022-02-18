# syntax=docker/dockerfile:1.3-labs
FROM ghcr.io/greyltc-org/firmware-builder:20220218.0.108 as compile
COPY main-uC /megaatmega2560/src
COPY stepper-uC /ATmega328PB/src
RUN --network=none <<EOF
#!/usr/bin/env bash
mkdir -p /out

BOARD=megaatmega2560
touch /${BOARD}/revs.txt

REV=adc
cat ${REV} >> /${BOARD}/revs.txt
cp -a /${BOARD} /${BOARD}_${REV}
pio run -d /${BOARD}_${REV}
cp /${BOARD}_${REV}/.pio/build/${BOARD}/firmware.hex /out/${BOARD}_${REV}.hex

export PLATFORMIO_BUILD_FLAGS=-DNO_ADC
REV=noadc
cat ${REV} >> /${BOARD}/revs.txt
cp -a /${BOARD} /${BOARD}_${REV}
pio run -d /${BOARD}_${REV}
cp /${BOARD}_${REV}/.pio/build/${BOARD}/firmware.hex /out/${BOARD}_${REV}.hex



BOARD=ATmega328PB
touch /${BOARD}/revs.txt

REV=ax0
cat ${REV} >> /${BOARD}/revs.txt
cp -a /${BOARD} /${BOARD}_${REV}
sed --in-place 's,^#define I2C_SLAVE_ADDRESS.*,#define I2C_SLAVE_ADDRESS 0x50,g' /${BOARD}_${REV}/src/firmware.cpp
pio run -d /${BOARD}_${REV}
cp /${BOARD}_${REV}/.pio/build/${BOARD}/firmware.hex /out/${BOARD}_${REV}.hex

REV=ax1
cat ${REV} >> /${BOARD}/revs.txt
cp -a /${BOARD} /${BOARD}_${REV}
sed --in-place 's,^#define I2C_SLAVE_ADDRESS.*,#define I2C_SLAVE_ADDRESS 0x51,g' /${BOARD}_${REV}/src/firmware.cpp
pio run -d /${BOARD}_${REV}
cp /${BOARD}_${REV}/.pio/build/${BOARD}/firmware.hex /out/${BOARD}_${REV}.hex

REV=ax2
cat ${REV} >> /${BOARD}/revs.txt
cp -a /${BOARD} /${BOARD}_${REV}
sed --in-place 's,^#define I2C_SLAVE_ADDRESS.*,#define I2C_SLAVE_ADDRESS 0x52,g' /${BOARD}_${REV}/src/firmware.cpp
pio run -d /${BOARD}_${REV}
cp /${BOARD}_${REV}/.pio/build/${BOARD}/firmware.hex /out/${BOARD}_${REV}.hex

EOF

FROM scratch AS export
COPY --from=compile /out/* /
