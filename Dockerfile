# syntax=docker/dockerfile:1.3-labs
FROM ghcr.io/greyltc-org/firmware-builder:20220216.0.104 as compile
COPY main-uC /megaatmega2560/src
COPY stepper-uC /ATmega328PB/src
RUN --network=none <<EOF
#!/usr/bin/env bash
mkdir -p /out

BOARD=megaatmega2560
pio run -d /${BOARD}
cp /${BOARD}/.pio/build/${BOARD}/firmware.hex /out/${BOARD}_adc.hex
export PLATFORMIO_BUILD_FLAGS=-DNO_ADC
pio run -d /${BOARD}
cp /${BOARD}/.pio/build/${BOARD}/firmware.hex /out/${BOARD}_noadc.hex

BOARD=ATmega328PB
sed --in-place 's,^#define I2C_SLAVE_ADDRESS.*,#define I2C_SLAVE_ADDRESS 0x50,g' /${BOARD}/src/firmware.cpp
pio run -d /${BOARD}
cp /${BOARD}/.pio/build/${BOARD}/firmware.hex /out/${BOARD}_ax0.hex
sed --in-place 's,^#define I2C_SLAVE_ADDRESS.*,#define I2C_SLAVE_ADDRESS 0x51,g' /${BOARD}/src/firmware.cpp
pio run -d /${BOARD}
cp /${BOARD}/.pio/build/${BOARD}/firmware.hex /out/${BOARD}_ax1.hex
sed --in-place 's,^#define I2C_SLAVE_ADDRESS.*,#define I2C_SLAVE_ADDRESS 0x52,g' /${BOARD}/src/firmware.cpp
pio run -d /${BOARD}
cp /${BOARD}/.pio/build/${BOARD}/firmware.hex /out/${BOARD}_ax2.hex

EOF

FROM scratch AS export
COPY --from=compile /out/* /

