# syntax=docker/dockerfile:1.3
FROM ghcr.io/greyltc-org/firmware-builder:20220216.0.104 as compile
COPY main-uC /megaatmega2560/src
COPY stepper-uC /ATmega328PB/src
RUN --network=none pio run -d /megaatmega2560
RUN --network=none pio run -d /ATmega328PB

FROM scratch AS export
ENV BOARD=ATmega328PB
COPY --from=compile /${BOARD}/.pio/build/${BOARD}/firmware.hex /${BOARD}.hex
ENV BOARD=megaatmega2560
COPY --from=compile /${BOARD}/.pio/build/${BOARD}/firmware.hex /${BOARD}.hex
