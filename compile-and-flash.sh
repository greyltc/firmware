#!/usr/bin/env bash

PROJECT="$1"
DEVICE_PORT=$2

print_usage(){
  echo "Usage: `basename $0` pio_folder [device_port]"
  echo "where pio-folder is the fully set-up project folder (see README.md)"
  echo "and device_port is the port to use to program the device."
  echo "not specifying a device_port only builds."
  echo "maybe you can find the right device_port in the output below:"
  pio device list
  exit -1
}


if [ $# -ne "2" ] && [ $# -ne "1" ]
then
  print_usage
fi

if [ $# -eq "2" ]
then
  if [ -c ${DEVICE_PORT} ]
  then
    pio run -v -t clean -d "${PROJECT}"
    pio run --verbose -d "${PROJECT}" --target upload --upload-port ${DEVICE_PORT}
  else
    echo "ERROR: ${DEVICE_PORT} is not a character device"
    print_usage
  fi
fi


if [ $# -eq "1" ] 
then
  pio run -v -t clean -d "${PROJECT}"
  pio run -v -d "${PROJECT}"
fi
