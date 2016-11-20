#!/bin/bash

docker run --rm                         \
    --user docker                       \
    --device /dev/bus/usb:/dev/bus/usb  \
    qmulus /opt/Qmulus/bin/Qmulus
