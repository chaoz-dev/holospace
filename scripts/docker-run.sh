#!/bin/bash

docker run --rm                         \
    --user docker                       \
    --device /dev/bus/usb:/dev/bus/usb  \
    holospace /opt/holospace/bin/holospace
