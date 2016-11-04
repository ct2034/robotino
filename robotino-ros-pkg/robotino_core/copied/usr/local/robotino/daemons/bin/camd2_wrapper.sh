#!/bin/bash

/usr/local/robotino/daemons/bin/camd2 -daemon -device=/dev/video$1 -channel=$1 -name=$2

