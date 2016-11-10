#!/bin/bash

DAEMONPARAM="-daemon"

if [ $# -ge 3 ] ; then
  if [ $3 == "--no-daemon" ] ; then
    DAEMONPARAM=""  
  fi
fi

if [ ! -f "/opt/smartsoft/start-smartsoft.sh" ] ; then
  /usr/local/robotino/daemons/bin/laserd2 -v ${DAEMONPARAM} -device=/dev/$1 -channel=$2
fi

