#!/usr/bin/env python2

import time  # sleep()

from vfk_msb_py.msb_ws4py_client import MsbWsClient
from vfk_msb_py.msb_classes import *
from vfk_msb_py.msb_communicate import *

msgs = 0

def callback(m):
    try:
        if str(m).startswith("a[\"C {\\"):
            # print "callback"
            addCount()
    except Exception as e:
        print "Callback Exception {0}".format(e)

def printCount():
    global msgs
    print(str(msgs)+" Messages per second")
    msgs = 0

def addCount():
    global msgs
    msgs = msgs + 1

if __name__ == '__main__':
    """example on how to use the python library"""

    print("init MsbWsClient .. ")
    try:
        mwc = MsbWsClient('wss://10.3.6.140:8084', callback)

        f = Function("num",
                     # there can not be capital letters in name
                     DataFormat("start", "Integer").toCol(),
                     "number",
                     "A number")
        s = Application(
            "testclient_uuid2",
            "testclient_name2",
            "testclient_desc2",
            [],
            [f.toOrderedDict()],
            "token2")

        mwc.register(s)

        # wait a bit
        time.sleep(2)

        # mwc.emitEvent(s, e.toOrderedDict(), 3, 2)

        printCount()

        while True:
            time.sleep(1)
            printCount()

    except KeyboardInterrupt:
        print("EXIT ..")
        mwc.disconnect()
