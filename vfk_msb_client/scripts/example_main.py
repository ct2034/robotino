#!/usr/bin/env python2

import time  # sleep()

from msb_websocket_client_py import MsbWsClient
from msb_classes import *
from msb_communicate import *


def callback(m):
    print(m)

if __name__ == '__main__':
    """example on how to use the python library"""
    print("init MsbWsClient .. ")
    try:
        mwc = MsbWsClient('ws://demo.virtualfortknox.de/msb2', callback)

        e = Event("NUM",
                  "number",
                  "A number",
                  [DataFormat("Start", "Integer").toCol()])
        s = Application(
            "testclient_uuid2",
            "testclient_name",
            "testclient_desc",
            [e.toOrderedDict()],
            [],
            "testclient_token2")

        mwc.register(s)

        # wait a bit
        time.sleep(2)

        mc = MsbCommunicate()
        mwc.emitEvent(s, e, 3, 2)

    except KeyboardInterrupt:
        mwc.close()
