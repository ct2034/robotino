#!/usr/bin/env python

import uuid
import pkg_resources
from ws4py.client.threadedclient import WebSocketClient

from msb_communicate import *


class MsbWsClient(WebSocketClient):
    def __init__(self, url):
        # check ws4py version
        env = pkg_resources.Environment()
        v = env['ws4py'][0].version.split(".")
        if ((int(v[0])*100 + int(v[1])*10 + int(v[2])) < 35):
            raise Exception("please use ws4py version 0.3.5 or later")

        self.conUUid = uuid.uuid1()  # make uuid for connection
        self.conUrl = (
            url +
            "/websocket/data/" +
            str(self.conUUid.get_clock_seq_hi_variant()) +
            "/" +
            str(self.conUUid) +
            "/websocket"
        )
        print "initialized " + self.conUrl
        try:
            WebSocketClient.__init__(self, self.conUrl)
        except Exception, e:
            raise e

    def register(self, service):
        mc = MsbCommunicate()
        send_str = mc.getRegisterMessage(service)
        self.send(send_str)

    def emitEvent(self, service, event, data, prio):
        mc = MsbCommunicate()
        send_str = mc.getEventMessage(service, event, data, prio)
        self.send(send_str)

    def closed(self, code, reason=None):
        print "Closed down", code, reason

    def received_message(self, m):
        m = str(m)
        print m

    def msg_send(self, str):
        self.send(str)
