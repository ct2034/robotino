#!/usr/bin/env python2
import uuid
import websocket
from time import sleep
from thread import start_new_thread

from msb_communicate import *


def on_error(ws, error):
    """websocket callback for errors"""
    print "ERROR: " + error


def on_close(ws):
    """websocket callback for closing of the socket"""
    print "Closed down", code, reason
    raise Exception(
        "Disconnected from websocket: " +
        str(code) +
        " " +
        str(reason)
    )


def on_open(ws):
    """websocket callback for opening of the socket"""
    print "OPENED"


class MsbWsClient():
    """the used websocket client"""
    def __init__(self, url, callbackFunction):
        self.conUUid = uuid.uuid1()  # make uuid for connection
        self.conUrl = (
            url +
            "/websocket/data/" +
            str(self.conUUid.get_clock_seq_hi_variant()) +
            "/" +
            str(self.conUUid) +
            "/websocket"
        )
        try:
            self.mc = MsbCommunicate()
            self.ws = websocket.WebSocketApp(
                self.conUrl,
                on_message=callbackFunction,
                on_error=on_error,
                on_close=on_close
            )
            self.ws.on_open = on_open
            start_new_thread(self.ws.run_forever, ())
            print "initialized " + self.conUrl
            sleep(.2)
        except Exception, e:
            raise e

    def register(self, service):
        """websocket callback for opening of the socket"""
        send_str = self.mc.getRegisterMessage(service)
        self.msg_send(send_str)

    def emitEvent(self, service, event, data, prio):
        send_str = self.mc.getEventMessage(service, event, data, prio)
        self.msg_send(send_str)

    def msg_send(self, m):
        self.ws.send(str(m))
