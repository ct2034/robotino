#!/usr/bin/env python

import threading
import uuid

import pkg_resources
from ws4py.client.threadedclient import WebSocketClient

from vfk_msb_py.msb_communicate import *


class MsbWsClient(WebSocketClient):
    def __init__(self, url, defaultCallback):
        self.connected = False

        # check ws4py version
        env = pkg_resources.Environment()
        v = env['ws4py'][0].version.split(".")
        if (int(v[0]) * 100 + int(v[1]) * 10 + int(v[2])) < 35:
            raise RuntimeError("please use ws4py version 0.3.5 or higher")

        self.conUUid = uuid.uuid1()  # make uuid for connection

        self.conUrl = (
            url +
            "/websocket/data/" +
            str(self.conUUid.clock_seq_hi_variant) +
            "/" +
            str(self.conUUid) +
            "/websocket"
        )
        self.defaultCallback = defaultCallback
        self.functionCallbacks = {}
        logging.info("initialized " + self.conUrl)
        try:
            WebSocketClient.__init__(self, self.conUrl)
            self.connect()
            self.connected = True
            logging.info("connected: " + str(self.connected))
            threading.Thread(target=self.run_forever).start()
        except KeyboardInterrupt:
            self.disconnect()
            logging.error("KeyboardInterrupt")
        except Exception as e:
            self.disconnect()
            raise e

    def callback(self, msg):
        if str(msg).startswith('a["C {'):
            mc = MsbCommunicate()
            msgDic = mc.handleCallback(str(msg))
            if self.functionCallbacks[msgDic["functionId"]]:
                self.functionCallbacks[msgDic["functionId"]](msgDic)
            else:
                self.defaultCallback(msg)
        else:
            self.defaultCallback(msg)

    """register a smart object to msb"""

    def register(self, service):
        mc = MsbCommunicate()
        send_str = mc.getRegisterMessage(service)
        for f in service.functions:
            self.functionCallbacks[f.functionId] = f.callback

        self.send(send_str)
        logging.debug("send_str: " + str(send_str))

    """emit an event to msb"""

    def emit_event(self, service, event, data={}, prio=1):
        mc = MsbCommunicate()
        send_str = mc.getEventMessage(service, event, data, prio)
        if self.connected:
            self.send(send_str)
            logging.debug("send_str: " + str(send_str))
        else:
            logging.debug("connected: " + str(self.connected))

    """close connection to msb"""

    def disconnect(self):
        if self.connected:
            self.close()
            self.connected = False
        logging.info("[disconnect] connected: " + str(self.connected))

    """returns information whether connection is IO or not"""

    def get_io(self):
        return self.connected

    # ---------------
    # private methods

    """private method"""

    def closed(self, code, reason=None):
        self.connected = False
        logging.info("[closed] connected: " + str(self.connected))
        logging.info("Closed down", code, reason)

    """private method"""

    def received_message(self, m):
        logging.debug(str(m))
        self.callback(m)

    """private method"""

    def msg_send(self, str):
        self.send(str)
