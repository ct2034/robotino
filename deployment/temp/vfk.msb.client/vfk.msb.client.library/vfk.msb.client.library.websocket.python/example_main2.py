import logging

FORMAT = "%(asctime)s %(levelname)s %(message)s"
logging.basicConfig(format=FORMAT, level=logging.INFO)

import time  # sleep()

from vfk_msb_py.msb_ws4py_client import MsbWsClient
from vfk_msb_py.msb_communicate import *


def callback(m):
    """ this method is called whenever data is received via websocket """
    logging.info(m)


def functionCallback(md):
    """ this method is called whenever a function callback is received via websocket """
    logging.info("This is function callback, data:")
    logging.info(md)


if __name__ == '__main__':
    logging.info("init MsbWsClient .. ")
    try:
        # mwc = MsbWsClient('ws://10.3.6.140:8085', callback)
        # mwc = MsbWsClient('wss://10.3.6.140:8084', callback)
        # mwc = MsbWsClient('ws://atm.virtualfortknox.de/msb', callback)
        mwc = MsbWsClient('wss://localhost:8084', callback)

        # Starting the client with a url to connect to and a callback function
        # Examples for possible URLs are:
        # * 'ws://10.3.6.140:8085'
        # * 'ws://demo.virtualfortknox.de/msb2
        # The callback method is called whenever a function of the smart object
        # is called or data is recieved via the websocket connection

        e = Event(eventId="vector_event",
                  name="vector",
                  description="A vector",
                  dataFormat=ComplexDataFormat(properties=[
                      DataFormat("x", "Float"),
                      DataFormat("y", "Float"),
                      DataFormat("z", "Float")
                  ])
                  )

        e2 = Event(eventId="empty",
                   name="empty event",
                   description="")
        # This is an example for a event declaration (data to be sent by this)

        f = Function(functionId="vector_function",
                     name="vector",
                     description="A vector",
                     dataFormat=ComplexDataFormat(name="vector", properties=[
                         DataFormat("x", doc_type="Float"),
                         DataFormat("y", doc_type="Float"),
                         DataFormat("z", doc_type="Float")
                     ]),
                     callback=functionCallback
                     )

        f2 = Function(functionId="empty",
                      name="empty",
                      description="",
                      callback=functionCallback)

        f3 = Function(functionId="arrayfun",
                      name="array",
                      description="array input test",
                      dataFormat=DataFormat(is_array=True,
                                            doc_type="String",
                                            name="a"),
                      callback=functionCallback)

        s = SmartObject(
            "testclient_uuid2",
            "testclient_name2",
            "testclient_desc2",
            [e, e2],
            [f, f2, f3],
            "token2")
        # This is the whole self description of a smart object

        mwc.register(s)
        # This registers the smart object with the msb

        time.sleep(2)
        # We have to wait for a bit to be sure the object is registered

        while True:
            data = {"x": 1, "y": 2, "z": 3}
            mwc.emit_event(s, e, data=data, prio=2)
            # mwc.emit_event(s, e2)
            # logging.info("Emitting a test event")
            # sys.stdout.flush()
            # # this emits the data '3' as an event to the msb
            time.sleep(10)

    except KeyboardInterrupt:
        logging.info("EXIT ..")
        mwc.disconnect()
        # You have to handel the disconnection of the client
