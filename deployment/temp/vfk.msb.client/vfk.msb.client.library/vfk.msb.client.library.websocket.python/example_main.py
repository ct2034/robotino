import logging

import time  # sleep()
# from exceptions import KeyboardInterrupt

FORMAT = "%(asctime)s %(levelname)s %(message)s"
logging.basicConfig(format=FORMAT, level=logging.INFO)

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

        e3 = Event(eventId="ARR",
                   name="array_test",
                   description="an array",
                   dataFormat=DataFormat(
                       doc_type="String",
                       is_array=True))
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

        s = SmartObject(
            "testclient_uuid",
            "testclient_name",
            "testclient_desc",
            [e, e2, e3],
            [f, f2],
            "newtoken")
        # This is the whole self description of a smart object

        mwc.register(s)
        # This registers the smart object with the msb

        time.sleep(2)
        # We have to wait for a bit to be sure the object is registered

        while True:
            #     data = {"x": 1, "y": 2, "z": 3}
            #     mwc.emit_event(s, e, data=data, prio=2)
            mwc.emit_event(s, e3, data=["Peter", "Craig"])
            logging.info("Emitting a test event")
            # sys.stdout.flush()
            time.sleep(1)

    except KeyboardInterrupt:
        logging.info("EXIT ..")
        mwc.disconnect()
        # You have to handel the disconnection of the client
