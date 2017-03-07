#!/usr/bin/env python2
import logging
import re

import iso8601
from datetime import datetime

from vfk_msb_py.msb_classes import *


class MsbCommunicate:
    """interfaces to talk to MSB / VFK"""

    def getRegisterMessage(self, service):
        """get the message to register a smart object"""
        return self.makeMsgWithLetter(
            "R",
            service.toJson())

    def getEventMessage(self, service, event, data, prio):
        """get the message to emit an event"""
        msg = self.makeMsgWithLetter(
            "E",
            self.eventToJson(
                service=service,
                event=event,
                data=data,
                prio=prio
            )
        )
        return msg

    def eventToJson(self, service, event, data, prio):
        """parse an event to json"""
        t = datetime.now()
        d = self.getDict(data)
        logging.debug('DICT:')
        logging.debug(d)

        od_send = OrderedDict([
            ("uuid", service.uuid),
            ("eventId", event.eventId),
            ("priority", prio),
            ("postDate", self.getRfc3339(t)),
            ("dataObject", d)
        ])
        return json.dumps(od_send, sort_keys=True, separators=(',', ':'))

    def getDict(self, d):
        """retrieve dict of an objects parameters"""
        logging.debug('======= get Dict')
        logging.debug(type(d))
        try:  # is object
            logging.debug(d.__slots__)
            logging.debug('is object')
            slots = d.__slots__
            dic = {}
            for slot in slots:
                content = eval("d." + slot)
                dic[slot] = self.getDict(content)
            return dic
        except AttributeError:
            if type(d) is list:  # is list
                l = []
                for one in d:
                    l.append(self.getDict(one))
                logging.debug(l)
                logging.debug('is list')
                return l
            else:  # is parameter
                logging.debug(d)
                logging.debug('is parameter')
                return d

    def handleCallback(self, msg):
        """react to a callback that was retrieved over websocket"""
        if "C" not in msg:
            logging.error("Not a callback: " + msg)
            raise Exception("Not a callback: " + msg)
        logging.debug(msg)
        msgJson = re.sub(r"(?<!\\)\"]", "", msg)
        msgJson = (
            msgJson.
                replace('a["C ', '').
                # replace('"]', '').
                replace("\\\"", "\"").
                replace("\\\\", "\\")
        )
        try:
            logging.debug(msgJson)
            msgDic = json.loads(msgJson)
            return msgDic
        except Exception as e:
            logging.error("Error parsing json: " + str(e) + "\n" + str(msgJson))
            raise e

    def makeMsgWithLetter(self, letter, in_str):
        """add letter to json object (R or E)"""
        json_str = in_str.replace("\"", "\\\"")
        return '["' + letter + ' ' + json_str + '"]'

    def fromRfc3339(self, tStr):
        return iso8601.parse_date(tStr)

    def getRfc3339(self, dat):
        return dat.isoformat('T')
