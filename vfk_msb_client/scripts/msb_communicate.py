#!/usr/bin/env python2
import json
import iso8601
from collections import OrderedDict
from datetime import datetime

from msb_classes import *


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
        # print('DICT:')
        # print(d)

        od_send = OrderedDict([
            ("uuid", service.uuid),
            ("eventId", event["eventId"]),
            ("priority", prio),
            ("postDate", self.getRfc3339(t)),
            ("dataObject", d)
        ])
        return json.dumps(od_send, sort_keys=False, separators=(',', ':'))

    def getDict(self, d):
        """retrieve dict of an objects parameters"""
        # print '======= get Dict'
        # print type(d)
        try:  # is object
            # print d.__slots__
            # print 'is object'
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
                # print l
                # print 'is list'
                return l
            else:  # is parameter
                # print d
                # print 'is parameter'
                return d

    def handleCallback(self, msg):
        """react to a callbkac that was retrieved over websocket"""
        if "C" not in msg:
            print "Not a callback: " + msg
            raise Exception("Not a callback: " + msg)
        msgJson = (
            msg.
            replace('a["C ', '').
            replace('"]', '').
            replace("\\\"", "\"")
        )
        try:
            msgDic = json.loads(msgJson)
            return msgDic
        except Exception, e:
            print "Error parsing json: " + str(e)
            raise e

    def makeMsgWithLetter(self, letter, in_str):
        """add letter to json object (R or E)"""
        json_str = in_str.replace("\"", "\\\"")
        return '["' + letter + ' ' + json_str + '"]'

    def fromRfc3339(self, tStr):
        return iso8601.parse_date(tStr)

    def getRfc3339(self, dat):
        return dat.isoformat('T')
