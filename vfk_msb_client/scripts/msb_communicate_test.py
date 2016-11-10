#!/usr/bin/env python

from msb_communicate import *

# import datetime


class TestMsbCommunicate:
    """test communication"""
    # def test_eventToJson(self):
    #   mc = MsbCommunicate()
    #   df = DataFormat("Start", "Date")
    #   e = Event("START", "Start time", "The time of Start", df)
    # s = Service(
    #     "testclient_uuid",
    #     "testclient_name",
    #     "testclient_desc",
    #     [e.toOrderedDict()], [], "testclient_token")
    #   data = mc.getRfc3339(datetime.datetime.now())
    #   expectedJson = '{"uuid":"testclient_uuid","eventId":"START","priority":2,"post_date":"' + mc.getRfc3339(datetime.datetime.now()+datetime.timedelta(microseconds=20)) + '","exchangeData":{"dataObject":{"Start":"' + data + '"}}}'
    #   assert expectedJson == mc.eventToJson(s, e, data, 2)

    # def test_getRegisterMessage(self):
    #   testService = Service("u", "n", "d", [], [], "t")
    #   expectedJson = '["R {\"uuid\":\"u\",\"name\":\"n\",\"description\":\"d\",\"events\":[],\"functions\":[],\"token\":\"t\",\"@class\":\"Service\"}"]'
    #   mc = MsbCommunicate()
    #   assert expectedJson == mc.getRegisterMessage(testService)
