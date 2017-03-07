#!/usr/bin/env python

from msb_classes import *


class TestClasses:
    """test some stuff from Service class"""
    def test_Service_toJson(self):
        testService = Service("u", "n", "d", [], [], "t")
        expectedJson = '{"uuid":"u","name":"n","description":"d","events":[],"functions":[],"token":"t","@class":"Service"}'
        assert testService.toJson() == expectedJson

    def test_SmartObject_toJson(self):
        testService = SmartObject("u", "n", "d", [], [], "t")
        expectedJson = '{"uuid":"u","name":"n","description":"d","events":[],"functions":[],"token":"t","@class":"SmartObject"}'
        assert testService.toJson() == expectedJson

    def test_Application_toJson(self):
        testService = Application("u", "n", "d", [], [], "t")
        expectedJson = '{"uuid":"u","name":"n","description":"d","events":[],"functions":[],"token":"t","@class":"Application"}'
        assert testService.toJson() == expectedJson

    def test_Service_wEvent(self):
        testService = Service("u", "n", "d", [
                Event("ei", "n", "d", []).toOrderedDict()
            ], [], "t")
        expectedJson = '{"uuid":"u","name":"n","description":"d","events":[{"eventId":"ei","name":"n","description":"d","dataFormat":{}}],"functions":[],"token":"t","@class":"Service"}'
        assert testService.toJson() == expectedJson

    def test_Service_wDataFormat(self):
        testService = Service("u", "n", "d", [
                Event("ei", "n", "d", [
                    DataFormat("n", "Integer")
                ]).toOrderedDict()
            ], [], "t")
        expectedJson = '{"uuid":"u","name":"n","description":"d","events":[{"eventId":"ei","name":"n","description":"d","dataFormat":{"n":{"type":"integer","format":"int32"}}}],"functions":[],"token":"t","@class":"Service"}'
        assert testService.toJson() == expectedJson

    def test_Service_wFunction(self):
        testService = Service("u", "n", "d", [], [
                Function("f", [
                    DataFormat("n", "String")
                ], "n", "d").toOrderedDict()
            ], "t")
        expectedJson = '{"uuid":"u","name":"n","description":"d","events":[],"functions":[{"functionId":"f","dataFormat":{"n":{"type":"string"}},"name":"n","description":"d"}],"token":"t","@class":"Service"}'
        assert testService.toJson() == expectedJson
