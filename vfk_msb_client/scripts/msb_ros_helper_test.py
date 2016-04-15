#!/usr/bin/env python
import json

from msb_ros_helper import *

class TestMsbRosHelper:
    """test the helper methods"""

    def test_getTopics(self):
        self.mrh = MsbRosHelper()
        expected = ['a', 'b', 'c']
        assert expected == self.mrh.getTopics("a,b,c")

    def test_getTopics1(self):
        self.mrh = MsbRosHelper()
        expected = ['a', 'b', 'c']
        assert expected == self.mrh.getTopics("a, b , c")

    def test_getTopics2(self):
        self.mrh = MsbRosHelper()
        expected = ['a']
        assert expected == self.mrh.getTopics(" a ")

    def test_getTopics3(self):
        self.mrh = MsbRosHelper()
        expected = ['a']
        assert expected == self.mrh.getTopics("a")

    def test_getTopicsStar1(self):
        self.mrh = MsbRosHelper()
        expected = ['*']
        assert expected == self.mrh.getTopics(" * ")

    def test_getTopicsStar2(self):
        self.mrh = MsbRosHelper()
        expected = ['*']
        assert expected == self.mrh.getTopics("*")

    def test_runCommand(self):
        self.mrh = MsbRosHelper()
        expected = "Hello World!\n"
        assert expected == self.mrh.runCommand(["echo", "Hello World!"])

    def test_getJsonForMsgType(self):
        self.mrh = MsbRosHelper()
        out = json.dumps(
            self.mrh.getDictForMsgType("geometry_msgs/PoseStamped")
        )
        out = json.loads(out)
        expected = json.loads(
            '{"message": {"$ref": "#/definitions/geometry_msgs-PoseStamped"},"geometry_msgs-PoseStamped": {"properties": {"header": {"$ref": "#/definitions/std_msgs-Header"},"pose": {"$ref": "#/definitions/geometry_msgs-Pose"}}},"std_msgs-Header": {"properties": {"seq": {"type": "integer","format": "int32"},"stamp": {"type": "string"},"frame_id": {"type": "string"}}},"geometry_msgs-Pose": {"properties": {"position": {"$ref": "#/definitions/geometry_msgs-Point"},"orientation": {"$ref": "#/definitions/geometry_msgs-Quaternion"}}},"geometry_msgs-Point": {"properties": {"x": {"type": "number","format": "float"},"y": {"type": "number","format": "float"},"z": {"type": "number","format": "float"}}},"geometry_msgs-Quaternion": {"properties": {"x": {"type": "number","format": "float"},"y": {"type": "number","format": "float"},"z": {"type": "number","format": "float"},"w": {"type": "number","format": "float"}}}}'
        )
        assert expected == out
