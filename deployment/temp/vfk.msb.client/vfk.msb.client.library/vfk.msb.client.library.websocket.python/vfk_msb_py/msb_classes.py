#!/usr/bin/env python2
import json
from collections import OrderedDict

# SERVICES ------------------------------------------------------------


class Service:
    """a Service definition"""

    def __init__(self, uuid, name, description, events, functions, token):
        self.uuid = uuid
        self.name = name
        self.description = description
        self.events = events
        self.functions = functions
        self.token = token

    def toJson(self):
        od = OrderedDict([  # need this to have fixed order
            ("uuid", self.uuid),
            ("name", self.name),
            ("description", self.description),
            ("events", [e.toOrderedDict() for e in self.events]),
            ("functions", [f.toOrderedDict() for f in self.functions]),
            ("token", self.token),
            ("@class", self.__class__.__name__)
        ])
        return json.dumps(od, sort_keys=True, separators=(',', ':'))


class Application(Service):
    """an Application definition"""

    def __init__(self, *args, **kwargs):
        Service.__init__(self, *args, **kwargs)


class SmartObject(Service):
    """a SmartObject definition"""

    def __init__(self, *args, **kwargs):
        Service.__init__(self, *args, **kwargs)


# OTHERS ------------------------------------------------------------

class Function():
    """a Function to be included in Services"""

    def __init__(self, functionId, name, description, dataFormat={}, callback=False):
        self.functionId = functionId
        self.name = name.lower()
        self.description = description
        if (dataFormat.__class__ == DataFormat or
            dataFormat.__class__ == ComplexDataFormat):
            self.dataFormat = dataFormat.toCol()
        elif (dataFormat.__class__ == dict or
              dataFormat.__class__ == OrderedDict):
            self.dataFormat = dataFormat
        else:
            raise Exception("Function needs a dataFormat or dict")
        self.fixslash()
        self.callback = callback

    def toOrderedDict(self):
        od = OrderedDict([
            ("functionId", self.functionId),
            ("dataFormat", self.dataFormat),
            ("name", self.name),
            ("description", self.description)
        ])
        return od

    def fixslash(self):  #  hack solving: https://cb.ipa.fraunhofer.de/cb/issue/85777 #85777
        self.functionId = '/'+self.functionId.strip('/')


class Event():
    """an Event to be included in Services"""

    def __init__(self, eventId, name, description, dataFormat={}):
        self.eventId = eventId
        self.name = name
        self.description = description
        if (dataFormat.__class__ == DataFormat or
            dataFormat.__class__ == ComplexDataFormat):
            self.dataFormat = dataFormat.toCol()
        elif (dataFormat.__class__ == dict or
              dataFormat.__class__ == OrderedDict):
            self.dataFormat = dataFormat
        else:
            raise Exception("Event needs a dataFormat or dict, is: " + str(dataFormat.__class__))

    def toOrderedDict(self):
        od = OrderedDict([
            ("eventId", self.eventId),
            ("name", self.name),
            ("description", self.description),
            ("dataFormat", self.dataFormat)
        ])
        return od

class DataFormat():
    """a DataFormat to be returned by Functions or Events"""

    def __init__(self, name="dataObject", doc_type="",  is_array=False):
        self.name = name
        self.doc_type = doc_type
        self.is_array = is_array
        # TODO: create by actual python datatype e.g. datetime

    def getName(self):
        return self.name

    def toCol(self):
        if self.doc_type in DataTypes.doc.keys():
            typeAndFormat = DataTypes.doc[self.doc_type]
            if self.is_array:
                return {
                    self.name:
                        OrderedDict([
                            ("type", "array"),
                            ("items", DataFormat("REPLACE", self.doc_type, False).toCol()["REPLACE"])
                        ])
                }
            elif typeAndFormat.__len__() is 2:
                return {
                    self.name:
                        OrderedDict([
                            ("type", typeAndFormat[0]),
                            ("format", typeAndFormat[1])
                        ])
                }
            elif typeAndFormat.__len__() is 1:
                return {
                    self.name:
                        OrderedDict([
                            ("type", typeAndFormat[0])
                        ])
                }
            else:
                raise TypeError(
                    self.doc_type +
                    " is not a well documented type, typeAndFormat is a " +
                    typeAndFormat)
        else:
            raise TypeError(str(self.doc_type) + " is not a valid type")

class ComplexDataFormat(DataFormat):

    def __init__(self, properties, name="dataObject"):
        self.name = name
        self.properties = properties

    def toCol(self):
        d = {}
        d[self.name] = {}
        d[self.name]["$ref"] = "#/definitions/def_" + self.name
        d["def_"+self.name] = {}
        d["def_"+self.name]["properties"] = {}
        for p in self.properties:
            d["def_"+self.name]["properties"].update(p.toCol())
        return d

# HELPERS ------------------------------------------------------------


class DataTypes:
    """a data types to be used by Functions or Events
    according to https://cb.ipa.fraunhofer.de/cb/displayDocument
    /websocket+client+library+v0.0.3.docx?doc_id=242785"""

    doc = {
        "Integer": ("integer", "int32"),
        # "Long": ("integer", "int64"),
        "Float": ("number", "float"),
        "String": ("string",),
        # "Short": ("string"),
        # "Byte": ("string", "byte"),
        "Boolean": ("boolean",)
        # "Date": ("string", "date-time")
    }
    toPy = {
        "Integer": int,
        # "Long": long,
        "Float": float,
        "String": str,
        # "Short": str, #?!?
        # "Byte": byte,
        "Boolean": bool
        # "Date": datetime
    }
    fromRos = {
        "bool": "Boolean",

        "int8": "Integer",
        "uint8": "Integer",
        "int16": "Integer",
        "uint16": "Integer",
        "int32": "Integer",
        "uint32": "Integer",
        "int64": "Integer",
        "uint64": "Integer",

        "float32": "Float",
        "float64": "Float",

        "string": "String",
        "time": "String",
        "duration": "String"
    }
