#!/usr/bin/env python2
import subprocess

from msb_classes import *


class MsbRosHelper:
    """some helper methods needed for ros"""

    def runCommand(self, command):
        """runs a command in system shell and returns the output"""
        sp = subprocess
        return sp.check_output(command)

    def getPubdTopics(self):
        """get currently published topics"""
        out = self.runCommand(["rostopic", "list"])
        l = out.split("\n")
        # print l
        l = self.cleanTopicsList(l)
        return l

    def getTopics(self, inp):
        """get topics"""
        l = inp.strip().split(',')
        # print l
        return self.cleanTopicsListList(l)

    def cleanTopicsList(self, inlist):
        """clean the list of topics"""
        outlist = []
        for entry in inlist:
            if entry is not "":
                entry = entry.strip()
                if entry[0] is "/":
                    # print "entry"
                    # print entry
                    outlist.append(entry[1:])
                    # print "outlist"
                    # print outlist
                else:
                    outlist.append(entry)
        return outlist

    def cleanTopicsListList(self, inlist):
        """clean the cascaded list of topics"""
        outlist = []
        for entry in inlist:
            if entry is not "":
                entry = entry.strip()
                if entry[0] is "/":
                    outlist.append(entry[1:].split(" "))
                else:
                    outlist.append(entry.split(" "))
        return outlist

    def getDictForMsgType(self, typein):
        """parse a ROS message type into a dict for msb registration"""
        typeinWithDash = typein.replace("/", "-")
        # print typeinWithDash
        defi = {}
        defi[typeinWithDash] = {}
        defi[typeinWithDash]["properties"] = {}
        currentDefTyp = {}
        currentDefTyp[-1] = typeinWithDash

        out = self.runCommand(["rosmsg", "show", typein])
        lines = out.split("\n")
        # print lines
        while "" in lines:
            lines.remove("")

        for line in lines:
            indent = line.count("  ")
            line = line[indent * 2:]
            (typ, nam) = line.split(" ")
            if "/" in typ:  # Definition
                typ = typ.replace("/", "-")
                defi[typ] = {}
                defi[typ]["properties"] = {}
                currentDefTyp[indent] = typ
                defi[
                    currentDefTyp[indent - 1]
                ]["properties"].update(
                    {
                        nam: {"$ref": "#/definitions/" + typ}
                    }
                )
            else:  # Atomic Property
                try:
                    msbType = DataTypes.fromRos[typ]
                except Exception, e:
                    raise TypeError(
                        "Can not relove " +
                        typ +
                        " to msb. (Maybe missing in classes file)"
                    )
                defi[
                    currentDefTyp[indent - 1]
                ]["properties"].update(DataFormat(
                    nam,
                    msbType
                ).toCol())

        defi["dataObject"] = {
            "$ref": "#/definitions/" + typeinWithDash
        }

        # print json.dumps(defi, indent=2)
        return defi
