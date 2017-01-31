# -*- coding: utf-8 -*-
"""
SerialUtilz: serialThreads

author: Mark Semple

Different threads for dealing with Serial Objects


"""

import threading
# import queue
import time
import logging

import serial
import serial.tools.list_ports
from PyQt5.QtCore import QThread, pyqtSignal

if __name__ == "__main__":
    import serialFuncs as serFcns
else:
    import SerialUtilz.serialFuncs as serFcns


class QtSerialPoller(QThread):
    """ """

    foundCOMPort = pyqtSignal(str)

    def __init__(self, maxCount=1, serialName=None, VID=None, PID=None):
        super().__init__()

        if serialName is None and VID is None and PID is None:
            print("No Serial Info Given")
            logging.error("Attemping search for serial, \
                           but not sure what to look for")
            return

        self.serialName = serialName
        self.VID = VID
        self.PID = PID
        self.isAlive = True
        self.maxCount = maxCount

    def run(self):
        """ Thread to Auto-Discover Serial Device, and then Auto-Connect """
        comPort = None
        count = 0
        maxCount = self.maxCount

        while comPort is None and count <= int(maxCount):

            count += 1
            logging.info("Polling for %s" % self.serialName)
            comPort = serFcns.FindSpecificCOMPort(PortCaption=self.serialName,
                                                  VID=self.VID,
                                                  PID=self.PID)
            time.sleep(0.5)

        if type(comPort) is list:
            comPort = comPort[0]

        if type(comPort) is bytes:
            comPort = comPort.decode('utf-8')

        if comPort is None:
            logging.error("No COM Port found")
            return
        else:
            logging.info("MOLLI Found at: %s" % comPort)
            self.foundCOMPort.emit(comPort)


class serialWriter(threading.Thread):
    """ A thread for writing to the COM PORT.

        Requires SER as valid serial device
        Requires txtString as command to repeat over and over

    """

    def __init__(self, userSettings={}, txtString=None, ser=None):
        super(serialWriter, self).__init__()

        if not type(txtString) == str:
            raise TypeError
        if not bool(txtString):
            # raise error
            print("raise error here..")
            pass

        self.ser = ser
        self.userSettings = userSettings
        self.port_handles = None
        self.alive = threading.Event()
        self.alive.set()
        # self.update_freq = 50
        # string = "TX 0801"
        self.command = bytes("%s\r" % (txtString), "UTF-8")

    def run(self):

        self.ser.write(formatWrite("TSTART "))

        while self.alive.isSet():
            try:
                self.ser.write(self.command)
                time.sleep(0.008)
            except serial.serialException:
                pass
        print("Finished writing")

        self.ser.write(serFcns.formatWrite("TSTOP "))
        self.ser.read(self.ser.inWaiting())
        # print("setting 9600Baud")
        # self.ser.write(formatWrite("COMM 00000"))

        self.ser.reset_output_buffer()
        self.ser.reset_input_buffer()

    def join(self, timeout=None):
        # self.stop()
        self.alive.clear()
        threading.Thread.join(self, timeout)


class serialReader(threading.Thread):
    """ Thread to handle serial communication. When started, this goes.
        clock begins, initialization protocol is run, writer starts,
        and then reading-loop begins. at each step of the read, take all
        the content and throw it into data Queue. Will do sorting later
    """

    def __init__(self, data_q, com_port, eol=b'!6!'):
        super().__init__()
        self.ser = serial.Serial(com_port)
        self.ser.timeout = 0.1
        self.ser.close()
        self.eol = eol
        self.data_q = data_q
        self.alive = threading.Event()
        self.alive.set()

    def run(self):
        # time.clock()
        self.ser.open()
        time.sleep(0.05)
        while self.alive.isSet():
            data = serFcns.readUntil(self.ser, self.eol)
            self.data_q.put(data)
        self.ser.reset_input_buffer()

    def join(self, timeout=None):
        self.alive.clear()
        threading.Thread.join(self, timeout)
        time.sleep(0.1)
        self.ser.close()


class initComThread(QThread):

    initAurora = pyqtSignal(dict)
    statusMsg = pyqtSignal(str, int)

    def __init__(self, userSettings={}, auroraInitialized=False, ser=None):
        QThread.__init__(self)
        self.userSettings = userSettings
        self.auroraInitialized = auroraInitialized
        self.isAlive = True
        self.ser = ser

    def run(self, comPort=None):
        """ Thread to Auto-Discover NDI Aurora,
        then Auto-Connect"""

        if not self.auroraInitialized:
            print("initializing")
            if bool(self.ser):
                self.ser.close()
            print("serial closed")
            self.ser.open()
            print("serial opened")
            self.ser.flushOutput()
            print("out buffer reset")
            self.ser.flushInput()
            print("in buffer reset")
            portDict = self.onInitialize(self.ser)
            if portDict:
                self.initAurora.emit(portDict)

        else:
            print("Already initialized")

    def onInitialize(self, ser):
        """ COMMANDS TO NDI AURORA TO INITIALIZE SYSTEM """
        ser.timeout = 1
        portDict = {}

        # print("Reset")
        ser.sendBreak()  # reset aurora
        # parseINIT(readTilEnd(ser))
        rep = ser.readline()
        if rep == b'':
            self.statusMsg.emit("no connection made", 2000)
            return {}

        self.statusMsg.emit("INIT", 0)
        ser.write(serFcns.formatWrite("INIT "))
        # parseINIT(readTilEnd(ser))
        self.statusMsg.emit(ser.readline().decode('UTF-8'), 0)

        self.statusMsg.emit("Change BAUD", 0)
        ser.write(serFcns.formatWrite("COMM 50001"))  # 115200, handshaking
        # ser.write(formatWrite("COMM 00001"))  # 9600, handshaking

        # parseINIT(readTilEnd(ser))
        self.statusMsg.emit(ser.readline().decode('UTF-8'), 0)

        ser.baudrate = 115200  # don't bother making flexible.
        ser.xonxoff = True

        # ser.reset_output_buffer()
        # ser.reset_input_buffer()

        ser.timeout = 3
        self.statusMsg.emit("Assign Port Handles", 0)
        ser.write(serFcns.formatWrite("PHSR "))
        nPorts, portNames = serFcns.parsePHSR(ser.readline())
        self.statusMsg.emit("Found %d port handles" % nPorts, 0)

        if nPorts < 1:
            self.statusMsg.emit("Unable to find any port handles", 0)
            ser.close()
            return {}

        ser.timeout = 1
        for ind, port in enumerate(portNames):

            # portDict[port] = PortHandle(portID=port, name=portNames[ind])

            self.statusMsg.emit("Init Port Handle %s" % port, 0)
            ser.write(serFcns.formatWrite("PINIT %s" % port))
            # self.statusMsg.emit(serFcns.parsePINIT(readTilEnd(ser)), 0)

            # self.statusMsg.emit("Enable Port Handle %s" % port, 0)
            ser.write(serFcns.formatWrite("PENA %sD" % port))
            # self.statusMsg.emit(parsePENA(readTilEnd(ser)), 0)
            # print(ser.readline())

        ser.timeout = 2
        self.statusMsg.emit("Starting Aurora", 0)
        ser.write(serFcns.formatWrite("TSTART "))
        # parseTSTART(readTilEnd(ser))
        self.statusMsg.emit(ser.readline().decode('UTF-8'), 0)
        self.statusMsg.emit("Ready to track", 0)
        ser.timeout = 0.1

        return portDict


class LiveDataFeed(object):
    """ A simple "live data feed" abstraction that allows a reader
        to read the most recent data and find out whether it was
        updated since the last read.

        Interface to data writer:
        add_data(data):
            Add new data to the feed.
        Interface to reader:
        read_data():
            Returns the most recent data.
        has_new_data:
            A boolean attribute telling the reader whether the
            data was updated since the last read.
    """

    def __init__(self):
        self.cur_data = None
        self.has_new_data = False

    def add_data(self, data):
        self.cur_data = data
        self.has_new_data = True

    def read_data(self):
        self.has_new_data = False
        return self.cur_data


if __name__ == "__main__":
    pass
