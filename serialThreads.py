# -*- coding: utf-8 -*-
"""
MOLLI [2016]
Medical Physics, Odette Cancer Centre
Sunnybrook Hospital, Toronto

SERIAL-IO
"""

import threading
import queue
import time
import logging

import serial
import serial.tools.list_ports
from PyQt5.QtCore import QThread, pyqtSignal


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
            data = readUntil(self.ser, self.eol)
            self.data_q.put(data)
        self.ser.reset_input_buffer()

    def join(self, timeout=None):
        self.alive.clear()
        threading.Thread.join(self, timeout)
        time.sleep(0.1)
        self.ser.close()


# poll for MOLLI!
class QTSerialPoller(QThread):
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
            comPort = FindSpecificCOMPort(PortCaption=self.serialName,
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


def FindSpecificCOMPort(PortCaption='MOLLI', VID='4D8', PID='F181'):
    # Scan computer for COM PORTs that match the given string name
    COM_PORT_ID = None

    for port in serial.tools.list_ports.comports():

        if PortCaption.lower() in port.description.lower():
            COM_PORT_ID = port.device
            break

        elif port.vid is not None and VID in hex(port.vid).upper():
            COM_PORT_ID = port.device
            break

        elif port.pid is not None and PID in hex(port.pid).upper():
            COM_PORT_ID = port.device
            break

    return COM_PORT_ID


def readUntil(ser, eol=b'\r'):
    leneol = len(eol)
    line = bytearray()
    while True:
        c = ser.read(1)
        if c:
            line += c
            if line[-leneol:] == eol:
                break
    return bytes(line)


def get_all_from_queue(Q):
    """
    Generator to yield one after the others all items
    currently in the queue Q, without any waiting.
    """
    try:
        while True:
            yield Q.get_nowait()
    except queue.Empty:
        raise StopIteration


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

    molli_PortID = FindSpecificCOMPort('MOLLI Surgical Probe')
    print("Molli found at: ", molli_PortID)
    data_q = 1
    ser = serial.Serial(molli_PortID)
    SIO = serialReader(data_q=data_q, com_port='COM9')
    SIO.start()
    time.sleep(2)
    SIO.join()
