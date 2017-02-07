# -*- coding: utf-8 -*-
"""
SerialUtilz: serialThreads

author: Mark Semple

Different threads for dealing with Serial Objects


"""

import threading
import queue
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


class serialReader(threading.Thread):
    """ Thread to handle serial communication. When started, this goes.
        clock begins, initialization protocol is run, writer starts,
        and then reading-loop begins. at each step of the read, take all
        the content and throw it into data Queue. Will do sorting later
    """

    def __init__(self,
                 data_q,
                 ser=None,
                 com_port=None,
                 baud=9600,
                 designatedWriter=False,
                 txtString='',
                 eol_write='',
                 eol_read=''):

        super().__init__(name='SerialReaderThread')

        self.ser = serFcns.makeSerialObject(ser, com_port, baud)

        self.ser.timeout = 0.1
        # self.ser.close()
        self.eol_read = bytes(eol_read, 'utf-8')
        self.eol_write = bytes(eol_write, 'utf-8')
        self.data_q = data_q
        self.designatedWriter = designatedWriter
        self.alive = threading.Event()
        self.alive.set()
        self.command = b''
        if bool(txtString):
            self.command = bytes(txtString, 'UTF-8') + eol_write

    def run(self):
        """ Thread to handle serial communication. When started, this goes.
            clock begins, initialization protocol is run, writer starts,
            and then reading-loop begins. at each step of the read, take all
            the content and throw it into data Queue. Will do sorting later
        """
        # time.clock()
        # self.ser.open()
        # time.sleep(0.1)

        print("STARTING")
        print(self.ser)

        while self.alive.isSet():

            print("reader reading!")

            if not self.designatedWriter and bool(self.command):
                self.ser.write(self.command)

            data = serFcns.readUntil(self.ser, self.eol_read)
            self.data_q.put(data)

    def join(self, timeout=None):
        self.alive.clear()
        time.sleep(0.25)
        if self.designatedWriter:
            self.COMwriter.join(0.02)
        threading.Thread.join(self, timeout)
        # self.ser.close()


class serialWriter(threading.Thread):
    """ A thread for writing to the COM PORT.
        Requires SER or COM_PORT as valid serial device / address
        Requires txtString as command to repeat over and over
    """

    def __init__(self, txtString=None, ser=None,
                 com_port=None, baud=9600, eol_write=b'\r'):
        super().__init__(name='SerialWriterThread')

        self.ser = serFcns.makeSerialObject(ser, com_port, baud)

        if not type(txtString) == str:
            raise TypeError

        if not bool(txtString):
            # raise error
            print("raise error here..")
            pass

        self.alive = threading.Event()
        self.alive.set()
        self.command = bytes(txtString, "UTF-8") + eol_write

        print("writer command is: %s" % txtString)

    def run(self):

        while self.alive.isSet():
            # try:
            print("writer writing")
            self.ser.write(self.command)
            time.sleep(0.008)
            # except serial.SerialException:
            # pass

        print("Finished writing")
        self.ser.write(serFcns.formatWrite('TSTOP '))

    def join(self, timeout=None):
        self.alive.clear()
        threading.Thread.join(self, timeout)


class QComThread(QThread):

    def __init__(self, ser=None, com_port=None, serArgs={}, *args, **kwargs):

        super().__init__(*args, **kwargs)

        if ser is None and com_port is None:
            print("No Port!")
            return

        elif ser is None:
            self.ser = serial.Serial(port=com_port,
                                     **serArgs)

        else:
            self.ser = ser

        # self.ser.timeout = timeout
        self.ser.close()
        self.isAlive = True


class QComInitThread(QComThread):

    initDone = pyqtSignal()
    # statusMsg = pyqtSignal(str, int)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def run(self):
        print("Beginning Initialization")

        self.ser.close()
        print("serial closed")

        self.ser.open()
        print("serial opened")

        self.ser.reset_output_buffer()
        self.ser.reset_input_buffer()

        init_out = self.onInitialize()
        if init_out:
            self.initDone.emit()
        else:
            print("init failed")
            self.ser.close()
        self.ser.close()

    def onInitialize(self):
        """ Device-Specific -- Overwrite this function upon inheritance"""
        print("boop")
        pass


class QComReaderThread(QComThread):
    pass


class QAuroraThread(threading.Thread):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        pass


    def onInitialize(self, ser):
        pass


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


if __name__ == "__main__":
    pass
