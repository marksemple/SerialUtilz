# -*- coding: utf-8 -*-
"""
SerialUtilz: serialFuncs

author: Mark Semple

Convenience functions for working with Serial Hardware Devices

Requires PySerial
Optimized for PyQt5
"""

# import threading
import queue
# import time
# import logging
import os

# import serial
# from PyQt5.QtCore import QThread, pyqtSignal

if os.name == "posix":  # catch for Apple computers
    import serial.tools.list_ports_osx as list_ports
else:
    import serial.tools.list_ports as list_ports


def FindSpecificCOMPort(PortCaption='PortCaption', VID='VID', PID='PID'):
    """ Scan computer for COM PORTs that match given criteria
        - Device Description (sometimes works, though not reliable on its own)
        - VID number
        - PID number
        Returns the serial device address (ie. COM14, or /dev/ttyS2)
    """

    COM_PORT_ID = None

    for port in list_ports.comports():

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


def formatWrite(command, eol='\r'):
    print(command + eol)
    return bytes(command + eol, "UTF-8")


def readUntil(ser, eol=b'\r'):
    """ When communicating with serial device """

    leneol = len(eol)
    line = bytearray()
    while True:
        c = ser.read(1)
        if c:
            line += c
            if line[-leneol:] == eol:
                break
    return bytes(line)


def InsertDecimal(stringList, posFromLeft):
    """  """
    numList = []
    for string in stringList:
        L = len(string)
        output = string[0: (L - posFromLeft)] + '.' + string[-posFromLeft:]
        numList.append(float(output))
    return numList


def parseSerial(bytestring):
    text = bytestring.decode("UTF-8")
    print(text)


def parsePHSR(bytestring):
    text = bytestring.decode("UTF-8")
    print(text)
    try:
        nPorts = int(text[0:2])
    except ValueError as ve:
        print(ve)
        return 0, []
    portNames = []
    for port in range(0, nPorts):
        portNames.append(text[2 + 5 * port: 4 + 5 * port])
    return nPorts, portNames


def get_item_from_queue(Q, timeout=0.01):
    """
    Attempts to retrieve an item from the queue Q. If Q is
    empty, None is returned.
    Blocks for 'timeout' seconds in case the queue is empty,
    so don't use this method for speedy retrieval of multiple
    items (use get_all_from_queue for that).
    """
    try:
        item = Q.get(True, timeout)
    except queue.Empty:
        return None
    return item


if __name__ == "__main__":
    pass
