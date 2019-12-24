#!/usr/bin/env python3
#-*-coding:utf-8-*-

import time
import ctypes
import struct
import os
import re
import sys
import serial

class rs485Info:
    _curStatus = 0
    _funcode = 0
    _dataLen = 0
    _dataCount = 0
    _data = []
    ser = 0
    def __init__(self,ser):
        self.ser = ser
        self._dataCount = 0
        pass

rs485Buff = []

def rs485OpenSerial(serialport):
    global rs485Buff
    ser = serial.Serial(port = serialport,baudrate = 115200,timeout = 0.002)
    if ser.isOpen() == True:
        info = rs485Info(ser)
        rs485Buff.append(info)
        return ser
    else:
        return False
    
def rs485CloseSerial(ser):
    global rs485Buff
    if type(ser) == serial.serialposix.Serial:
        ser.close()
        index = getCurInfo(ser)
        rs485Buff[index].ser = 0

def replayRs485Data(ser,funcode,data):
    repBuff = []
    crc = 0
    repBuff.append(0xAA)
    repBuff.append(funcode)
    crc += funcode
    repBuff.append(len(data))
    crc += len(data)
    if len(data) == 0:
        repBuff.append(0)
    else:
        for dd in data:
            repBuff.append(dd)
            crc += dd
            crc %= 256
    repBuff.append(crc.to_bytes(1,'little')[0])
    rs485Write(ser,repBuff)

def rs485Write(ser,data):
    ser.write(data)

def rs485Read(ser,len):
    return ser.read(len)

def getCurInfo(ser):
    global rs485Buff
    for i in range(rs485Buff.__len__()):
        if ser == rs485Buff[i].ser:
            return i

def getCrcData(databuff):
    crc = 0
    for i in range(databuff._dataLen):
        crc += databuff._data[i]
    crc += databuff._dataLen
    crc += databuff._funcode
    crc %= 256
    return crc.to_bytes(1,'little')[0]

def rs485Loop(ser):
    global rs485Buff
    rpy = []
    index = getCurInfo(ser)
    serdata = rs485Read(ser,256)

    # serdata = [0xAA,0x03,0x02,0x32,0x00,0x37]
    revLen = len(serdata)    
    if revLen != 0:
        print(serdata,revLen)
    for dataIndex in range(revLen):
        data = serdata[dataIndex]
        #head
        if rs485Buff[index]._curStatus == 0:
            # pc set data
            if data == 0xAA:
                rs485Buff[index]._curStatus = 1
                continue
        # funcode
        if rs485Buff[index]._curStatus == 1:
            rs485Buff[index]._funcode = data
            rs485Buff[index]._curStatus = 2
            continue
        # data len
        if rs485Buff[index]._curStatus == 2:
            rs485Buff[index]._dataLen = data
            rs485Buff[index]._curStatus = 3
            continue
        # data
        if rs485Buff[index]._curStatus == 3:
            if rs485Buff[index]._dataCount == rs485Buff[index]._dataLen:
                rs485Buff[index]._curStatus = 4
                rs485Buff[index]._dataCount = 0
            else:
                rs485Buff[index]._data.append(data)
                rs485Buff[index]._dataCount += 1                  
                continue
        # crc
        if rs485Buff[index]._curStatus == 4:            
            crc = int(getCrcData(rs485Buff[index]))            
            if crc == data:
                # had rev success
                # print(rs485Buff[index]._data)
                rpy.append(rs485Buff[index]._funcode)
                rpy.append(rs485Buff[index]._dataLen)
                rpy.append(rs485Buff[index]._data)       
            rs485Buff[index]._curStatus = 0
            rs485Buff[index]._data = []
            rs485Buff[index]._dataCount = 0
            continue
    return rpy






