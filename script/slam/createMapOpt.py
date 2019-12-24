#!/usr/bin/env python3
#-*-coding:utf-8-*-

import time
import rospy
import ctypes
from std_msgs.msg import UInt32MultiArray
import struct
import json
import subprocess
import signal
import os
import re
import sys
import roslaunch
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid





BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)

from Rs485 import Rs485
from slam import createMap
from slam import navigtion


class SlamOpt:
    # 创建地图使用的全局变量
    CREATE_BEGIN = 1
    GET_IAMGE = 2
    CREATE_END = 3
    BEGIN_RUN_MAP = 4
    _createRunFlg = 0
    _createStartFlg = 0
    fileName = '111'
    serial = ''    
    def __init__(self):        
        pass

    '''
    开启雷达和cartography
    '''
    def beginScanMap(self): 
        print("this will open lidar...\r\n")
        createMap.openLidar()
        print("this will open Cartographer...\r\n")
        createMap.openCartographer()

    '''
    检查建图程序是否正常启动
    '''
    def checkLidarStatus(self):
        if createMap.checkcartoGraph() == True and createMap.checkLidar() == True:   
            return True
        return False
    '''
    获取图片，
    '''
    def getCartoMap(self):
        createMap.saveMapToFile(self.fileName)

    '''
    创建画图的主函数，
    '''
    def createMain(self):
        # 扫描打开状态
        self.createImage()
        # 检测lidar 和cartographer打开状态
        if self.checkLidarStatus() == True and self._createStartFlg == 0:
            print("Cartographer open success\r\n")        
            print("lidar had open success\r\n") 
            self._createStartFlg = 1
            Rs485.replayRs485Data(self.serial,0xA9,[4])

    def createImage(self):
        if self._createRunFlg == self.CREATE_BEGIN:
            self._createRunFlg = 0
            self.beginScanMap()
            self._createStartFlg = 0
        if self._createRunFlg == self.GET_IAMGE:
            self._createRunFlg = 0
            self.getCartoMap()

        if self._createRunFlg == self.CREATE_END:
            self._createRunFlg = 0
            self.getCartoMap()
            createMap.closeLidar()
            createMap.closeCartographer()       
            self._createStartFlg = 0
        if self._createRunFlg == self.BEGIN_RUN_MAP:
            self._createRunFlg = 0
            navigtion.openDevSLAM()













