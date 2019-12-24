#!/usr/bin/env python3
#-*-coding:utf-8-*-
import rospy
import time
import ctypes
import struct
import json
import subprocess
import signal
import os
import re
import sys
import threading
import ctypes
import struct
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import UInt8
from std_msgs.msg import UInt32MultiArray
from std_msgs.msg import UInt16MultiArray

# print(os.path.dirname(os.path.abspath(__file__)))

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)
# from slam import createMap
from slam import navigtion
from slam import createMapOpt
from Rs485 import Rs485
from optData import dataCallback

class main:
    cmdvelPub = ''
    devCtlPub = ''
    ledPub = ''
    sliderlPub = ''
    amclPub = ''
    ser = ''
    odomDict = {}
    devStatusDict = {}
    statusDict = {}
    sensorDict = {}
    sjgStatus = ''
    warningStatus = ''
    def __init__(self):    
        self.sg_dataCb = dataCallback.DataCallBack()
        self.sg_slamOpt = createMapOpt.SlamOpt()
        self.sendIamge = False
        self.mythread = threading.Thread(target = self.sendDataToPC)
        pass

    def sendDataToPC(self):
        while not rospy.is_shutdown():
            if self.sendIamge == False:
                time.sleep(0.01)
                continue
            filename = self.sg_slamOpt.fileName
            rospy.loginfo(filename)
            curFileSize = 0
            try :
                fileOpt = open("/home/szzq/rosMap/" + filename + ".zip","rb")
                fileSize = os.path.getsize("/home/szzq/rosMap/" + filename + ".zip")
            except :
                Rs485.replayRs485Data(self.ser,0xAB,[1])
                self.sendIamge = False
                continue
            listdata = []
            temp = struct.pack('i',fileSize)  
            listdata += list(temp)                  
            Rs485.replayRs485Data(self.ser,0xAB,listdata)
            time.sleep(0.05)
            while not rospy.is_shutdown():
                try:                                                
                    readFileData = fileOpt.read(1024*10)
                    curFileSize += len(readFileData)
                    # print(listdata)                    
                    Rs485.rs485Write(self.ser,readFileData)
                    if curFileSize >= fileSize:                        
                        break
                    time.sleep(0.05)
                except :
                    Rs485.replayRs485Data(self.ser,replayFunc,[2])
                    break
            fileOpt.close()
            self.sendIamge = False            
        self.mythread.join()

    def getodomData(self,odomdata):        
        linex = odomdata.pose.pose.position.x
        liney = odomdata.pose.pose.position.y
        speedX = odomdata.twist.twist.linear.x
        angle = odomdata.twist.twist.angular.z
        self.odomDict["linex"] = int(linex*100)
        self.odomDict["liney"] = int(liney*100)
        self.odomDict["speedX"] = int(speedX*100)
        self.odomDict["angle"] = int(angle*100)
        pass
    def getStatusData(self,statusData):        
        self.statusDict['warning'] = statusData.data[0]
        self.statusDict['devMode'] = statusData.data[1]
        self.statusDict['light'] = statusData.data[2]
        self.statusDict['power'] = statusData.data[4]
        pass
    def getSensorData(self,sensorData):        
        self.sensorDict['CO'] = sensorData.data[0]
        self.sensorDict['SH'] = sensorData.data[1]
        self.sensorDict['EX'] = sensorData.data[2]
        self.sensorDict['DO'] = sensorData.data[3]
        self.sensorDict['HIGH'] = sensorData.data[4]

    def ansyRev485Data(self,dataBuff):
        replayFunc = dataBuff[0] + 0xA0
        if dataBuff[0] == self.sg_dataCb.GET_DEV_RUN_STATUS:
            try:
                listdata = self.sg_dataCb.getDevRunStatus(self.odomDict)
                print(listdata)
                Rs485.replayRs485Data(self.ser,replayFunc,listdata)
            except :                          
                print("run status error")
                Rs485.replayRs485Data(self.ser,replayFunc,[2])     
            pass

        if dataBuff[0] == self.sg_dataCb.GET_DEV_STATUS:
            try:
                Rs485.replayRs485Data(self.ser,replayFunc,[self.statusDict['power'],self.statusDict['devMode'],self.statusDict['warning'],self.statusDict['light']])
            except :
                Rs485.replayRs485Data(self.ser,replayFunc,[2])
            pass
        if dataBuff[0] == self.sg_dataCb.SET_DEV_RUN_STATUS:
            self.sg_dataCb.setDevRunStatus(self.cmdvelPub,dataBuff[2])
            Rs485.replayRs485Data(self.ser,replayFunc,[0])
            pass
        if dataBuff[0] == self.sg_dataCb.SET_SJ_HIGH:
            self.sg_dataCb.setSliderAimHihg(self.sliderlPub,dataBuff[2])
            Rs485.replayRs485Data(self.ser,replayFunc,[0])
            pass
        if dataBuff[0] == self.sg_dataCb.SET_SJ_STOP:
            self.sg_dataCb.setSliderStop(self.sliderlPub)
            Rs485.replayRs485Data(self.ser,replayFunc,[0])
            pass
        if dataBuff[0] == self.sg_dataCb.SET_SLAM_RUN_MAP:            
            rpy = self.sg_dataCb.setRunMap(dataBuff[2])
            self.sg_slamOpt._createRunFlg = self.sg_slamOpt.BEGIN_RUN_MAP
            if rpy == False:
                Rs485.replayRs485Data(self.ser,replayFunc,[1])
            else:
                Rs485.replayRs485Data(self.ser,replayFunc,[0])
            pass
        if dataBuff[0] == self.sg_dataCb.GET_SENSOR_DATA:
            try:
                listdata = self.sg_dataCb.getSensorData(self.sensorDict,self.odomDict)
                print(listdata)
                Rs485.replayRs485Data(self.ser,replayFunc,listdata)
            except:                     
                Rs485.replayRs485Data(self.ser,replayFunc,[2])
            pass
        if dataBuff[0] == self.sg_dataCb.SET_BEGIN_SCAN_MAP:
            # send flag
            self.sg_slamOpt._createRunFlg = self.sg_slamOpt.CREATE_BEGIN
            g_fileName = chr(dataBuff[2][0]) + chr(dataBuff[2][1]) + chr(dataBuff[2][2])
            print(g_fileName)
            self.sg_slamOpt.fileName = g_fileName
            Rs485.replayRs485Data(self.ser,replayFunc,[0])
            pass
        if dataBuff[0] == self.sg_dataCb.SET_BEGIN_GET_MAP:
            self.sg_slamOpt._createRunFlg = self.sg_slamOpt.GET_IAMGE
            Rs485.replayRs485Data(self.ser,replayFunc,[0])
            pass
        if dataBuff[0] == self.sg_dataCb.GET_MAP_DATA:
            # open zip file and send some file
            self.sendIamge = True
            # Rs485.replayRs485Data(self.ser,replayFunc,[0])
            pass
        if dataBuff[0] == self.sg_dataCb.SET_END_GET_MAP:
            # close thread and save one image
            self.sg_slamOpt._createRunFlg = self.sg_slamOpt.CREATE_END
            Rs485.replayRs485Data(self.ser,replayFunc,[0])
            pass
        if dataBuff[0] == self.sg_dataCb.SET_DEV_STATUS:
            self.sg_dataCb.setDevSatus(self.devCtlPub,dataBuff[2])
            Rs485.replayRs485Data(self.ser,replayFunc,[0])
            pass
        if dataBuff[0] == self.sg_dataCb.SET_DEV_WARNING:
            self.sg_dataCb.setDevWarning(self.ledPub,dataBuff[2])
            Rs485.replayRs485Data(self.ser,replayFunc,[0])
            pass
        if dataBuff[0] == self.sg_dataCb.SET_AMCL:
            self.sg_dataCb.setAmcl(self.amclPub,dataBuff[2])
            Rs485.replayRs485Data(self.ser,replayFunc,[0])
            pass

    def openDevNode(self,rate):
        navigtion.openDevModel()
        start = int(time.time())    
        while not rospy.is_shutdown():
            if navigtion.checkDevModel() == True:
                return 1        
            if int(time.time()) - start > 10:
                return -1
            rate.sleep()


    def connSerial(self):
        while not rospy.is_shutdown():
            for com in range(10):                
                PortNamme = "/dev/ttyUSB" + str(com)
                try:
                    self.ser = Rs485.rs485OpenSerial(PortNamme)
                except:
                    continue                
                print(PortNamme)
                time.sleep(0.5)
                revData = Rs485.rs485Loop(self.ser)
                if len(revData) != 0:
                    return True
                Rs485.rs485CloseSerial(self.ser)
                
    def mainLoop(self):
        try:
            self.connSerial()
                # PortNamme = rospy.get_param("autoSerial")
        except :    
            print("not input serial")
            return
    # dev status
        rospy.Subscriber('odom', Odometry, self.getodomData)
        rospy.Subscriber('Dev_status', UInt8MultiArray, self.getStatusData)            
        rospy.Subscriber('GasAndSlider', UInt16MultiArray, self.getSensorData)
        self.amclPub = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=10)

    # sensor
        self.sliderlPub = rospy.Publisher("Slider", UInt8MultiArray, queue_size=1000)
        self.ledPub = rospy.Publisher("LED", UInt8, queue_size=1000)
    #control
        self.devCtlPub = rospy.Publisher("DEV_CTL", UInt32MultiArray, queue_size=1000)
        self.cmdvelPub = rospy.Publisher("cmd_vel", Twist, queue_size=1000)
        rospy.init_node('robusterAuto', anonymous=True)        
        # set send serial
        self.sg_slamOpt.serial = self.ser 
        # get sensor data for rosserial
        navigtion.openRosSerial()
        # open send data thread
        self.mythread.start()
        rate = rospy.Rate(20)   #50ms
        if self.openDevNode(rate) == -1:
            print("bringup node open false")
            return
        else:
            print("bringup node open success")
        while not rospy.is_shutdown():
            self.sg_slamOpt.createMain()
            revData = Rs485.rs485Loop(self.ser)
            if revData != []:
                # rev data
                self.ansyRev485Data(revData)
            rate.sleep()
            
    def exitFunc(self,args,argv):
        self.sg_dataCb.setDevRunStatus(self.cmdvelPub,[0,0])

        self.sg_dataCb.setSliderStop(self.sliderlPub)

        self.sg_dataCb.setDevSatus(self.devCtlPub,[0,0])

        time.sleep(0.3)
        print("exit")    
        os._exit(0)

if __name__ == '__main__':
    _devMain = main()
    signal.signal(signal.SIGINT,_devMain.exitFunc)
    signal.signal(signal.SIGTERM,_devMain.exitFunc)
 
    try:        
        _devMain.mainLoop()
        # threadSlam.join()        
    except rospy.ROSInterruptException:
        pass    










