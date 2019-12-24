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
import roslaunch
import _thread
from nav_msgs.msg import Odometry


uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

rosBasePath = "/home/szzq/szzq_ws/src/robuster_mr/launch/"
devName = "mr500_bringup.launch"
slamName = "mr500slamwithG4.launch"
# 启动小车标志位
devOpenFlag = False

# 启动小车SLAM标志位
slamOpenFlag = False

'''
打开rosserial_python的指令
'''
def openRosSerial():
    global uuid
    tracking_launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/szzq/autoSlam/src/rosserial_python/launch/rosserial.launch"])
    tracking_launch.start()


'''
能获取到odom，表示底层开启成功
'''
def devScanData(data):
    global devOpenFlag
    devOpenFlag = True
    pass
    
'''
打开小车模型的指令
'''
def openDevModel():
    global uuid,rosBasePath,devName    
    rospy.Subscriber("/odom", Odometry, devScanData)    
    tracking_launch = roslaunch.parent.ROSLaunchParent(uuid, [rosBasePath + devName])
    tracking_launch.start()
    # subprocess.call("roslaunch robuster_mr mr500_bringup.launch &",shell=True)
    
'''
判断小车模型是否启动
'''
def checkDevModel():
    global devOpenFlag
    return devOpenFlag

'''
打开SLAM的指令
'''
def openDevSLAM():
    global uuid,rosBasePath,slamName
    tracking_launch = roslaunch.parent.ROSLaunchParent(uuid, [rosBasePath + slamName])
    tracking_launch.start()  

'''
判断Slam是否启动
'''
def checkSlamModel():
    global slamOpenFlag
    return slamOpenFlag

#todo:
# 判断slam节点打开成功

'''
打开速度控制的指令
'''
def openDevSendGold():
    subprocess.call("rosrun nav sendgoal_node &",shell=True)

'''
设置导航点坐标指令
'''  
#/multinav
def setSlamGold(name,data):
    rospy.set_param(name,data)

'''
打开速度发布节点
'''
def openNav():
    subprocess.call("rosrun multinav nav.py &",shell=True)

