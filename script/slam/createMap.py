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


uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

rosLidarPath = "/home/szzq/szzq_ws/src/ydlidar_ros/launch/lidar.launch"
rosCartoPath = "/home/szzq/catkin_ws/src/cartographer_ros/cartographer_ros/launch/lidar_2d.launch"
# 雷达启动状态标志位
lidarOpenFlag = False
# 建图启动状态标志位
cartoOpenFlag = False
'''
获取激光雷达数据成功，代表激光雷达启动了
'''
def lidarScanData(data):
    global lidarOpenFlag
    lidarOpenFlag = True    
    pass
'''
能获取到图片数据了，表示建图程序启动成功
'''
def cartoScanData(data):
    global cartoOpenFlag
    cartoOpenFlag = True
    pass
    
lidar_launch = ''    
'''
关闭激光雷达的指令
'''
def closeLidar():
    global lidar_launch,lidarOpenFlag
    lidar_launch.shutdown()
    lidarOpenFlag = False
'''
打开激光雷达的指令
'''
def openLidar():
    global uuid,rosLidarPath,lidar_launch
    rospy.Subscriber("/scan", LaserScan, lidarScanData)
    lidar_launch = roslaunch.parent.ROSLaunchParent(uuid, [rosLidarPath])
    lidar_launch.start()    
    # subprocess.call("roslaunch ydlidar_ros lidar.launch &",shell=True)
    
'''
判断激光雷达是否启动
'''
def checkLidar():
    global lidarOpenFlag
    return lidarOpenFlag
'''
判断建图是否启动
'''
def checkcartoGraph():
    global cartoOpenFlag
    return cartoOpenFlag

Carto_launch = ''
'''
关闭建图脚本的指令
'''
def closeCartographer():
    global Carto_launch,cartoOpenFlag
    Carto_launch.shutdown()
    cartoOpenFlag = False

'''
打开建图脚本的指令
'''
def openCartographer():
    global uuid,rosCartoPath,Carto_launch
    rospy.Subscriber("/map", OccupancyGrid, cartoScanData)
    Carto_launch = roslaunch.parent.ROSLaunchParent(uuid, [rosCartoPath])
    Carto_launch.start()    
    # subprocess.call("roslaunch cartographer_ros lidar_2d.launch &",shell=True)

'''
保存图片，并且打包
'''
def saveMapToFile(name):
    if not os.path.exists("/home/szzq/rosMap"):
        try:
            os.makedirs("/home/szzq/rosMap")
        except:
            print("mkdir file error .the file had exists or filepath error")
        
    cmd = "rosrun map_server map_saver -f ~/rosMap/" +  name + " &"
    subprocess.call(cmd,shell=True)
    time.sleep(2)

    zipcmd = "cd ~/rosMap ; zip " + name  + ".zip " + name + ".*" 
    print(zipcmd)
    subprocess.call(zipcmd,shell=True)














