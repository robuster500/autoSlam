import struct
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import UInt32MultiArray
from std_msgs.msg import UInt16MultiArray
from std_msgs.msg import UInt8

class DataCallBack:
    GET_DEV_RUN_STATUS = 0x01
    GET_DEV_STATUS     = 0x02
    SET_DEV_RUN_STATUS = 0x03
    SET_SJ_HIGH        = 0x04
    SET_SJ_STOP        = 0x05
    SET_SLAM_RUN_MAP   = 0x06
    GET_SENSOR_DATA    = 0x07
    SET_BEGIN_SCAN_MAP = 0x09
    SET_BEGIN_GET_MAP  = 0x0A
    GET_MAP_DATA       = 0x0B
    SET_END_GET_MAP    = 0x0C
    SET_DEV_STATUS     = 0x0D
    SET_DEV_WARNING    = 0x0E
    SET_AMCL           = 0x0F

    def __init__(self):
        pass
    
    def getDevRunStatus(self,_odomDict):
        listdata = []
        temp = struct.pack('i',_odomDict["linex"])        
        listdata += list(temp)
        temp = struct.pack('i',_odomDict["liney"])        
        listdata += list(temp)
        temp = struct.pack('i',_odomDict["speedX"])        
        listdata += list(temp)
        temp = struct.pack('i',_odomDict["angle"])        
        listdata += list(temp)
        return listdata

    def setSliderAimHihg(self,pub,dataBuff):
        data = []        
        data.append(dataBuff[0])
        data.append(0x01)
        msg = UInt8MultiArray(data = data)
        pub.publish(msg)

    def setSliderStop(self,pub):
        data = []        
        data.append(1)        
        data.append(0)
        msg = UInt8MultiArray(data = data)
        pub.publish(msg)

    def setDevRunStatus(self,pub,dataBuff):
        msg = Twist() #初始化
        msg.linear.x=float(dataBuff[0])/100
        msg.angular.z=float(dataBuff[1])/100
        pub.publish(msg)

    def getSensorData(self,_sensorDict,x,y,w):
        listdata = []
        temp = struct.pack('h',_sensorDict['CO'])        
        listdata += list(temp)     
        temp = struct.pack('h',_sensorDict['SH'])        
        listdata += list(temp)     
        temp = struct.pack('h',_sensorDict['EX'])        
        listdata += list(temp)     
        temp = struct.pack('h',_sensorDict['DO'])      
        listdata += list(temp) 
        temp = struct.pack('h',_sensorDict['HIGH'])   
        listdata += list(temp)             
        temp = struct.pack('i',_odomDict["linex"])        
        listdata += list(temp)
        temp = struct.pack('i',_odomDict["liney"])        
        listdata += list(temp)
        temp = struct.pack('i',_odomDict["angle"])  
        listdata += list(temp)
        return listdata

    def setDevSatus(self,pub,dataBuff):
        data = []
        data.append(0x04)
        data.append(dataBuff[0])
        data.append(dataBuff[1])
        msg = UInt32MultiArray(data = data)
        pub.publish(msg)


    def setDevWarning(self,pub,dataBuff):      
        msg = UInt8(data = dataBuff[0])
        pub.publish(msg)

    def setRunMap(self,dataBuff):
        # self.amclPub = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=10)
        try:
            fileName = chr(dataBuff[0]) + chr(dataBuff[1]) + chr(dataBuff[2])
            rospy.set_param("slamMap","/home/szzq/rosMap/" + fileName + ".yaml")
        except:
            return False
            
    def setAmcl(self,pub):
        #amcl
        try:
            x,y,z = struct.unpack("<fff",dataBuff[3:15])
            temp = PoseWithCovarianceStamped()
            temp.position.x = float(x)
            temp.position.y = float(y)
            temp.position.z = float(z)
            pub.publish(temp)
            print(x,y,z)
        except:
            print(dataBuff,len(dataBuff))
        
