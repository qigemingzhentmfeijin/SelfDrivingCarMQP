#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan

global setupFlag
setupFlag = True


    
global leftRange
global rightRange
global exhaustedList
    
def callback(data):
    global setupFlag
    global preLidarMsg
    if (setupFlag == True):
        preLidarMsg = configScannerMsg()
        setupFlag == False
    filteredVals = LidarFilter(data.ranges)
    pubToRviz(filteredVals)

def LidarFilterSetUp(left,right,width):
    global exhaustedList
    global leftRange
    global rightRange
    exhaustedList = []
    leftRange = math.radians(left)
    rightRange = math.radians(right)
    for i in range (0, 511):
        exhaustedList.append(width/math.sin(abs(-1.570796370+i*0.00613592332229)))
        i = 1+1;
    return 0

def LidarFilter(rawdata):
    global exhaustedList
    global leftRange
    global rightRange
    #chop the array
    i = int(math.ceil((1.56466042995 - leftRange) / 0.00613592332229))    #left
    j = int(math.ceil((1.57079637051 - rightRange)/ 0.00613592332229)) -1 #right
    resultList = list(rawdata[j:511 - i])

    #
    for k in range (j,511-i):
        if resultList[k-j] <= exhaustedList[k]:
            resultList[k-j] = resultList[k-j]* abs(math.cos(-1.57079637051 + k*0.00613592332229))
        else:
            resultList[k-j] = 6.0
    return min(resultList)




def pubToRviz(dataArray):
    global pub_LidarTester
    #updatedScan = preLidarMsg
    #updatedScan.ranges = dataArray
    #print dataArray
    pub_LidarTester.publish(dataArray)
    rospy.sleep(0.05)


def configScannerMsg():
    testerPackVal = LaserScan()
    testerPackVal.header.frame_id = "laser"
    testerPackVal.angle_min = -1.57079637051
    testerPackVal.angle_max = 1.56466042995
    testerPackVal.angle_increment = 0.00613592332229
    testerPackVal.time_increment = 9.76562514552e-05
    testerPackVal.scan_time = 0.10000000149
    testerPackVal.range_min = 0.019999999553
    testerPackVal.range_max = 5.59999990463
    return testerPackVal;


def run():
    LidarFilterSetUp(60,60,0.3)
    global pub_LidarTester
    rospy.init_node('LidarPubTester', anonymous=True)
    pub_LidarTester = rospy.Publisher('racer/lidar/tester', Float32, queue_size=1)
    rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
