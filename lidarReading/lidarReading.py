import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from math import sqrt
import time


global referenceArray
global pub


def initValues():
	referenceArray = []
	


def lidarFilter(


def run():
	global pub

	rospy.init_node('LidarDistance')
	
	initValues()
	
	pub = rospy.Publisher("/racer/Lidar/distance", Float32, queue_size = 10)
	rawLidarSub = rospy.Subscriber("/scan", LaserScan, lidarCallback)
	rospy.spin()
	





if __name__ == '__main__':
	try:
		run()
	except rospy.ROSInterruptExceptio:
		pass
