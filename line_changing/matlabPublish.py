import rospy
import numpy as np
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Float64MultiArray
import time


global totalTime
global timeStamp


def runSend():
	global totalTime 
	global timeStamp 
	global pub
	timeStamp = 0.5
	rospy.init_node('sendOdom')
	totalTime = 0.0

	pub = rospy.Publisher("/racer/Odom/pos", Float64MultiArray, queue_size = 1)
	
	

	odomSub = rospy.Subscriber("/zed/odom", Odometry, updateOdom)
	rospy.spin()


def updateOdom(data):
	global pub
	Pose = data.pose.pose.position
	x = Pose.x
	y = Pose.y
	z = Pose.z
	point = Float64MultiArray()
	point.data = [x,y,z]
	pub.publish(point)
	print x, y, z
	


if __name__ == '__main__':
	try:
		runSend();
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start Odom Sender node.')
