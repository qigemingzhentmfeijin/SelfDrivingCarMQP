#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool

global initTime
global prev_PubRunTime
global high_power
global high_duration
global low_power
global low_duration
global estop
global stopFlag
prev_PubRunTime = 0

###
high_power = 20
high_duration= 90
low_power = 15
low_duration = 20
###

estop = 0
stopFlag = 1;

def currentRunTime():
	global initTime
	return time.time()-initTime

def callbackSpeed(data):
	print(str(currentRunTime())+","+str(data.data))

def lowspeedPub(end):
	global high_power
	global high_duration
	global low_power
	global low_duration
	global prev_PubRunTime
	
	pub_lowspeedinfo = rospy.Publisher('racer/teensy/lowspeedinfo', Float32MultiArray, queue_size=1)
	if currentRunTime() - prev_PubRunTime >= 0.05:
		lowspeedVal = Float32MultiArray()
		lowspeedVal.data = [high_power, high_duration, low_power, low_duration]
		pub_lowspeedinfo.publish(lowspeedVal)
		prev_PubRunTime = currentRunTime()
	

def emergencyStop():
	global stopFlag
	global pub_estop
	if (stopFlag == 1):
		pub_estop = rospy.Publisher('racer/teensy/estop',Bool, queue_size = 1)
		lowspeedPub(True)
		pub_estop.publish(True)
		stopFlag = 0
	

def listener():
	rospy.Subscriber('racer/teensy/rpm', Float32, callbackSpeed)
	rospy.sleep(0.1)

def run():
	global initTime
	global pub_estop
	rospy.init_node('lowspeedPub', anonymous=True)
	# print("Ready for 3 seconds...")
	rospy.sleep(3)	# 3 seconds for ready when the code is runnning
	# print("GO!")
	initTime = time.time()
	while not rospy.is_shutdown():
		lowspeedPub(False)
		listener()
		#rospy.spin()
		pub_estop = rospy.Publisher('racer/teensy/estop',Bool, queue_size = 1)
		pub_estop.publish(False)
		rospy.on_shutdown(emergencyStop)

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
		pass
