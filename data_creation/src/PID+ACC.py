#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool

global initTime
global prev_PubRunTime
global kp
global ki
global kd
global desiredSpeed
global heartBeat
global const_power_test
global runTime
global estop
global stopFlag
prev_PubRunTime = 0
kp = 3
ki = 0.001
kd = 0.5
desiredSpeed = 0
heartBeat = 0
const_power_test = 0
runTime = 1000
estop = 0
stopFlag = 1;

def currentRunTime():
	global initTime
	return time.time()-initTime

def callbackSpeed(data):
	print(str(currentRunTime())+","+str(data.data))

def PIDPublisher(end):
	global kp
	global ki
	global kd
	global desiredSpeed
	global heartBeat
	global const_power_test
	global runTime
	global prev_PubRunTime
	global pub_pidindo
	
	if 0 < currentRunTime() < runTime and not end:
		heartBeat = 1
		# print("Running...")
	else:
		heartBeat = 0
	pub_pidinfo = rospy.Publisher('racer/teensy/pidinfo', Float32MultiArray, queue_size=1)
	if currentRunTime() - prev_PubRunTime >= 0.1:
		PIDVal = Float32MultiArray()
		PIDVal.data = [kp, ki, kd, desiredSpeed, heartBeat, const_power_test]
		pub_pidinfo.publish(PIDVal)
		prev_PubRunTime = currentRunTime()
	

def emergencyStop():
	global stopFlag
	global pub_estop
	global runTime
	if (stopFlag == 1 and currentRunTime() < runTime):
		pub_estop = rospy.Publisher('racer/teensy/estop',Bool, queue_size = 1)
		PIDPublisher(True)
		pub_estop.publish(True)
		stopFlag = 0

def getNextSpeed():
	rospy.Subscriber('racer/ACC/nextSpeed', Float32, callbackNextSpeed)

def callbackNextSpeed(data):
	global desiredSpeed
	desiredSpeed = data.data / 0.1109

def listener():
	rospy.Subscriber('racer/teensy/rpm', Float32, callbackSpeed)
	rospy.sleep(0.1)

def run():
	global initTime
	global pub_estop
	rospy.init_node('PIDPublisher', anonymous=True)
	# print("Ready for 3 seconds...")
	rospy.sleep(0.1)	# 0.1 seconds for ready when the code is runnning
	# print("GO!")
	initTime = time.time()
	while not rospy.is_shutdown():
		getNextSpeed()
		listener()
		PIDPublisher(False)
		#rospy.spin()
		pub_estop = rospy.Publisher('racer/teensy/estop',Bool, queue_size = 1)
		pub_estop.publish(False)
		rospy.on_shutdown(emergencyStop)

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
	pass
