#!/usr/bin/env python
import rospy
import time
import math
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool

global initTime
global prev_PubRunTime
global kp
global ki
global kd
global fgain
global desiredSpeed
global heartBeat
global const_power_test
global runTime
global estop
global stopFlag
global vel_period
global vel_max
prev_PubRunTime = 0
kp = 250
ki = 0.00002
kd = 0.6
fgain = 0
vel_period = 4
vel_max = 30
desireSpeed = 0
heartBeat = 0
const_power_test = 0
runTime = 30
estop = 0
stopFlag = 1

def sineSpeedOut(period, max_vel):
	velocity = max_vel/2*(math.sin(2*math.pi/period*currentRunTime()) + 1)
	return  velocity

def currentRunTime():
	global initTime
	return time.time()-initTime

def callbackSpeed(data):
	print(str(currentRunTime())+","+str(data.data))

def PIDPublisher(end):
	global kp
	global ki
	global kd
	global fgain
	global desiredSpeed
	global heartBeat
	global const_power_test
	global runTime
	global prev_PubRunTime
	
	if 0 < currentRunTime() < runTime and not end:
		heartBeat = 1
		# print("Running...")
	else:
		heartBeat = 0
	pub_pidinfo = rospy.Publisher('racer/teensy/pidinfo', Float32MultiArray, queue_size=1)
	if currentRunTime() - prev_PubRunTime >= 0.1:
		PIDVal = Float32MultiArray()
		PIDVal.data = [kp, ki, kd, fgain, desiredSpeed, heartBeat, const_power_test]
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
	

def listener():
	rospy.Subscriber('racer/teensy/rpm', Float32, callbackSpeed)
	rospy.sleep(0.1)

def run():
	global initTime
	global pub_estop
	global vel_period
	global vel_max
	global desiredSpeed
	rospy.init_node('PIDPublisher', anonymous=True)
	# print("Ready for 3 seconds...")
	rospy.sleep(0.1)	# 0.1 seconds for ready when the code is runnning
	# print("GO!")
	# listener()
	initTime = time.time()
	while not rospy.is_shutdown():
		desiredSpeed = sineSpeedOut(vel_period, vel_max);
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
