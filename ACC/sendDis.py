import rospy
import numpy as np 
import math 
from scipy.constants import g as grav
from math import sqrt
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from cvxopt import matrix
from cvxopt.blas import dot
from cvxopt.solvers import qp
import cvxopt
import time


global totalTime



def runSend():
	global totalTime
	rospy.init_node('sendDistance')
	totalTime = 0.0
	
	pub = rospy.Publisher("/racer/zed/distance", Float32MultiArray, queue_size = 1)

	while(True):
		rospy.sleep(0.1)
		if((time.time()-0.5)>=totalTime):
			totalTime = time.time()
			result = Float32MultiArray()
			result.data = [0]     
			result.data[0] = 15.0
			pub.publish(result)
			print "Sending"

			rospy.sleep(0.2)

if __name__ == '__main__':

	
	try:
		runSend();
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start Sender node.')


