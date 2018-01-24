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


global m			#car mass
global g 			#9.81
global se 
global F0 
global F1 
global F2 
global gamma
global ca 
global cd 
global psc 			#soft constraint
global x2 			#car current speed
global v0 			#leading car speed
global vd 			#desire speed
global D_old 		#old distance to front barrier
global D 			#current distance to front barrier
global softness 	#softness factor
global initD 		#initial distance to front
global dt    		#time interval
global pub 
global preV
global preTime		#used for calculate time interval

def simuSpeed(data):
	global x2
	global rps2mps
	global lowSpeed
	x2 = data*rps2mps

def updateSpeed(data):

	'''
	speed call back funciton
	'''
	global x2
	global rps2mps
	global lowSpeed
	x2 = data.data * rps2mps
	#print "x2", x2, data.data

	if(x2>1.199 and lowSpeed == True):			#high speed
		lowSpeed = False
		switchMode(lowSpeed)
	elif(x2<=1.199 and lowSpeed == False):		#low Speed		
		lowSpeed = True
		switchMode(lowSpeed)

#*************************************************************

def updateDistance(data):

	'''
	distance call back function
	'''
	#print "updateDis"
	#print "running ACC"
	global v0
	global D_old
	global D 
	global pub
	global preV
	global preTime
	global dt

	D_old = D						#distance update
	D = data.data
	v0 = distance2speed(D_old, D)
	
	currentTime = time.time()
	dt = currentTime-preTime		#find time interval
	if(dt<0.05):
		return 0
		pass
	preTime = currentTime
	print ""	
	print "Frame Frequency:", 1/dt


	if(D<=1.0):
		pub.publish(upDateQueue(0.0))#=======================
		print "front car less than 1.0m BREAK!"
	else:
		ACC_cal()

#*************************************************************
#*************************************************************
#*************************************************************

def switchMode(speed):
	if (speed):			#low speed

		F0 = 0
		F1 = 0
		F2 = 36
		print "switch to low speed mode========="
	else:				#high speed
		F0 = 5.805
		F1 = -13.914
		F2 = 44.3
		print "switch to high speed mode@@@@@@@@@"

#*************************************************************

def upDateQueue(now):
	global preV
	print "now", now
	if(now == 0.0):
		preV[0] = 0.0
		preV[1] = 0.0
		preV[2] = 0.0
		preV[3] = 0.0
		print "Speed:", 0.0
		return 0.0
	

	if(now<0.3 and now>0.0):
		#preV.append(now)
		preV.append(1.6*dt)
		preV.pop(0)
	else:
		preV.append(now)
		preV.pop(0)
	print preV
	
	result = 0.4*preV[3]+0.3*preV[2]+0.2*preV[1]+0.1*preV[0]
	
	print "Buffered Speed:", result

	return result
	#return now

#*************************************************************

def distance2speed(D1, D2):
	'''
	D2 is the new distance input

	D1 is the old distance input
	Output is the speed in dt time interval
	'''
	
	relativeSpeed = (D2-D1)/dt
	v0 = relativeSpeed + x2
	return v0
#*************************************************************

def ms2rpm(v):
	return v/rps2mps

#*************************************************************
#*************************************************************
#*************************************************************

def ACC_cal():


	'''
	calculate LCF and BCF using QP
	publish the next speed
	'''

	global m
	global g 
	global se 
	global F0 
	global F1 
	global F2 
	global gamma
	global ca 
	global cd 
	global psc 
	global x2
	global v0
	global vd 
	global D_old
	global D 
	global softness
	global initD 
	global dt
	global pub
	
	if (D <= 1.0):
		print "less than 1.0m"
		return 0.0
		pass

	z = D 
	y = x2 - vd 
	#print "x2 and vd and y", x2, vd, y
	Fr = F0 + F1*(y+vd)+F2*(y+vd)**2
	Vv0 = -2.0*y/m*Fr + se*(y**2)
	Vv1 = 2.0*y/m
	h = z-1.8*x2-0.3					#0.3m for parking
	#h = z-1.8*x2
	try:		
		B = -1.0* math.log(h/(1+h))
	except:
		h = 0.0001
		B = -1.0* math.log(h/(1+h))

	LfB = -1.0*(1.8*Fr + m*(v0 - x2))/(m*(1.0 - 1.8*x2 + z)*(-1.8*x2 + z));
	LgB = 1.8/(m*(1.0 - 1.8*x2 + z)*(-1.8*x2 + z))
	try:
		hf = -1.8*x2-0.5*((v0-x2)**2)/cd*g+z
		Bf = -1.0* math.log(hf/(1.0+hf))
	except:
		hf = 0.001
		Bf = -1.0* math.log(hf/(1.0+hf))		
	LfBf = (v0-x2-1.8)/(m*cd*g)/(-1.0*hf+hf**2)
	LgBf = (m*cd*g*(v0-x2)-Fr*(v0-x2-1.8))/(m*cd*g)/(-1.0*hf+hf**2)

	H = matrix([[2.0/(m**2), 0.0], [0.0, 2*psc]])		#P

	F = matrix([-2.0*Fr/(m**2), 0.0])					#q
	A = matrix([[Vv1, -1.0], [LgB, 0.0], [1.0,0.0], [1.0, 0.0], [LgBf, 0.0]]).T

	b = matrix([-1.0*Vv0, -1.0*LfB+gamma/B, vd/m*dt, ca*m*g, -1.0*LgBf+1.0/Bf])



	
	#A = matrix([[Vv1, -1.0], [LgB, 0.0], [1.0, 0.0], [-1.0, 0.0], [LgBf, 0.0]]).T

	#b = matrix([-1.0*Vv0, -1.0*LfB+gamma/B, ca*m*g, cd*m*g, -1.0*LgBf+1.0/Bf])



	
	try:
		
		U = cvxopt.solvers.qp(H, F, A, b)
		result = min(ca*g, max(-cd*g, (U["x"][0]/m)))

		#print "result, result, result, result", result
	

	
	except:
		result = 0.0
		pub.publish(upDateQueue(0.0))#========================
		print "oops Hit!!!"
		print "."
		print "."
		return result
		
	
	vel = result*dt+x2
	print result*dt, vel




	if (vel<0.0):
		pub.publish(upDateQueue(0.0))#========================
		print "Backup Protection"
		print "."
		print "."
	elif(vel>vd):
		vel = vd/0.1109		
		pub.publish(ms2rpm(upDateQueue(vd)))#=========================
		print "Speeding Protection"
		print "."
		print "."


	else:

		toPub = ms2rpm(upDateQueue(vel))			
		pub.publish(toPub)
		print "Moving Forward! Next speed:rpm", toPub
		#simuSpeed(toPub)
	

	return vel

#*************************************************************
#*************************************************************
#*************************************************************

def initialValues():

	'''
	boring initials
	'''

	global m
	global g 
	global se 
	global F0 
	global F1 
	global F2 
	global gamma
	global ca 
	global cd 
	global psc 
	global x2
	global v0
	global vd 
	global D_old
	global D 
	global softness
	global initD 
	global dt
	global preV
	global lowSpeed
	global rps2mps
	global preTime



	rospy.init_node('ACC')

	m = 4.5
	g = 9.81
	se = 10.0
	F0 = 0
	F1 = 0
	F2 = 36
	gamma = 1.0
	ca = 0.3
	cd = 0.3
	psc = math.exp(-5)
	lowSpeed = True
	rps2mps = 0.1109	

	preV = []
	preV.append(0.0)
	preV.append(0.0)
	preV.append(0.0)
	preV.append(0.0) 
	


	initD = 4.0
	vd = 3.0
	v0 = 0.0					#leading car speed
	x2 = 0.0					#current speed
	D = initD
	D_old = D
	softness = 1.0
	dt = 0.1					#initialized as 0.1s change with the time interval between each frame for the rest
	preTime = time.time()
	cvxopt.solvers.options['show_progress'] = False

#*************************************************************
#*************************************************************
#*************************************************************

def runACC():
	global pub
	
	pub = rospy.Publisher("/racer/ACC/nextSpeed", Float32, queue_size = 2)

	speedSub = rospy.Subscriber("/racer/teensy/rpm", Float32, updateSpeed)

	distanceSub = rospy.Subscriber("/racer/lidar/tester", Float32, updateDistance)

	pubtest = rospy.Publisher("/testtest", Float32, queue_size = 1)
	
	rospy.spin()

#*************************************************************
#*************************************************************
#*************************************************************

if __name__ == '__main__':

	initialValues()
	try:
		runACC();
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start ACC node.')

