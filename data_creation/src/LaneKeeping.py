import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import atexit




class Lane_Keeping:	
	def estop(self):
		print "stopping"
		self.pub_msg_ebrake.publish(True)
	def __init__(self):
		rospy.init_node('lane_keeping')
		self.bridge = CvBridge()
		#self.pub_img_gray = rospy.Publisher('/lane_keeping/image_gray', Image, queue_size=1)
		#self.pub_img_inv = rospy.Publisher('/lane_keeping/image_inv', Image, queue_size=1)
		self.pub_img_pro = rospy.Publisher('/lane_keeping/image_processing', Image, queue_size=1)
		self.pub_img_thr = rospy.Publisher('/lane_keeping/image_thresholding', Image, queue_size=1)
		self.pub_msg_error = rospy.Publisher('lane_keeping_error', Int32, queue_size=1)
		self.pub_msg_steering = rospy.Publisher('racer/master/steer', Int32, queue_size=1)

		self.pub_msg_ebrake = rospy.Publisher('racer/master/estop', Bool, queue_size=1)
		self.pub_msg_speed = rospy.Publisher('racer/master/acc', Int32, queue_size=1)
		rospy.Subscriber("/zed/right/image_rect_color", Image, self.callback)
		# spin() simply keeps python from exiting until this node is stopped	
		
		
		
		rospy.sleep(0.001)
		
		rospy.on_shutdown(self.estop)
		rospy.spin()

	def callback(self, msg):
		#-- read image
		#image_1 = cv2.imread('1494801946.png',1)
		image_1 = self.bridge.imgmsg_to_cv2(msg, "bgr8")

		#-- convert from RGB to gray
		gray_image = cv2.cvtColor(image_1, cv2.COLOR_BGR2GRAY)
		
		
		#gray_img_msg = self.bridge.cv2_to_imgmsg(gray_image, "mono8") 
		#self.pub_img_gray.publish(gray_img_msg)#-- publish image after image processing

		#-- crop image
		crop_img = gray_image[176:276,:]

		#-- inverse light
		inv_img=255-crop_img

		#inv_img_msg = self.bridge.cv2_to_imgmsg(inv_img, "mono8") 
		#self.pub_img_inv.publish(inv_img_msg)#-- publish image after inverse light


		#-- remove the light
		kernel = np.ones((20,20),np.uint8)
		background = cv2.morphologyEx(inv_img,cv2.MORPH_OPEN,kernel)
		image_2=inv_img-background

		#-- increase image contrast
		image_3 = cv2.equalizeHist(image_2)
		image_3_msg = self.bridge.cv2_to_imgmsg(image_3, "mono8") 
		self.pub_img_pro.publish(image_3_msg)#-- publish image after image processing


		#-- thresholding
		ret,thresh_img = cv2.threshold(image_3,127,255,cv2.THRESH_BINARY)

		image_thr_msg = self.bridge.cv2_to_imgmsg(thresh_img, "mono8") 
		self.pub_img_thr.publish(image_thr_msg)#-- publish image after thresholding

		#-- calculate the nearest lane marker to the reference point
		array1=thresh_img[60,1:320]
		array1_rev=array1[::-1]
		array2=thresh_img[60,321:640]

		x1=next((i for i, v in enumerate(array1) if v >0 ), 320)
		x2=next((i for i, v in enumerate(array2) if v >0 ), 320)

		sign=x1-x2

		error=x1*(sign<0)+x2*(sign>0);
		self.pub_msg_error.publish(error)#-- publish error

		#-- calculate control command using P(PI/PD/PID?) controller

		velo_x_max=360
		velo_y_min=5
		velo_y_max=60
		Px=3


		velo_x=error*Px
		if abs(velo_x)>velo_x_max:
		    velo_x=velo_x/abs(velo_x)*velo_x_max
		velo_y=(velo_x_max-abs(velo_x))/velo_x_max*(velo_y_max-velo_y_min)+velo_y_min
		rospy.loginfo("velo_x=%d, velo_y=%d", velo_x,velo_y)
		
		#TODO: Remove this when steering is converted within Teensy
		pwm_center_value = 9830.0;  
		pwm_lowerlimit = 6554.0;    
		pwm_upperlimit = 13108.0;
		velo_x = (velo_x / (2.0*velo_x_max)) * (pwm_upperlimit - pwm_lowerlimit) + pwm_center_value
		velo_y =pwm_center_value
		self.pub_msg_ebrake.publish(False)
		self.pub_msg_steering.publish(velo_x)#-- publish steering angle ref
		self.pub_msg_speed.publish(velo_y)#-- publish speed ref
		

if __name__ == '__main__':
	keeper=Lane_Keeping()

