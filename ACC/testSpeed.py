import rospy
from std_msgs.msg import Float32


def Send(vel):
	global pub
	while(True):
		pub.publish(vel)
		print "start"

if __name__ == '__main__':
	global pub
	rospy.init_node('speed')
	pub = rospy.Publisher('racer/ACC/nextSpeed', Float32, queue_size=1)
	Send(30)
	rospy.spin()
	

