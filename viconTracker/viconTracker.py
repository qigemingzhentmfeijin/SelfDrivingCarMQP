#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import PoseStamped as PS
from geometry_msgs.msg import Pose

def callback(data):
    global pose
    pose.Pose.Point.x=data.data[0]
    pose.Pose.Point.y=data.data[1]
    pose.Pose.Point.z=data.data[2]
    pose.Pose.Quaternion.x=data.data[3]
    pose.Pose.Quaternion.x=data.data[4]
    pose.Pose.Quaternion.x=data.data[5]

def talker():
    global pose
    pose = PS()
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('viconTracker', PS, queue_size=1)
    sub = rospy.Subscriber('viconPoseData', Float32[], callback) #data type??

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass