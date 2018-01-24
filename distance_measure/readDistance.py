import rospy
import numpy
import time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import sensor_msgs.point_cloud2

global tototaimu


def callback(data):
    global pub
    global tototaimu
    if((time.time()-0.1)>=tototaimu):
        result = 0
        x = 5.0

        for p in sensor_msgs.point_cloud2.read_points(data, field_names = ("x", "y", "z"),skip_nans=True):      
            if x > p[0]: 
                x = p[0]
        result = Float32MultiArray()
        result.data = [0]     
        result.data[0] = offsetDis(x)
 
        print "distance: ", result.data[0]
        pub.publish(result)
        tototaimu = time.time()

    else:pass
    
    
	
def offsetDis(dis):
    return -0.0393*dis*dis+0.4448*dis+0.1797
    



def listener():
    global pub
    #print "!!!"
    rospy.Subscriber("/cropbox/output",PointCloud2,callback)
    pub = rospy.Publisher("/racer/zed/distance", Float32MultiArray, queue_size = 1)
    #print "in listener"
    rospy.spin()
	
    	







if __name__ == '__main__':
    global tototaimu
    print "Start checking front distance"
    tototaimu = 0.0
    rospy.init_node('readDistance_node')
    
    listener()
        
	
