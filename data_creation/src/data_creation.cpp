#include <ros/ros.h>
#include <ros/package.h>

#include <string>
#include <fstream>
#include <boost/bind.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>


#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>

#include <iostream>
#include <fstream>
#include <algorithm>

int count = int(ros::WallTime::now().toSec());
cv::Mat left_image, right_image, composed_image;
int steer_cmd, acc_cmd;
bool record;
bool left_updated, right_updated;
std_msgs::Int32 steer_msg;
std_msgs::Int32 acc_msg;
ros::Publisher steer_pub;
ros::Publisher acc_pub;


#define MAX_STEER 6500

int pwm_center_value = 9830;  //  15% duty cycle - corresponds to zero velocity, zero steering
int pwm_lowerlimit = 6554;    //  10% duty cycle - corresponds to max reverse velocity, extreme left steering
int pwm_upperlimit = 13108;
int pwm_range = (pwm_upperlimit - pwm_lowerlimit);




void read_joy(const sensor_msgs::Joy::ConstPtr& joy_msg){
  double joy_steer, joy_acc, joy_brake;
  joy_steer = joy_msg->axes[0];
  joy_acc = joy_msg->axes[1];
  joy_brake = joy_msg->axes[2];
  record = joy_msg->buttons[0] & 0b00000000000000001;
  // std::cout << joy_steer <<", "<<joy_brake<<std::endl;
  if (joy_steer >= 0){
    steer_cmd = ((double) std::min(joy_steer,(double)MAX_STEER))/(MAX_STEER*2)*pwm_range +pwm_center_value;
  }else{
    steer_cmd = ((double) std::max(joy_steer,(double)-MAX_STEER))/(MAX_STEER*2)*pwm_range +pwm_center_value;
  }
  acc_cmd = ((double)(255 - joy_acc) - (255 - joy_brake))/510*pwm_range + pwm_center_value;
  // std::cout << steer_cmd <<", "<<acc_cmd<<std::endl;
  steer_msg.data = steer_cmd;
  acc_msg.data = acc_cmd;
  steer_pub.publish(steer_msg);
  acc_pub.publish(acc_msg);
  ros::spinOnce();
}

void writeToFile() //std_msgs::Header h
{
  // create file names
  std::stringstream filenameL, filenameR, filenameI, filetext;

  filenameI<<ros::package::getPath("data_creation")<<"/images/"<<count<<".png";
  filetext<<ros::package::getPath("data_creation")<<"/labels/"<<count<<".txt";

//  cv::imwrite(filenameL.str(),left_image);
//  cv::imwrite(filenameR.str(),right_image);
  if(record){
    ROS_INFO("Writing file");
    ROS_INFO(filenameI.str().c_str());
    cv::imwrite(filenameI.str(),composed_image);
    std::ofstream out;
    out.open(filetext.str().c_str());
    out << steer_cmd <<","<<acc_cmd <<std::endl;
    std::cout << steer_cmd <<","<<acc_cmd <<std::endl;
  }
  //else{
    //cv::putText(composed_image,"Press A on controller to Record",cv::Point(10,10),cv::FONT_HERSHEY_PLAIN,2,cv::Scalar(255,0,0));
  //}
  //cv::imshow("image",composed_image);
  //ros::spinOnce();
  //cv::waitKey(1);




  count++;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msgL, const sensor_msgs::ImageConstPtr& msgR )
{
  cv_bridge::CvImagePtr cv_ptr;

  cv_ptr = cv_bridge::toCvCopy(msgL, sensor_msgs::image_encodings::BGR8);
  left_image = cv_ptr->image; // convert msg to image

  cv_ptr = cv_bridge::toCvCopy(msgR, sensor_msgs::image_encodings::BGR8);
  right_image = cv_ptr->image;

  cv::hconcat(left_image, right_image, composed_image);
  //std_msgs::Header h = msgR->header;

  writeToFile();
}

int main(int argc, char **argv)
{
  record = false;
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  steer_cmd = pwm_center_value;
  acc_cmd = pwm_center_value;
  image_transport::Ima`geTransport it(nh);
  image_transport::SubscriberFilter itfilterL, itfilterR;
  ros::Subscriber joy_sub = nh.subscribe("joy",1,read_joy);

  steer_pub = nh.advertise<std_msgs::Int32>("racer/master/steer",1);
  acc_pub = nh.advertise<std_msgs::Int32>("racer/master/acc",1);
  itfilterL.subscribe(it,"/zed/left/image_rect_color", 1);
  itfilterR.subscribe(it,"/zed/right/image_rect_color", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> mysync;

  message_filters::Synchronizer<mysync> sync(mysync(10),itfilterL,itfilterR);
  sync.registerCallback(boost::bind(&imageCallback, _1, _2));

  ros::spin();
}
