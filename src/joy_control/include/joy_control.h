#ifndef JOY_CONTROL_H
#define JOY_CONTROL_H
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "iostream"
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

class joycontrol
{
public:
  joycontrol();

  enum joymode
 {
   stay=0,
   forward,
   back,
   left,
   right=4,
   reset=5,
 };

  joymode joy_mode;

  ros::NodeHandle n;
  ros::Subscriber sub_j;
	
  ros::Publisher joy_cmd_pub;

  void Joy_Callback(const sensor_msgs::Joy::ConstPtr& Joy);
  void set_speed(const double ,const double ,const double);

};

#endif 
