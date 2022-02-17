#ifndef ODOM_H
#define ODOM_H
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "iostream"
#include <geometry_msgs/Twist.h>
#include <msglist/encoder.h>
#include <tf/transform_broadcaster.h>

#define pi 3.141593
 
class odometry
{
public:
	struct position{
	double x;
	double y;
	double theta;
	};

  odometry();

	ros::NodeHandle n;
	ros::Subscriber sub_encoder;
	
	ros::Publisher odom_pub;

	tf::TransformBroadcaster odom_broadcaster;
	tf::TransformBroadcaster laser_broadcaster;

  void Encoder_Callback(const msglist::encoder& encoder);
  
  ros::Time time_now;
  double current_time,last_time;
  double current_encoder[2],last_encoder[2],encoder_add[2];
  double pulse_per_round;
  double radius;
  double length;
  double pulse_per_meter;
  double wheel_v[2],v,w;
  position robot_position,robot_add;
  unsigned int sequence; 
  
};

#endif 
