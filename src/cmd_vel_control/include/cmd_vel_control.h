#ifndef CMD_VEL_CONTROL_H
#define CMD_VEL_CONTROL_H
#include "ros/ros.h"
#include "serial/serial.h"
#include "iostream"
#include <std_msgs/String.h>
#include <msglist/encoder.h>
#include <geometry_msgs/Twist.h>

static int level;
 
class cmd_vel_control
{
public:
  cmd_vel_control();
  serial::Serial sp1;
  serial::Serial sp2;
  int encoder_count=0;
  
  struct rpm
  {
    int left;
    int right;
  };
  
  struct encoder
  {
    int round1;
    int round2;
  };

  enum prioritylevel
 {
   NONE=0,
   NAV=1,  //lowest
           //add other node here
   JOY=2     //highest
 };

  prioritylevel priority_level;

  ros::NodeHandle nh;
  ros::Subscriber sub_joy;
  ros::Subscriber sub_navigation;
//  ros::Subscriber sub_laser;	
//  subscribe any other node here

  ros::Publisher encoder_pub;

  void Joy_cmd_Callback(const geometry_msgs::Twist::ConstPtr& cmd);
  void Cmd_vel_Callback(const geometry_msgs::Twist::ConstPtr& cmd);
  void send_encoder();
  cmd_vel_control::rpm convert_speed_to_rpm(double ,double );
  void set_rpm(uint8_t * ,int );
  cmd_vel_control::encoder get_encoder(uint8_t *);


};

#endif 
