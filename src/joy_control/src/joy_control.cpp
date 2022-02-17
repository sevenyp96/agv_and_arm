#include "joy_control.h"


joycontrol::joycontrol()
{
  joy_mode=stay;
//sub
  sub_j = n.subscribe("joy",10,&joycontrol::Joy_Callback,this);
//pub
  joy_cmd_pub = n.advertise<geometry_msgs::Twist>("joy_cmd", 1);
}

void joycontrol::Joy_Callback(const sensor_msgs::Joy::ConstPtr& joy)
{
  joy_mode=reset;
  if (joy->buttons[5]==1)
{
  if(joy->axes[1]==1) 
  {
  	joy_mode=forward;
  }
  else if(joy->axes[1]==-1)
  {
  	joy_mode=back;
  }
  
  if(joy->axes[0]==1) 
  {
  	joy_mode=left;
  }
  else if(joy->axes[0]==-1)
  {
  	joy_mode=right;
  }
}
  if(joy->buttons[0]==1||joy->buttons[1]==1)
  joy_mode=stay;
  if(joy->buttons[2]==1||joy->buttons[3]==1)
  joy_mode=reset;

  switch(joy_mode)
  {
  case forward: 
  set_speed(0.3,0,0);        //0.2
  break;
  
  case back: 
  set_speed(-0.3,0,0);       //-0.2
  break;
    
  case left: 
  set_speed(0,0.4,0);           //0.3
  break;
    
  case right: 
  set_speed(0,-0.4,0);             //-0.3
  break;
    
  case stay: 
  set_speed(0,0,1);
  break;

  case reset: 
  set_speed(0,0,-1);
  break;

  default:
  set_speed(0,0,-1);
  }

}


void joycontrol::set_speed(const double linear,const double angular,const double button)
{	

    geometry_msgs::Twist twist;
	twist.linear.x=linear;
	twist.linear.y=0;
	twist.linear.z=0;

	twist.angular.x=0;
	twist.angular.y=0;
	twist.angular.z=angular;

	twist.linear.y=button;//set priority level
	
	joy_cmd_pub.publish(twist);
//    ROS_INFO_STREAM("send!");
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "joy_control_node");
  joycontrol joy1;
  ros::Rate loop_rate(10); 		
  		
    while(ros::ok())
    {
    ros::spinOnce();
    loop_rate.sleep();
    }
    
    //关闭串口
 //   sp3.close();
 
  ros::spin();

  return 0;
}
