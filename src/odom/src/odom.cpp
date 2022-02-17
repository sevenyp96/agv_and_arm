#include "odom.h"


odometry::odometry()
{
	//init
	time_now=ros::Time::now();
	current_time=time_now.toSec();
	last_time=current_time;
	pulse_per_round=122350.0;
	radius=0.150;
	length=0.450;
	current_encoder[0]=0.0;
	current_encoder[1]=0.0;	
	last_encoder[0]=0.0;
	last_encoder[1]=0.0;	
	encoder_add[0]=0.0;
	encoder_add[1]=0.0;
	v=0.0;
	w=0.0;	
	robot_position.x=0.0;
	robot_position.y=0.0;
	robot_position.theta=0.0;
	
	robot_add.x=0.0;
	robot_add.y=0.0;
	robot_add.theta=0.0;
	
	sequence=1;
	
	pulse_per_meter=pulse_per_round/(2*pi*radius);

  sub_encoder = n.subscribe("encoder",10,&odometry::Encoder_Callback,this);
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);

}

void odometry::Encoder_Callback(const msglist::encoder& encoder)
{
	v=0.0;
	w=0.0;	
	if(encoder.encoder_left!=0 && encoder.encoder_left!=0)
{	
	last_time=current_time;
	current_time=encoder.current_time.sec+encoder.current_time.nsec/1000000000.0;
	double time_diff=current_time-last_time;
//	std::cout<< time_diff << std::endl;
//	std::cout<< time_diff << std::endl;
	last_encoder[0]=current_encoder[0];
	last_encoder[1]=current_encoder[1];
	current_encoder[0]=encoder.encoder_left;
	current_encoder[1]=encoder.encoder_right;
	encoder_add[0]=current_encoder[0]-last_encoder[0];
	encoder_add[1]=current_encoder[1]-last_encoder[1];
	wheel_v[0]=encoder_add[0]/pulse_per_meter/time_diff;
	wheel_v[1]=encoder_add[1]/pulse_per_meter/time_diff;
	v=(wheel_v[0]+wheel_v[1])/2;
	w=(wheel_v[0]-wheel_v[1])/length;                      //1\0hudiao
	
//	 std::cout<<"count:  "<<encoder.count<<std::endl;
		if(encoder.count<10)
	{		
		v=0.0;
		w=0.0;
//		std::cout<<"v1:  "<< v << std::endl;
//	  std::cout<<"w1:  "<<w << std::endl<<std::endl;
	}
		//robot_add.x=v*time_diff*std::cos(robot_position.theta+robot_add.theta/2);
		//robot_add.y=v*time_diff*std::sin(robot_position.theta+robot_add.theta/2);
		robot_add.theta=w*time_diff;

		robot_add.x=v*time_diff*std::cos(robot_position.theta+robot_add.theta);
		robot_add.y=v*time_diff*std::sin(robot_position.theta+robot_add.theta);


		robot_add.theta=w*time_diff;
		
		robot_position.x+=robot_add.x;
		robot_position.y+=robot_add.y;
		robot_position.theta+=robot_add.theta;

      if(robot_position.theta > pi)
     {
	robot_position.theta-= 2*pi;	
     }
	
      if(robot_position.theta < -pi)
     {
	robot_position.theta+= 2*pi;	
     }
	ROS_INFO("x,y,theta,v,m");
	std::cout<< "x:  "<<robot_position.x << std::endl;
    std::cout<< "y:  "<<robot_position.y << std::endl;
	std::cout<< "theta:  "<<robot_position.theta << std::endl;	
	std::cout<<"v:  "<< v << std::endl;
	std::cout<<"w:  "<< w << std::endl<<std::endl;		
	
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robot_position.theta);
	
	nav_msgs::Odometry odomdata;
	odomdata.header.stamp=ros::Time::now();
	odomdata.header.seq=sequence;
	odomdata.header.frame_id="odom";
	odomdata.child_frame_id="base_link";
	odomdata.pose.pose.position.x=robot_position.x;
	odomdata.pose.pose.position.y=robot_position.y;
	odomdata.pose.pose.position.z=0.0;
	odomdata.pose.pose.orientation=odom_quat;
	
	odomdata.twist.twist.linear.x=v;
	odomdata.twist.twist.angular.z=w;
	
	odom_pub.publish(odomdata);
	sequence++;


	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp=ros::Time::now();
	odom_trans.header.frame_id="odom";
	odom_trans.child_frame_id="base_link";
	odom_trans.transform.translation.x=robot_position.x;
	odom_trans.transform.translation.y=robot_position.y;
	odom_trans.transform.translation.z=0.0;
	odom_trans.transform.rotation=odom_quat;

	odom_broadcaster.sendTransform(odom_trans);		


        //5.23change,odomdata.pose.pose.orientation.z shows the thrat of the VGA realative of the odom 
	//laser_broadcaster.sendTransform(
	//tf::StampedTransform(
	//tf::Transform(tf::Quaternion(0,0,odomdata.pose.pose.orientation.z,odomdata.pose.pose.orientation.w),tf::Vector3(0.31,0.0,0.68)),ros::Time::now(),"base_link","base_laser"));
 
     //tf trans move  to VLP16_points_yp.launch
	//laser_broadcaster.sendTransform(
	//tf::StampedTransform(
	//tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(0.533,0.000,0.396)),ros::Time::now(),"base_link","base_laser"));

	
}	
	
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom");
  odometry odom1;		 
  ros::spin();

  return 0;
}
