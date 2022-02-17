#include "ros/ros.h"
#include "iostream"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class send_position
{
public:
	ros::NodeHandle node;
	ros::Subscriber pose_trans;                 //subscriber pose in camera
	ros::Publisher target_pose_robot;           //pubilisher pose in robot
	geometry_msgs::PoseStamped pose_robot_end;  //the meadia value of pose  

	send_position();
	void PoseCallback(const geometry_msgs::Pose &pose_msg);
};

send_position::send_position()
{

	ros::Rate rate(10);

	//set_vel=n.advertise<geometry_msgs::Twist>("set_cmd_vel",1);
	//view_pose=n.subscribe("odom",10,&yidong::PoseCallback,this);
	pose_trans = node.subscribe("/target_pose_camera", 1, &send_position::PoseCallback, this);
	target_pose_robot = node.advertise<geometry_msgs::PoseStamped>("/target_pose_robot", 1);
	while (node.ok())
	{
		target_pose_robot.publish(pose_robot_end);
		ros::spinOnce();
		rate.sleep();
	}
}

void send_position::PoseCallback(const geometry_msgs::Pose &pose_msg)
{
	tf::TransformListener listener;

	geometry_msgs::PoseStamped pose_camera;
	geometry_msgs::PoseStamped pose_robot;

	pose_camera.header.frame_id = "camera_color_optical_frame";
	pose_camera.header.stamp = ros::Time();
	pose_camera.pose.position = pose_msg.position;
	pose_camera.pose.orientation.w = 1;
	//pose_robot.pose.orientation.w = 1;

	//tf::TransformListener listener(ros::Duration(10))//等待10s，如果10s之后都还没收到消息，那么之前的消息就被丢弃掉。
	//监听两个坐标之间的变换关系
	listener.waitForTransform("arm_base_link", "camera_color_frame", ros::Time(0), ros::Duration(0.5)); //ros::Time(0)表示使用缓冲中最新的tf数据
	listener.transformPose("arm_base_link", pose_camera, pose_robot);								  //将laser_link中的点变换到base_link中去

	ROS_INFO("camera_color_frame: (%.2f, %.2f. %.2f) -----> arm_base_link: (%.2f, %.2f, %.2f) at time %.2f",
			 pose_camera.pose.position.x, pose_camera.pose.position.y, pose_camera.pose.position.z,
			 pose_robot.pose.position.x, pose_robot.pose.position.y, pose_robot.pose.position.z, pose_robot.header.stamp.toSec());

	pose_robot_end = pose_robot;

	ROS_INFO_STREAM("the target positioin:\n"
					<< pose_robot);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "auto_move_node");
	send_position send1;
	ros::spin();
}
