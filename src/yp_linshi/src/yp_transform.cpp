#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>


void poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{  
	
	tf::TransformListener listener;

    geometry_msgs::PoseStamped pose_camera;
	geometry_msgs::PoseStamped pose_robot;

	pose_camera.header.frame_id = "camera_color_frame";
	pose_camera.header.stamp = ros::Time();
    pose_camera.pose.position = msg->position;
	pose_camera.pose.orientation.w = 1;
	//pose_robot.pose.orientation.w = 1;

	
	ros::Rate loop_rate(5);

	while(ros::ok())
{
        //tf::TransformListener listener(ros::Duration(10))//等待10s，如果10s之后都还没收到消息，那么之前的消息就被丢弃掉。
        //监听两个坐标之间的变换关系
        listener.waitForTransform("base_link","camera_color_frame",ros::Time(0),ros::Duration(3));//ros::Time(0)表示使用缓冲中最新的tf数据
        listener.transformPose("base_link",pose_camera,pose_robot);//将laser_link中的点变换到base_link中去

        ROS_INFO("camera_color_frame: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
        pose_camera.pose.position.x, pose_camera.pose.position.y, pose_camera.pose.position.z,
        pose_robot.pose.position.x, pose_robot.pose.position.y, pose_robot.pose.position.z, pose_robot.header.stamp.toSec());

		
        loop_rate.sleep();
}

    ROS_INFO_STREAM("the target positioin:\n"<<pose_robot);
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"transform_pose");
	ros::NodeHandle node;
	ros::Subscriber pose_trans = node.subscribe("/target_pose_camera",1,poseCallback);

	ros::spin();
	return 0;	
}
