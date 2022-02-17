#include "ros/ros.h"
#include "iostream"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/ApplyPlanningScene.h>


#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf/LinearMath/Quaternion.h>

//#include <moveit_msgs/RobotState.h>
#include "include/aubo_msgs/SetIO.h"
#include "include/aubo_msgs/SetIORequest.h"
#include "include/aubo_msgs/SetIOResponse.h"
#include "include/aubo_msgs/IOState.h"

#include <string>
#include <cstdlib>

#define PI 3.141592653

class Receive_move
{
public:
  ros::NodeHandle n;
  ros::Subscriber pose_sub;
  geometry_msgs::Pose target_pose;

  Receive_move();
  //~Receive_move();
  void robot_move1();
  void PoseCallback(const geometry_msgs::PoseStamped &pose_msg);
};

Receive_move::Receive_move()
{

  //ros::Rate rate(1);
  pose_sub = n.subscribe("/target_pose_robot", 1, &Receive_move::PoseCallback, this);
  ROS_INFO("Initialize target location information");

  int i = 6;
  while (i)
  {
    i--;
    //ROS_INFO("spin():%d",i);
    ros::spinOnce();
    //rate.sleep();
    sleep(1.0);
  }
  ROS_INFO("Initialization complete");
}

void Receive_move::PoseCallback(const geometry_msgs::PoseStamped &pose_msg)
{

  geometry_msgs::Pose pose_robot;

  pose_robot.position = pose_msg.pose.position;
  pose_robot.orientation.w = 1;

  target_pose = pose_robot;
  ROS_INFO("running callback");
}

void Receive_move::robot_move1()
{
  ROS_INFO("start move ,the location is:");
  std::cout << target_pose;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  static const std::string PLANNING_GROUP = "hand";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  move_group.setPoseReferenceFrame("arm_base_link");
  move_group.setMaxVelocityScalingFactor(0.06);     //max velocity
  move_group.setMaxAccelerationScalingFactor(0.03); //max Acceleration

  // Create a planning scene interface object
  //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Create a robot model information object
  //const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


ros::Publisher planning_scene_diff_publisher = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
ros::WallDuration sleep_t(0.5);
while (planning_scene_diff_publisher.getNumSubscribers() < 1)
{
  sleep_t.sleep();
}

  //********************************************************************************adding obstacles

  //***************Defining a desktop added in the world
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "arm_base_link";

  // Set the ID of the object
  collision_object.id = "desktop";

  // Set length, width and height
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.8;
  primitive.dimensions[1] = 0.9;
  primitive.dimensions[2] = 0.01;


  // Set the box (Desktop) pose
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.0;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  //***************Defining left wall added in the world
  moveit_msgs::CollisionObject collision_object2;
  collision_object2.header.frame_id = "arm_base_link";

  // Set the ID of the object
  collision_object2.id = "laser";

  // Set length, width and height
  shape_msgs::SolidPrimitive primitive2;
  primitive2.type = primitive2.BOX;
  primitive2.dimensions.resize(3);
  primitive2.dimensions[0] = 0.11;
  primitive2.dimensions[1] = 0.15;
  primitive2.dimensions[2] = 0.15;


  // Set the box (left wall) pose
  geometry_msgs::Pose box_pose2;
  box_pose2.orientation.w = 1.0;
  box_pose2.position.x = -0.33;
  box_pose2.position.y = 0.0;
  box_pose2.position.z = 0.075;
  
  collision_object2.primitives.push_back(primitive2);
  collision_object2.primitive_poses.push_back(box_pose2);
  collision_object2.operation = collision_object2.ADD;

  //***************Defining forward wall added in the world
  moveit_msgs::CollisionObject collision_object3;
  collision_object3.header.frame_id = "arm_base_link";

  // Set the ID of the object
  collision_object3.id = "gripper_desktop";

  // Set length, width and height
  shape_msgs::SolidPrimitive primitive3;
  primitive3.type = primitive.BOX;
  primitive3.dimensions.resize(3);
  primitive3.dimensions[0] = 0.40;
  primitive3.dimensions[1] = 0.15;
  primitive3.dimensions[2] = 0.22;


  // Set the box (left wall) pose
  geometry_msgs::Pose box_pose3;
  box_pose3.orientation.w = 1.0;
  box_pose3.position.x = -0.03;
  box_pose3.position.y = 0.374;
  box_pose3.position.z = 0.11;

  collision_object3.primitives.push_back(primitive3);
  collision_object3.primitive_poses.push_back(box_pose3);
  collision_object3.operation = collision_object.ADD;

  //***************Defining right wall added in the world
  moveit_msgs::CollisionObject collision_object4;
  collision_object4.header.frame_id = "arm_base_link";

  // Set the ID of the object
  collision_object4.id = "controll_desktop";

  // Set length, width and height
  shape_msgs::SolidPrimitive primitive4;
  primitive4.type = primitive.BOX;
  primitive4.dimensions.resize(3);
  primitive4.dimensions[0] = 0.22;
  primitive4.dimensions[1] = 0.18;
  primitive4.dimensions[2] = 0.055;

  // Set the box (left wall) pose
  geometry_msgs::Pose box_pose4;
  box_pose4.orientation.w = 1.0;
  box_pose4.position.x = 0.0;
  box_pose4.position.y = -0.35;
  box_pose4.position.z = 0.0275;

  collision_object4.primitives.push_back(primitive4);
  collision_object4.primitive_poses.push_back(box_pose4);
  collision_object4.operation = collision_object.ADD;


 moveit_msgs::CollisionObject collision_object5;
  collision_object4.header.frame_id = "arm_base_link";

  // Set the ID of the object
  collision_object4.id = "camera";

  // Set length, width and height
  shape_msgs::SolidPrimitive primitive5;
  primitive5.type = primitive.BOX;
  primitive5.dimensions.resize(3);
  primitive5.dimensions[0] = 0.1;
  primitive5.dimensions[1] = 0.1;
  primitive5.dimensions[2] = 0.13;

  // Set the box (left wall) pose
  geometry_msgs::Pose box_pose5;
  box_pose5.orientation.w = 1.0;
  box_pose5.position.x = -0.26;
  box_pose5.position.y = -0.25;
  box_pose5.position.z = 0.075;

  collision_object5.primitives.push_back(primitive5);
  collision_object5.primitive_poses.push_back(box_pose5);
  collision_object5.operation = collision_object.ADD;


//定义一个PlanningScene消息
moveit_msgs::PlanningScene planning_scene;
planning_scene.world.collision_objects.push_back(collision_object);
planning_scene.world.collision_objects.push_back(collision_object2);
planning_scene.world.collision_objects.push_back(collision_object3);
planning_scene.world.collision_objects.push_back(collision_object4);
planning_scene.world.collision_objects.push_back(collision_object5);
planning_scene.is_diff = true;
//发布该消息
planning_scene_diff_publisher.publish(planning_scene);

  ROS_INFO("Add obstacles complete");
  //*****************************************************************************************

  //*******************             Define common  Position

  std::vector<double> home_position;
  home_position.push_back(0);
  home_position.push_back(0);
  home_position.push_back(0);
  home_position.push_back(0);
  home_position.push_back(0);
  home_position.push_back(0);

 
  std::vector<double> above_gripper_position_zhua; //夹抓位姿，应该可以和夹取的起始位置合并，但怕误差暂时不合
  above_gripper_position_zhua.push_back(-105.353 * PI / 180);
  above_gripper_position_zhua.push_back(-15.777 * PI / 180);
  above_gripper_position_zhua.push_back(-114.907 * PI / 180);
  above_gripper_position_zhua.push_back(-8.367 * PI / 180);
  above_gripper_position_zhua.push_back(-89.313 * PI / 180);
  above_gripper_position_zhua.push_back(169.396 * PI / 180);

  std::vector<double> fix_gripper_position_zhua;
  fix_gripper_position_zhua.push_back(-105.353 * PI / 180);
  fix_gripper_position_zhua.push_back(-15.093 * PI / 180);
  fix_gripper_position_zhua.push_back(-117.970 * PI / 180);
  fix_gripper_position_zhua.push_back(-12.114 * PI / 180);
  fix_gripper_position_zhua.push_back(-89.313 * PI / 180);
  fix_gripper_position_zhua.push_back(169.396 * PI / 180);

  std::vector<double> up_gripper_zhua;                //换夹爪后提升的位置
  up_gripper_zhua.push_back(-105.353 * PI / 180);
  up_gripper_zhua.push_back(-12.700 * PI / 180);
  up_gripper_zhua.push_back(-81.271 * PI / 180);
  up_gripper_zhua.push_back(22.192 * PI / 180);
  up_gripper_zhua.push_back(-89.313* PI / 180);
  up_gripper_zhua.push_back(169.396 * PI / 180);


  tf::Quaternion q;
  q.setRPY(1.57, 0, -1.57);                //水平向前的位姿

  geometry_msgs::Pose target_pose1 , target_pose1_1,target_pose1_2;
  target_pose1.position.x = target_pose.position.x+0.050;
  target_pose1.position.y = target_pose.position.y-0.002;
  target_pose1.position.z = target_pose.position.z+0.079;
  target_pose1.orientation.x = q.x();
  target_pose1.orientation.y = q.y();
  target_pose1.orientation.z = q.z();
  target_pose1.orientation.w = q.w();

  target_pose1_1 = target_pose1;
  target_pose1_1.position.x = target_pose1.position.x+0.25;   //准备抓取的位置

  target_pose1_2 = target_pose1;
  target_pose1_2.position.x = target_pose1.position.x+0.15;   //准备抓取的位置


  //wait to service(bloking type)
  ros::service::waitForService("/aubo_driver/set_io");
  ros::ServiceClient client = n.serviceClient<aubo_msgs::SetIO>("/aubo_driver/set_io");

  //define the user IO data array
  aubo_msgs::SetIO U_DO_00, U_DO_01, U_DO_02, U_DO_03, U_DO_04, U_DO_05;
  int fun = 1; //expressed as User IO

  //Define pins 0-5
  //falan1 open
  U_DO_00.request.fun = fun;
  U_DO_00.request.pin = 0;
  U_DO_00.request.state = 1;

  //falan 2
  U_DO_01.request.fun = fun;
  U_DO_01.request.pin = 1;
  U_DO_01.request.state = 1;

  //gripper
  U_DO_02.request.fun = fun;
  U_DO_02.request.pin = 2;
  U_DO_02.request.state = 1;

  U_DO_03.request.fun = fun;
  U_DO_03.request.pin = 3;
  U_DO_03.request.state = 1;

  U_DO_04.request.fun = fun;
  U_DO_04.request.pin = 4;
  U_DO_04.request.state = 1;

  U_DO_05.request.fun = fun;
  U_DO_05.request.pin = 5;
  U_DO_05.request.state = 1;
/*
  ROS_INFO("move to above of gripper position");
  move_group.setJointValueTarget(above_gripper_position_zhua);
  move_group.move();
  sleep(1.0);

  move_group.setMaxVelocityScalingFactor(0.02);     //max velocity
  if (client.call(U_DO_00))
  {
    ROS_INFO("U__D0_00: %ld", (long int)U_DO_00.response.success);
    sleep(0.5);
    move_group.setJointValueTarget(fix_gripper_position_zhua);
    move_group.move();
  }
  else
  {
    ROS_ERROR("Failed to set io U_D0_00");
    ros::shutdown();
  }

  U_DO_00.request.state = 0;
  client.call(U_DO_00);
  sleep(0.5);

  client.call(U_DO_01);
  sleep(1.0);
  U_DO_01.request.state = 0;
  client.call(U_DO_01);
  
  ROS_INFO("move to above of gripper position");
  move_group.setJointValueTarget(above_gripper_position_zhua);
  move_group.move();

  ROS_INFO("up gripper:");
  move_group.setJointValueTarget(up_gripper_zhua);
  move_group.move();
*/
  move_group.setMaxVelocityScalingFactor(0.06);     //max velocity
  ROS_INFO("move to target position1_1:");
  std::cout << target_pose1_1;
  move_group.setPoseTarget(target_pose1_1);
  move_group.move();

  U_DO_04.request.state = 0;
  client.call(U_DO_04);
  client.call(U_DO_03);
  sleep(0.8);
  U_DO_03.request.state = 0;
  client.call(U_DO_03);

  ROS_INFO("move to target position1_2:");
  move_group.setPoseTarget(target_pose1_2);
  move_group.move();
  
  ROS_INFO("move to target position1:");
  move_group.setPoseTarget(target_pose1);
  move_group.move();

U_DO_04.request.state = 1;
  ROS_INFO("Reach the target point");
  sleep(0.5);
  client.call(U_DO_04); //   夹抓闭合

  move_group.setMaxVelocityScalingFactor(0.06);     //max velocity
  ROS_INFO("Return to HOME position...................");
 move_group.setJointValueTarget(home_position);
 move_group.move();
 
  // Remove the obstacles
  ROS_INFO("Remove the desktop from the world");

moveit_msgs::CollisionObject remove_obj1 , remove_obj2  ,remove_obj3 ,remove_obj4,remove_obj5;
remove_obj1.id=collision_object.id;
remove_obj2.id=collision_object2.id;
remove_obj3.id=collision_object3.id;
remove_obj4.id=collision_object4.id;
remove_obj5.id=collision_object5.id;

remove_obj1.operation = remove_obj1.REMOVE;
remove_obj2.operation = remove_obj2.REMOVE;
remove_obj3.operation = remove_obj3.REMOVE;
remove_obj4.operation = remove_obj4.REMOVE;
remove_obj5.operation = remove_obj5.REMOVE;

planning_scene.world.collision_objects.clear();

planning_scene.world.collision_objects.push_back(remove_obj1);
planning_scene.world.collision_objects.push_back(remove_obj2);
planning_scene.world.collision_objects.push_back(remove_obj3);
planning_scene.world.collision_objects.push_back(remove_obj4);
planning_scene.world.collision_objects.push_back(remove_obj5);
//发布消息
planning_scene_diff_publisher.publish(planning_scene);



  //ros::shutdown();

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "auto_move_node2");
  ;
  Receive_move move1;

  int i = 3;
  while (i)
  {
    ROS_INFO("sleep %ds", i);
    sleep(1);
    --i;
  }
  move1.robot_move1();
  ROS_INFO("robot move end");
  return 0;
}
