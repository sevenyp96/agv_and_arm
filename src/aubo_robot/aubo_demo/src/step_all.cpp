/*该文件是车臂联动的初始文件。流程为：车到A点，臂更换吸盘吸取目标，车到B 点，吸盘放下目标
更换夹爪，夹取目标，车到C点，臂放下目标，放下夹爪;车到D点*/
#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <string.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

#include "iostream"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf/LinearMath/Quaternion.h>

#include <moveit_msgs/RobotState.h>
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
  void robot_move1();
  void robot_move2();
  void robot_move3();
  void PoseCallback(const geometry_msgs::PoseStamped &pose_msg);
};

Receive_move::Receive_move()
{

  //ros::Rate rate(1);
  pose_sub = n.subscribe("/target_pose_robot", 1, &Receive_move::PoseCallback, this);
  ROS_INFO("Initialize target location information");

  for (int i = 1; i < 4; i++)
  {
    ROS_INFO("spin():%d", i);
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
  const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

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

  std::vector<double> up_gripper_zhua; //换夹爪后提升的位置
  up_gripper_zhua.push_back(-105.353 * PI / 180);
  up_gripper_zhua.push_back(-12.700 * PI / 180);
  up_gripper_zhua.push_back(-81.271 * PI / 180);
  up_gripper_zhua.push_back(22.192 * PI / 180);
  up_gripper_zhua.push_back(-89.313 * PI / 180);
  up_gripper_zhua.push_back(169.396 * PI / 180);

  tf::Quaternion q;
  q.setRPY(1.57, 0, -1.57); //水平向前的位姿

  geometry_msgs::Pose target_pose1, target_pose1_1, target_pose1_2;
  target_pose1.position.x = target_pose.position.x + 0.050;
  target_pose1.position.y = target_pose.position.y - 0.002;
  target_pose1.position.z = target_pose.position.z + 0.079;
  target_pose1.orientation.x = q.x();
  target_pose1.orientation.y = q.y();
  target_pose1.orientation.z = q.z();
  target_pose1.orientation.w = q.w();

  target_pose1_1 = target_pose1;
  target_pose1_1.position.x = target_pose1.position.x + 0.25; //准备抓取的位置

  target_pose1_2 = target_pose1;
  target_pose1_2.position.x = target_pose1.position.x + 0.15; //准备抓取的位置

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

 move_group.setMaxVelocityScalingFactor(0.2);     //max velocity
  ROS_INFO("move to above of gripper position");
  move_group.setJointValueTarget(above_gripper_position_zhua);
  move_group.move();
  sleep(0.5);

  move_group.setMaxVelocityScalingFactor(0.03); //max velocity
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

  move_group.setMaxVelocityScalingFactor(0.15); //max velocity
  ROS_INFO("up gripper:");
  move_group.setJointValueTarget(up_gripper_zhua);
  move_group.move();

  move_group.setMaxVelocityScalingFactor(0.15); //max velocity
  ROS_INFO("move to target position1_1:");
  std::cout << target_pose1_1;
  move_group.setPoseTarget(target_pose1_1);
  move_group.move();

  client.call(U_DO_03);
  sleep(0.8);
  U_DO_03.request.state = 0;
  client.call(U_DO_03);

  move_group.setMaxVelocityScalingFactor(0.06);     //max velocity
  ROS_INFO("move to target position1_2:");
  move_group.setPoseTarget(target_pose1_2);
  move_group.move();

  ROS_INFO("move to target position1:");
  move_group.setPoseTarget(target_pose1);
  move_group.move();

  ROS_INFO("Reach the target point");
  sleep(0.5);
  client.call(U_DO_04); //   夹抓闭合

  move_group.setMaxVelocityScalingFactor(0.2); //max velocity
  ROS_INFO("Return to HOME position...................");
  move_group.setJointValueTarget(home_position);
  move_group.move();

  // Remove the obstacles
  ROS_INFO("Remove the desktop from the world");

  moveit_msgs::CollisionObject remove_obj1, remove_obj2, remove_obj3, remove_obj4, remove_obj5;
  remove_obj1.id = collision_object.id;
  remove_obj2.id = collision_object2.id;
  remove_obj3.id = collision_object3.id;
  remove_obj4.id = collision_object4.id;
    remove_obj5.id = collision_object5.id;

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

void Receive_move::robot_move2()
{

  int j = 3;
  while (j)
  {
    j--;
    ros::spinOnce();
    sleep(1.0);
  }

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
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Create a robot model information object
  const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

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

  std::vector<double> above_gripper_position_xi; //吸盘位姿，应该可以和夹取的起始位置合并，但怕误差暂时不合
  above_gripper_position_xi.push_back(-122.844 * PI / 180);
  above_gripper_position_xi.push_back(-14.099 * PI / 180);
  above_gripper_position_xi.push_back(-113.840 * PI / 180);
  above_gripper_position_xi.push_back(-9.987 * PI / 180);
  above_gripper_position_xi.push_back(-90.089 * PI / 180);
  above_gripper_position_xi.push_back(152.667 * PI / 180);

  std::vector<double> fix_gripper_position_xi;
  fix_gripper_position_xi.push_back(-122.844 * PI / 180);
  fix_gripper_position_xi.push_back(-13.377 * PI / 180);
  fix_gripper_position_xi.push_back(-116.893 * PI / 180);
  fix_gripper_position_xi.push_back(-13.762 * PI / 180);
  fix_gripper_position_xi.push_back(-90.089 * PI / 180);
  fix_gripper_position_xi.push_back(152.666 * PI / 180);

  std::vector<double> xie_zhua_position; //卸下夹爪的位置
  xie_zhua_position.push_back(-105.362 * PI / 180);
  xie_zhua_position.push_back(-15.507 * PI / 180);
  xie_zhua_position.push_back(-117.301 * PI / 180);
  xie_zhua_position.push_back(-11.451 * PI / 180);
  xie_zhua_position.push_back(-89.313 * PI / 180);
  xie_zhua_position.push_back(169.396 * PI / 180);

  std::vector<double> up_gripper_xi; //换吸盘后提升的位置
  up_gripper_xi.push_back(-122.844 * PI / 180);
  up_gripper_xi.push_back(-14.352 * PI / 180);
  up_gripper_xi.push_back(-96.340 * PI / 180);
  up_gripper_xi.push_back(7.765 * PI / 180);
  up_gripper_xi.push_back(-90.089 * PI / 180);
  up_gripper_xi.push_back(152.666 * PI / 180);

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

  std::vector<double> up_gripper_zhua; //换夹爪后提升的位置
  up_gripper_zhua.push_back(-105.353 * PI / 180);
  up_gripper_zhua.push_back(-12.700 * PI / 180);
  up_gripper_zhua.push_back(-81.271 * PI / 180);
  up_gripper_zhua.push_back(22.192 * PI / 180);
  up_gripper_zhua.push_back(-89.313 * PI / 180);
  up_gripper_zhua.push_back(169.396 * PI / 180);

  tf::Quaternion q1, q2;
  q1.setRPY(-3.14, 0, 1.57); //吸取的姿态，垂直向下
  q2.setRPY(1.57, 0, -1.57); //夹取的姿态，水平向前

  geometry_msgs::Pose target_pose2; //吸取后的固定位置
  target_pose2.position.x = -0.0493;
  target_pose2.position.y = -0.4983;
  target_pose2.position.z = 0.4658 - 0.18;
  target_pose2.orientation.x = q1.x();
  target_pose2.orientation.y = q1.y();
  target_pose2.orientation.z = q1.z();
  target_pose2.orientation.w = q1.w();
  /*
  std::vector<double> target_pose2;                //换吸盘后提升的位置
  target_pose2.push_back(-81.603 * PI / 180);
  target_pose2.push_back(-1.565 * PI / 180);
  target_pose2.push_back(82.250 * PI / 180);
  target_pose2.push_back(-8.204 * PI / 180);
  target_pose2.push_back(89.939 * PI / 180);
  target_pose2.push_back(-81.584 * PI / 180);
*/
  geometry_msgs::Pose target_pose1, target_pose1_1, target_pose1_2, target_pose1_2_1, target_pose1_3, target_pose1_4;

  target_pose1.position.x = target_pose.position.x; //初始坐标
  target_pose1.position.y = target_pose.position.y;
  target_pose1.position.z = target_pose.position.z;
  target_pose1.orientation.x = q1.x();
  target_pose1.orientation.y = q1.y();
  target_pose1.orientation.z = q1.z();
  target_pose1.orientation.w = q1.w();

  target_pose1_1 = target_pose1; //放下夹具目标
  target_pose1_1.position.x = target_pose.position.x + 0.030;
  target_pose1_1.position.y = target_pose.position.y - 0.22;
  target_pose1_1.position.z = target_pose.position.z + 0.113;
  target_pose1_1.orientation.x = q2.x();
  target_pose1_1.orientation.y = q2.y();
  target_pose1_1.orientation.z = q2.z();
  target_pose1_1.orientation.w = q2.w();

  target_pose1_2 = target_pose1_1; //放下目标后向后退，避免碰倒物体
  target_pose1_2.position.x = target_pose1_1.position.x + 0.06;

  target_pose1_2_1 = target_pose1_2; //放下目标后向后退，避免碰倒物体
  target_pose1_2_1.position.x = target_pose1_1.position.x + 0.20;
  target_pose1_2_1.position.z = target_pose1_1.position.z + 0.20;

  target_pose1_3.position.x = target_pose.position.x + 0.03; //吸盘移动到目标上方的位置
  target_pose1_3.position.y = target_pose.position.y + 0.01;
  target_pose1_3.position.z = target_pose.position.z + 0.050;
  target_pose1_3.orientation.x = q1.x();
  target_pose1_3.orientation.y = q1.y();
  target_pose1_3.orientation.z = q1.z();
  target_pose1_3.orientation.w = q1.w();

  target_pose1_4 = target_pose1_3;
  target_pose1_4.position.z = target_pose1_3.position.z - 0.035; //吸盘吸住目标的位置

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

  ROS_INFO("down target:");
   move_group.setMaxVelocityScalingFactor(0.08);     //max velocity
  std::cout << target_pose1_1;
  move_group.setPoseTarget(target_pose1_1);
  move_group.move();

  U_DO_04.request.state = 0;
  client.call(U_DO_04);

  client.call(U_DO_03); //打开夹爪
  sleep(0.7);
  U_DO_03.request.state = 0;
  client.call(U_DO_03);


  move_group.setMaxVelocityScalingFactor(0.06);     //max velocity
  move_group.setPoseTarget(target_pose1_2);
  move_group.move();

  move_group.setPoseTarget(target_pose1_2_1);
  move_group.move();

  ROS_INFO("down gripper...................");
   move_group.setMaxVelocityScalingFactor(0.1);     //max velocity
  move_group.setJointValueTarget(up_gripper_zhua);
  move_group.move();

 move_group.setMaxVelocityScalingFactor(0.06);     //max velocity
  move_group.setJointValueTarget(above_gripper_position_zhua);
  move_group.move();
  sleep(0.5);


  move_group.setMaxVelocityScalingFactor(0.02); //max velocity
  move_group.setJointValueTarget(xie_zhua_position);
  move_group.move();
  client.call(U_DO_00); //Clamp the target


  move_group.setJointValueTarget(above_gripper_position_zhua); //卸爪完成
  move_group.move();
  // sleep(1.0);

  move_group.setMaxVelocityScalingFactor(0.03);  //max velocity
  ROS_INFO("move to above of gripper position"); //装吸盘
  move_group.setJointValueTarget(above_gripper_position_xi);
  move_group.move();
  sleep(1.0);

  U_DO_00.request.state = 1;
  if (client.call(U_DO_00))
  {
    ROS_INFO("U__D0_00: %ld", (long int)U_DO_00.response.success);
    sleep(0.5);
    move_group.setJointValueTarget(fix_gripper_position_xi);
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

  move_group.setJointValueTarget(above_gripper_position_xi);
  move_group.move();

  move_group.setMaxVelocityScalingFactor(0.06); //max velocity
  ROS_INFO("up gripper:");
  move_group.setJointValueTarget(up_gripper_xi);
  move_group.move();
  // sleep(1.0);
  
  move_group.setMaxVelocityScalingFactor(0.15);     //max velocity
  move_group.setPoseTarget(target_pose1_3);
  move_group.move();

  move_group.setMaxVelocityScalingFactor(0.03);     //max velocity
  move_group.setPoseTarget(target_pose1_4);
  move_group.move();

  ROS_INFO("Reach the target point");
  sleep(0.5);
  client.call(U_DO_02); //Clamp the target

  move_group.setPoseTarget(target_pose1_3);
  move_group.move();


  move_group.setMaxVelocityScalingFactor(0.15);     //max velocity
  ROS_INFO("move to target2");
  //move_group.setPoseTarget(target_pose2);
  //move_group.move();
  //move_group.setJointValueTarget(target_pose2);
  //move_group.move();

  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  joint_group_positions[0] = -84 * PI / 180; // radians
  if (joint_group_positions[1] > 0)
  {
    joint_group_positions[1] = joint_group_positions[1] - 35 * PI / 180; // radians
    joint_group_positions[2] = joint_group_positions[2] - 35 * PI / 180; // radians
  }

  if (joint_group_positions[1] < 0)
  {
    joint_group_positions[1] = joint_group_positions[1] + 35 * PI / 180; // radians
    joint_group_positions[2] = joint_group_positions[2] + 35 * PI / 180; // radians
  }
  move_group.setJointValueTarget(joint_group_positions);
  move_group.move();

  ROS_INFO("move to target2");
  // Remove the obstacles
  moveit_msgs::CollisionObject remove_obj1, remove_obj2, remove_obj3, remove_obj4,remove_obj5;
  remove_obj1.id = collision_object.id;
  remove_obj2.id = collision_object2.id;
  remove_obj3.id = collision_object3.id;
  remove_obj4.id = collision_object4.id;
   remove_obj5.id = collision_object5.id;

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

void Receive_move::robot_move3()
{

  int j = 3;
  while (j)
  {
    j--;
    ros::spinOnce();
    sleep(1.0);
  }

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
  const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

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

  std::vector<double> above_gripper_position_xi; //吸盘位姿，应该可以和夹取的起始位置合并，但怕误差暂时不合
  above_gripper_position_xi.push_back(-122.844 * PI / 180);
  above_gripper_position_xi.push_back(-14.099 * PI / 180);
  above_gripper_position_xi.push_back(-113.840 * PI / 180);
  above_gripper_position_xi.push_back(-9.987 * PI / 180);
  above_gripper_position_xi.push_back(-90.089 * PI / 180);
  above_gripper_position_xi.push_back(152.667 * PI / 180);

  std::vector<double> xie_xi_position;
  xie_xi_position.push_back(-122.840 * PI / 180);
  xie_xi_position.push_back(-13.595 * PI / 180);
  xie_xi_position.push_back(-116.080 * PI / 180);
  xie_xi_position.push_back(-12.739 * PI / 180);
  xie_xi_position.push_back(-90.084 * PI / 180);
  xie_xi_position.push_back(152.660 * PI / 180);

  std::vector<double> fix_gripper_position_xi;
  fix_gripper_position_xi.push_back(-122.844 * PI / 180);
  fix_gripper_position_xi.push_back(-13.377 * PI / 180);
  fix_gripper_position_xi.push_back(-116.893 * PI / 180);
  fix_gripper_position_xi.push_back(-13.762 * PI / 180);
  fix_gripper_position_xi.push_back(-90.089 * PI / 180);
  fix_gripper_position_xi.push_back(152.666 * PI / 180);

  std::vector<double> up_gripper_xi; //换吸盘后提升的位置
  up_gripper_xi.push_back(-122.844 * PI / 180);
  up_gripper_xi.push_back(-14.352 * PI / 180);
  up_gripper_xi.push_back(-96.340 * PI / 180);
  up_gripper_xi.push_back(7.765 * PI / 180);
  up_gripper_xi.push_back(-90.089 * PI / 180);
  up_gripper_xi.push_back(152.666 * PI / 180);

  tf::Quaternion q1, q2;
  q1.setRPY(-3.14, 0, 1.57); //吸取的姿态，垂直向下

  geometry_msgs::Pose target_pose1, target_pose1_1, target_pose1_2;

  target_pose1.position.x = target_pose.position.x + 0.025; //放下目标物的位置
  target_pose1.position.y = target_pose.position.y;
  target_pose1.position.z = target_pose.position.z + 0.14;
  target_pose1.orientation.x = q1.x();
  target_pose1.orientation.y = q1.y();
  target_pose1.orientation.z = q1.z();
  target_pose1.orientation.w = q1.w();

  target_pose1_1 = target_pose1;
  target_pose1_1.position.z = target_pose.position.z + 0.14;

  target_pose1_2 = target_pose1_1;
  target_pose1_2.position.z = target_pose1_1.position.z - 0.08;

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

  move_group.setMaxVelocityScalingFactor(0.2); //max velocity
  ROS_INFO("move to target position:");
  std::cout << target_pose1_1;
  move_group.setPoseTarget(target_pose1_1);
  move_group.move();

  move_group.setMaxVelocityScalingFactor(0.06); //max velocity
  std::cout << target_pose1_2;
  move_group.setPoseTarget(target_pose1_2);
  move_group.move();

  ROS_INFO("Reach the target point");
  sleep(0.5);
  U_DO_02.request.state = 0;
  client.call(U_DO_02); //关闭吸气放下物体

  std::cout << target_pose1_1;
  move_group.setPoseTarget(target_pose1_1);
  move_group.move();

  move_group.setMaxVelocityScalingFactor(0.2); //max velocity
  ROS_INFO("down gripper...................");
  move_group.setJointValueTarget(up_gripper_xi);
  move_group.move();
  sleep(0.2);

  move_group.setMaxVelocityScalingFactor(0.035); //max velocity
  move_group.setJointValueTarget(xie_xi_position);
  move_group.move();

  U_DO_00.request.state = 1;
  client.call(U_DO_00); //Clamp the target
  sleep(0.5);

  move_group.setJointValueTarget(above_gripper_position_xi); //卸吸盘完成
  move_group.move();
  U_DO_00.request.state = 0;
  client.call(U_DO_00); //Clamp the target
 // sleep(1.0);

  move_group.setMaxVelocityScalingFactor(0.2);     //max velocity
  ROS_INFO("Return to HOME position...................");
  move_group.setJointValueTarget(home_position);
  move_group.move();
  sleep(1.0);

  moveit_msgs::CollisionObject remove_obj1, remove_obj2, remove_obj3, remove_obj4, remove_obj5;
  remove_obj1.id = collision_object.id;
  remove_obj2.id = collision_object2.id;
  remove_obj3.id = collision_object3.id;
  remove_obj4.id = collision_object4.id;
    remove_obj5.id = collision_object5.id;

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

ros::Publisher cmdVelPub;
ros::Publisher marker_pub;

void shutdown(int sig)
{
  cmdVelPub.publish(geometry_msgs::Twist());
  ros::Duration(1).sleep(); // sleep for  a second
  ROS_INFO("nav_square.cpp ended!");
  ros::shutdown();
}

void init_markers(visualization_msgs::Marker *marker)
{
  marker->ns = "waypoints";
  marker->id = 0;
  marker->type = visualization_msgs::Marker::CUBE_LIST;
  marker->action = visualization_msgs::Marker::ADD;
  marker->lifetime = ros::Duration(); //0 is forever
  marker->scale.x = 0.2;
  marker->scale.y = 0.2;
  marker->color.r = 1.0;
  marker->color.g = 0.7;
  marker->color.b = 1.0;
  marker->color.a = 1.0;

  marker->header.frame_id = "odom";
  marker->header.stamp = ros::Time::now();
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "nav_move_base");
  ros::NodeHandle node;

  //Subscribe to the move_base action server
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

  //Define a marker publisher.
  marker_pub = node.advertise<visualization_msgs::Marker>("waypoint_markers", 10);

  //for init_markers function
  visualization_msgs::Marker line_list;

  signal(SIGINT, shutdown);
  ROS_INFO("move_base_square.cpp start...");

  //Create a list to hold the target quaternions (orientations)
  geometry_msgs::Quaternion quaternions[4];

  //convert the angles to quaternions
  quaternions[0] = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0.016);
  quaternions[1] = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -1.570);
  quaternions[2] = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 3.14);
  quaternions[3] = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

  //a pose consisting of a position and orientation in the map frame.
  geometry_msgs::Point point;
  geometry_msgs::Pose pose_list[4];

  point.x = 2.400;
  point.y = 0.600;
  point.z = 0.0;
  pose_list[0].position = point;
  pose_list[0].orientation = quaternions[0];

  point.x = 2.40;
  point.y = -2.260;
  point.z = 0.0;
  pose_list[1].position = point;
  pose_list[1].orientation = quaternions[1];

  point.x = 0.160;
  point.y = -2.260;
  point.z = 0.0;
  pose_list[2].position = point;
  pose_list[2].orientation = quaternions[2];

  point.x = 0.00;
  point.y = 0.00;
  point.z = 0.0;
  pose_list[3].position = point;
  pose_list[3].orientation = quaternions[3];

  //Initialize the visualization markers for RViz
  init_markers(&line_list);

  //Set a visualization marker at each waypoint
  for (int i = 0; i < 4; i++)
  {
    line_list.points.push_back(pose_list[i].position);
  }

  //Publisher to manually control the robot (e.g. to stop it, queue_size=5)
  cmdVelPub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 6);

  ROS_INFO("Waiting for move_base action server...");
  //Wait 60 seconds for the action server to become available
  if (!ac.waitForServer(ros::Duration(60)))
  {
    ROS_INFO("Can't connected to move base server");
    return 1;
  }
  ROS_INFO("Connected to move base server");
  ROS_INFO("Starting navigation test");

  //Update the marker display
  marker_pub.publish(line_list);

  //Intialize the waypoint goal
  move_base_msgs::MoveBaseGoal goal;

  //Use the map frame to define goal poses
  goal.target_pose.header.frame_id = "map";

  //Set the time stamp to "now"
  goal.target_pose.header.stamp = ros::Time::now();

  //Set the goal pose to the i-th waypoint
  goal.target_pose.pose = pose_list[0];

  //Start the robot moving toward the goal
  //Send the goal pose to the MoveBaseAction server
  ac.sendGoal(goal);

  //Allow 1 minute to get there
  bool finished_within_time = ac.waitForResult(ros::Duration(180));

  //If we dont get there in time, abort the goal
  if (!finished_within_time)
  {
    ac.cancelGoal();
    ROS_INFO("Timed out achieving goal");
  }
  else
  {
    //We made it!
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Goal A succeeded!");
    }
    else
    {
      ROS_INFO("The Goal A failed for some reason");
      ros::shutdown();
    }
  }

  //Manipulator action A
  Receive_move move1;
  ROS_INFO("arm action 1 begining");
  move1.robot_move1();
  ROS_INFO("arm action 1 begin end");

  //AGV moves to point B
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose = pose_list[1];
  ac.sendGoal(goal);
  bool finished_within_time2 = ac.waitForResult(ros::Duration(180));
  if (!finished_within_time2)
  {
    ac.cancelGoal();
    ROS_INFO("Timed out achieving goal");
  }
  else
  {
    //We made it!
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Goal B succeeded!");
    }
    else
    {
      ROS_INFO("The Goal B failed for some reason");
      ros::shutdown();
    }
  }

  //Manipulator action B
  ROS_INFO("arm action 2 begining");
  move1.robot_move2();
  ROS_INFO("arm action 2  end");

  //AGV moves to point C
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose = pose_list[2];
  ac.sendGoal(goal);
  bool finished_within_time3 = ac.waitForResult(ros::Duration(180));
  if (!finished_within_time3)
  {
    ac.cancelGoal();
    ROS_INFO("Timed out achieving goal");
  }
  else
  {
    //We made it!
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Goal C succeeded!");
    }
    else
    {
      ROS_INFO("The Goal C  failed for some reason");
      ros::shutdown();
    }
  }

  //Manipulator action C
  ROS_INFO("arm action 2 begining");
  move1.robot_move3();
  ROS_INFO("arm action 3 end");

  //AGV return to origin
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose = pose_list[3];
  ac.sendGoal(goal);
  bool finished_within_time4 = ac.waitForResult(ros::Duration(180));
  if (!finished_within_time4)
  {
    ac.cancelGoal();
    ROS_INFO("Timed out achieving goal");
  }
  else
  {
    //We made it!
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Goal 4 succeeded!");
    }
    else
    {
      ROS_INFO("The Goal 4 failed for some reason");
      ros::shutdown();
    }
  }

  return 0;
}
