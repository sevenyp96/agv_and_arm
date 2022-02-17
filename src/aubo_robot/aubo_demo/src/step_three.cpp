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
  void robot_move3();
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

  //定义一个PlanningScene消息
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(collision_object);
  planning_scene.world.collision_objects.push_back(collision_object2);
  planning_scene.world.collision_objects.push_back(collision_object3);
  planning_scene.world.collision_objects.push_back(collision_object4);
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

  moveit_msgs::CollisionObject remove_obj1, remove_obj2, remove_obj3, remove_obj4;
  remove_obj1.id = collision_object.id;
  remove_obj2.id = collision_object2.id;
  remove_obj3.id = collision_object3.id;
  remove_obj4.id = collision_object4.id;

  remove_obj1.operation = remove_obj1.REMOVE;
  remove_obj2.operation = remove_obj2.REMOVE;
  remove_obj3.operation = remove_obj3.REMOVE;
  remove_obj4.operation = remove_obj4.REMOVE;

  planning_scene.world.collision_objects.clear();

  planning_scene.world.collision_objects.push_back(remove_obj1);
  planning_scene.world.collision_objects.push_back(remove_obj2);
  planning_scene.world.collision_objects.push_back(remove_obj3);
  planning_scene.world.collision_objects.push_back(remove_obj4);
  //发布消息
  planning_scene_diff_publisher.publish(planning_scene);

  //ros::shutdown();
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "auto_move_node2");
  ;
  Receive_move move1;
  bool s;

  int i = 3;
  while (i)
  {
    ROS_INFO("sleep %ds", i);
    sleep(1);
    --i;
  }
  move1.robot_move3();
  ROS_INFO("robot move end");
  return 0;
}
