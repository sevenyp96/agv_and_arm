cmake_minimum_required(VERSION 2.8.3)
project(aubo_demo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)



if("$ENV{ROS_DISTRO}" STREQUAL "kinetic")
    find_package(catkin REQUIRED COMPONENTS
      roscpp
      rospy
      std_msgs
      moveit_core
      moveit_visual_tools
      moveit_ros_planning
      moveit_ros_planning_interface
      pluginlib
      geometric_shapes
      pcl_ros
      pcl_conversions
      rosbag
      tf
      tf2_ros
      #tf2_eigen
      tf2_geometry_msgs
    )
    Message(STATUS "<<<<<<<<<<<<<<<<<<<<<<<<<Note: Find Package for kinetic")
endif()

if("$ENV{ROS_DISTRO}" STREQUAL "indigo")
    find_package(catkin REQUIRED COMPONENTS
      roscpp
      rospy
      std_msgs
      moveit_core
      moveit_ros_planning
      moveit_ros_planning_interface
      pluginlib
      geometric_shapes
      pcl_ros
      pcl_conversions
      rosbag
      tf
      tf2_ros
      #tf2_eigen
      tf2_geometry_msgs
    )
    Message(STATUS "<<<<<<<<<<<<<<<<<<<<<<<<<Note: Find Package for  for indigo")
endif()





###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aubo_demo
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



if("$ENV{ROS_DISTRO}" STREQUAL "kinetic")
    ## Declare a C++ executable
    ## With catkin_make MoveGroupInterface_To_Kinetic package is built within a single CMake context
    ## The recommended prefix ensures that target names across packages don't collide
    add_executable(MoveGroupInterface_To_Kinetic src/MoveGroupInterface_To_Kinetic.cpp)
    add_executable(AuboTest_kinetic src/AuboTest_kinetic.cpp)
    add_executable(Test_Aubokinematic src/AuboTest_kinematic_kinetic.cpp)

    ## Specify libraries to link a library or executable target against
    target_link_libraries(MoveGroupInterface_To_Kinetic ${catkin_LIBRARIES} ${Boost_LIBRARIES})
    target_link_libraries(AuboTest_kinetic ${catkin_LIBRARIES} ${Boost_LIBRARIES})
    target_link_libraries(Test_Aubokinematic ${catkin_LIBRARIES} ${Boost_LIBRARIES})

    Message(STATUS "<<<<<<<<<<<<<<<<<<<<<<<<<Note:build aubo_demo node for kinetic")
endif()

add_executable(move_text_node src/move_text.cpp)
target_link_libraries(move_text_node  ${catkin_LIBRARIES})

add_executable(robot_move_node src/robot_move.cpp)
target_link_libraries(robot_move_node  ${catkin_LIBRARIES})

add_executable(move2_node src/move2.cpp)
target_link_libraries(move2_node  ${catkin_LIBRARIES})


add_executable(IO_test1_node src/IO_test1.cpp)
target_link_libraries(IO_test1_node  ${catkin_LIBRARIES})

add_executable(move3_add_gripper_node src/move3_add_gripper.cpp)
target_link_libraries(move3_add_gripper_node  ${catkin_LIBRARIES})

add_executable(move4_not_sub_node src/move4_not_sub.cpp)
target_link_libraries(move4_not_sub_node  ${catkin_LIBRARIES})

add_executable(arm_and_agv_node src/arm_and_agv.cpp)
target_link_libraries(arm_and_agv_node  ${catkin_LIBRARIES})

add_executable(test_install_gripper_node src/test_install_gripper.cpp)
target_link_libraries(test_install_gripper_node  ${catkin_LIBRARIES})

add_executable(test_gripping_target_node src/test_gripping_target.cpp)
target_link_libraries(test_gripping_target_node  ${catkin_LIBRARIES})

add_executable(one_setp_node src/one_setp.cpp)
target_link_libraries(one_setp_node  ${catkin_LIBRARIES})

add_executable(test_agv_and_arm_node src/test_agv_and_arm.cpp)
target_link_libraries(test_agv_and_arm_node  ${catkin_LIBRARIES})

add_executable(one_setp_zhua_node src/one_setp_zhua.cpp)
target_link_libraries(one_setp_zhua_node  ${catkin_LIBRARIES})

add_executable(step_one_node src/step_one.cpp)
target_link_libraries(step_one_node  ${catkin_LIBRARIES})

add_executable(step_two_node src/step_two.cpp)
target_link_libraries(step_two_node  ${catkin_LIBRARIES})

add_executable(step_three_node src/step_three.cpp)
target_link_libraries(step_three_node  ${catkin_LIBRARIES})

add_executable(step_all_node src/step_all.cpp)
target_link_libraries(step_all_node  ${catkin_LIBRARIES})

if("$ENV{ROS_DISTRO}" STREQUAL "indigo")
    ## Declare a C++ executable
    ## With catkin_make MoveGroupInterface_To_Kinetic package is built within a single CMake context
    ## The recommended prefix ensures that target names across packages don't collide
    add_executable(MoveGroupInterface_To_Indigo src/MoveGroupInterface_To_Indigo.cpp)

    ## Specify libraries to link a library or executable target against
    target_link_libraries(MoveGroupInterface_To_Indigo ${catkin_LIBRARIES} ${Boost_LIBRARIES})

    Message(STATUS "<<<<<<<<<<<<<<<<<<<<<<<<<Note:build aubo_demo node for indigo")
endif()





