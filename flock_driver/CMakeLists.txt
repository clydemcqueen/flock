cmake_minimum_required(VERSION 2.8.3)
project(flock_driver)

## Find catkin and everything we need in ROS
find_package(catkin REQUIRED COMPONENTS
  flock_msgs
  geometry_msgs
  rospy
)

## Declare a catkin package and export dependencies
catkin_package(
  CATKIN_DEPENDS
  flock_msgs
  geometry_msgs
  rospy
)
