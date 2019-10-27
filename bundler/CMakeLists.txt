cmake_minimum_required(VERSION 2.8.3)
project(bundler)

find_package(catkin REQUIRED
  COMPONENTS
  audio_features
  message_generation
  std_msgs
)

add_message_files(DIRECTORY msg
  FILES
  Bundle.msg
)

generate_messages(DEPENDENCIES
  audio_features
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  audio_features
  grouper
  message_runtime
  rosbag
  rospy
  rospy_wrapper
  std_msgs
)
