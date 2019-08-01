cmake_minimum_required(VERSION 2.8.3)
project(audio_features)

find_package(catkin REQUIRED
  COMPONENTS
  message_generation
  std_msgs
)

add_message_files(DIRECTORY msg FILES
  AudioFeatures.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS
  audio_io_msgs
  grouper
  message_runtime
  rosbag
  rospy
  rospy_wrapper
  std_msgs
)
