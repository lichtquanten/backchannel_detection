cmake_minimum_required(VERSION 2.8.3)
project(model)

find_package(catkin REQUIRED
  COMPONENTS
  message_generation
  std_msgs
)

catkin_python_setup()

add_message_files(DIRECTORY msg
  FILES
  Backchannel.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  bundler
  rospy
  rospy_wrapper
)
