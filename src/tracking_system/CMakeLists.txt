cmake_minimum_required(VERSION 2.8.3)
project(autoweed_tracking_core)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  custom_messages
  tf2_ros
  tf2
)

## Declare ROS messages and services
# add_message_files(
#   FILES
#   Target.msg
# )

## Generate added messages and services
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS
  # message_runtime
  custom_messages
  tf2
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

# add_executable(talker src/talker.cpp)
# target_link_libraries(talker ${catkin_LIBRARIES})
# add_dependencies(talker beginner_tutorials_generate_messages_cpp)

# add_executable(listener src/listener.cpp)
# target_link_libraries(listener ${catkin_LIBRARIES})
# add_dependencies(listener beginner_tutorials_generate_messages_cpp)

add_executable(tracking_core src/tracking.cpp)
target_link_libraries(tracking_core ${catkin_LIBRARIES})
add_dependencies(tracking_core ${catkin_EXPORTED_TARGETS})

add_executable(target_test_program src/target_tester.cpp)
target_link_libraries(target_test_program ${catkin_LIBRARIES})
add_dependencies(target_test_program ${catkin_EXPORTED_TARGETS})
