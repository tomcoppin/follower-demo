cmake_minimum_required(VERSION 2.8.3)
project(follower_demo)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs message_generation cv_bridge image_transport
sensor_msgs)

find_package(OpenCV REQUIRED)

## Add in message and service files
add_message_files(FILES raven_automove.msg raven_state.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS})

add_executable(follower src/talker.cpp  src/Raven_Controller.cpp src/Raven_PathPlanner.cpp)
target_link_libraries(follower ${catkin_LIBRARIES})
add_dependencies(follower AutoCircle_test_generate_messages_cpp)

add_executable(listenerfollower src/listener.cpp)
target_link_libraries(listenerfollower ${catkin_LIBRARIES}
${OpenCV_LIBRARIES})
add_dependencies(listenerfollower AutoCircle_test_generate_messages_cpp)
