# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(realsense_ar_tracking)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs ar_track_alvar_msgs genmsg tf geometry_msgs)

## Declare ROS messages and services
#add_message_files(FILES AlvarMarker.msg AlvarMarkers.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs ar_track_alvar_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(tracker src/tracker.cpp)
target_link_libraries(tracker ${catkin_LIBRARIES})

add_executable(tf_listener src/tf_listener.cpp)
target_link_libraries(tf_listener ${catkin_LIBRARIES})

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})


# %EndTag(FULLTEXT)%
