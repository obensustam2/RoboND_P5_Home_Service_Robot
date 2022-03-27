cmake_minimum_required(VERSION 3.0.2)
project(pick_objects)

find_package(catkin REQUIRED COMPONENTS roscpp rospy actionlib move_base_msgs std_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(pick_objects src/pick_objects.cpp)
target_link_libraries(pick_objects ${catkin_LIBRARIES})
