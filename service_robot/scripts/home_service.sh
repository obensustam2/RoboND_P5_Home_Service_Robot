#!/usr/bin/env bash

. /opt/ros/melodic/setup.sh
. $HOME/RoboND_P5_Home_Service_Robot/devel/setup.sh

sleep 3

echo "Starting Roscore"
gnome-terminal --tab -- bash -c "roscore"
sleep 3

echo "Starting Turtlebot3 World"
gnome-terminal --tab -- bash -c "roslaunch  service_robot world.launch"
sleep 3

echo "Starting Navigation"
gnome-terminal --tab -- bash -c "roslaunch service_robot navigation.launch"
sleep 3

echo "Starting Rviz"
gnome-terminal --tab -- bash -c "roslaunch service_robot view_navigation.launch"
sleep 4

echo "Starting Pick Objects"
gnome-terminal --tab -- bash -c "rosrun pick_objects pick_objects"

echo "Starting Add Markers"
gnome-terminal --tab -- bash -c "rosrun add_markers add_markers"
