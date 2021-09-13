#!/bin/sh
terminator -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
terminator -e "roslaunch turtlebot_gazebo gmapping_demo.launch" & 
sleep 5
terminator -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" 
