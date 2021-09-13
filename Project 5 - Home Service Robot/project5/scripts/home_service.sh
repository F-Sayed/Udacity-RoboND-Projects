#!/bin/sh
terminator -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
terminator -e "roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
terminator -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
terminator -e "rosrun add_markers add_markers" &
sleep 10
terminator -e "rosrun pick_objects pick_objects"