#!/bin/sh
echo "Add Markers"
echo "Make sure you have sourced your devel/setup.bash file"


xterm  -e  "roslaunch turtlebot_gazebo turtlebot_gazebo.launch" &
sleep 5

xterm  -e  "roslaunch turtlebot_gazebo amcl_demo.launch" & 
sleep 5

xterm  -e  "roslaunch turtlebot_rviz_launchers view_markers.launch" & 
sleep 5

xterm -e "rosrun add_markers add_markers"
