#!/bin/sh
echo "Test SLAM"
echo "Make sure you have sourced your devel/setup.bash file"

xterm  -e  "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e "rosrun teleop_twist_keyboard teleop_twist_keyboard.py"
