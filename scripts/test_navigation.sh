
export TURTLEBOT_GAZEBO_WORLD_FILE=/home/workspace/catkin_ws/src/turtlebot_simulator/turtlebot_gazebo/worlds/playground.world
export TURTLEBOT_GAZEBO_MAP_FILE=/home/workspace/catkin_ws/src/turtlebot_simulator/turtlebot_gazebo/maps/playground.yaml

echo "Test Navigation"
echo "Make sure you have sourced your devel/setup.bash file"
xterm  -e  "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch"
