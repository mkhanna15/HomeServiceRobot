# Home-Service-Robot
### Project Purpose
This project simulates an autonomous robot that picks up an object and drops it off at predetermined locations. 

### Packages Used:
#### Ros Packages
* [slam_gmapping](https://github.com/ros-perception/slam_gmapping)
  * This package creates a map of the environment using laser-based SLAM.
* [turtlebot](https://github.com/turtlebot/turtlebot)
  * Used turtlebot_teleop  to manually drive the robot from the keyboard in some scripts file.
* [turtlebot_interactions](https://github.com/turtlebot/turtlebot_interactions)
   * This package is used to launch RViz which allows the user to visualize SLAM, particle filters and send nav goals. Here RViz config files can be saved which will launch the map, robot model, markers etc.
* [turtlebot_simulator](https://github.com/turtlebot/turtlebot_simulator)
  * This package contains the turtlebot_gazebo file which allows you to launch the turtlebot world  with the turtlebot(turtlebot_world.launch) and launches the amcl algorithm (amcl_demo.launch). The robot is able to localize itself using the amcl package. This launch file is modified to specify the initial robot pose. The orientation is changed to what is seen by Rviz. Here is the change:
```
<arg name="initial_pose_x" default="0.0"/>
<arg name="initial_pose_y" default="0.0"/>
<arg name="initial_pose_a" default="-1.57"/>
```

#### Packages created by me
* [pick_objects](https://github.com/mkhanna15/HomeServiceRobot/tree/master/pick_objects)
  * This package sends pick up and drop off goals to the robot which then uses the ROS navigation stack to reach the goal. Pick_objects package drives the navigation of the robot.
* [add_markers](https://github.com/mkhanna15/HomeServiceRobot/tree/master/add_markers)
  * This package publishes a virtual object in Rviz that appears in the pick up goal location. Once the robot reaches the pick up location the object disappears and the robot is then directed to the drop off goal location by the pick_objects package. Once the robot reaches the drop off the goal location then the object appears. 

### Scripts
* ./test_slam.sh
   * allows user to teleop the robot and have it interface with SLAM to visualize the map in RViz
* ./test_navigation.sh
  * users can give a direction to the robot with 2D nav goal in rviz and using the ROS navigation stack it will localize itself and reach the goal.
* ./pick_objects.sh
  * robot drives to pick up location and drop off location.
* ./add_markers/sh
  * marker appears at pick up location waits 5 secs then appears at drop off location
* ./home_service/sh
  * robot autonomously drivers to pick up location picks up object and the drives to drop off location and drops off object. 

### To use this Repo:

First create a catkin workspace
[See catkin tutorial](http://wiki.ros.org/ROS/Tutorials/catkin/CreateWorkspace)

To launch the home service, type in these commands in the catkin workspace. 
```
> catkin_make
> source devel/setup.bash
> ./src/scripts/home_service.sh
```

The home service script
```
echo "Home Service"
echo "Make sure you have sourced your devel/setup.bash file"

export TURTLEBOT_GAZEBO_MAP_FILE="$(rospack find turtlebot_gazebo)/maps/MyWorld.yaml"
export TURTLEBOT_GAZEBO_WORLD_FILE="$(rospack find turtlebot_gazebo)/worlds/MyWorld.world"

xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 7
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch " &
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 7
xterm  -e  " rosrun pick_objects pick_objects " &
sleep 5
xterm  -e  " rosrun add_markers add_markers "
```

Launches the:
* gazebo world (provided my own world)
* AMCL package that localizes the robot (provided map of my world)
* RViz and rviz config file
* pick_objects and add_markers package
