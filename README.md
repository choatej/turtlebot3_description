# Turtlebot3 Robot Description in ROS2/RVIZ2

![Build Status](https://github.com/choatej/turtlebot3_description/actions/workflows/ci.yml/badge.svg)
![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)
![ROS2 Distro](https://img.shields.io/badge/ROS2-Foxy-blue)
![GitHub version](https://img.shields.io/github/v/tag/choatej/turtlebot3_description?label=version)

This project is adapting the ROS1 turtlebot project and moving parts of it to ROS2

## License Notice

This project is licensed under the [MIT License](LICENSE). However, the files in the `meshes` and `urdf` directories were taken from the [TurtleBot3 project](https://github.com/ROBOTIS-GIT/turtlebot3) and are unmodified. These files are used under the [Apache 2.0 License](http://www.apache.org/licenses/LICENSE-2.0). No modifications have been made to these files.

For more information about the Apache 2.0 License, see the [APACHE-2.0-LICENSE](APACHE-2.0-LICENSE) file in this repository.


## What this project provides

1. URDF/xacro model for the turtlebot3 burger
1. A simple rviz config file for viewing the turtlebot
1. A very basic world for the robot to exist in
1. A launch configuration that will spawn the robot into rviz and into the aformentioned world in gazebo

## Quickstart / I want to see the robot!

This guide assumes that you
1. have ROS2 iron installed - this has not been tested on other versions but it should work
1. have Gazebo 2 installed
1. have rviz2 installed
1. have a ROS2 workspace already

To get and execute the code:
1. clone this repository into a ROS2 workspace
```bash
cd ~/ros2_ws/src # or wherever your ROS2 project lives
git clone https://github.com/choatej/turtlebot3_description.git
```

1. Navigate back to the workspace root and build it
```bash
cd ~/ros2_ws # or wherever your ROS2 workspace is
source /opt/ros/iron/setup.bash
colcon_build
```

1. Source your environment and launch!
```bash
source ~/ros2_ws/install/setup.bash # or wherever you just built from
ros2 launch turtlebot3_description turtlebot3_burger.launch.py # tab completion can help here
```

You should see the robot in both rviz and gazebo.  At that point, if you want to move the robot around the world, in a second console run:
```bash
source /opt/ros/iron/setup.bash # or your ROS2 distro's setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
An it should provide you with a keyboard interface for controlling the robot's motion.

## But I don't have ROS2 iron installed. What about me?
If you do not have ROS2 installed and don't care to install it, there is still hope!
You can build a docker image that will do all the work for you.  You do have docker installed don't you?

```bash
docker build -t choatej/turtlebot3_description:local .
docker-compose up
```
And this will start the container which will run both rviz and gazebo for you.