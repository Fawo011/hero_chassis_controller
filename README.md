# Package Name

## Overview

This is a hero chassis controller used to control the rm_hero robot.

**Keywords:** RobotMaster, hero_chassis_controller, controller


### License

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: Fawool(Mingliang Liu)<br />
Affiliation: GDUT DynamicX)<br />
Maintainer: Fawool, LMLSDSG@163.com**

The hero_chassis_controller package has been tested under [ROS]  Noetic on respectively Ubuntu 
20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.



## Installation

### Installation from Packages

To install all packages from the this repository as Debian packages use

    sudo apt-get install ros-noetic-...

Or better, use `rosdep`:

	sudo rosdep install --from-paths src

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [rm_description](https://github.com/gdut-dynamic-x/rm_description)
- controller_interface
- forward_command_controller
- hardware_interface
- control_toolbox
- geometry_msgs
- control_msgs
- realtime_tools
- tf
- nav_msgs
- pluginlib
- teleop_twist_joy(search in github)

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package
using

	cd catkin_workspace/src
	git clone https://github.com/Fawo011/hero_chassis_controller.git (or git@github.com:Fawo011/hero_chassis_controller.git)
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin build



## Usage

Describe the quickest way to run this software, for example:

Run the main node with

	roslaunch hero_chassis_controller load_controller.launch

## Config files

Config file folder /config

* **controller.yaml** Shortly explain the content of this config file



## Launch files

* **load_controller.launch:** shortly explain what is launched (e.g standard simulation, simulation with gdb,...)



## Nodes

### controller_spawner

Spawn the controller 

#### Subscribed Topics

* **`/cmd_vel`** ([sensor_msgs/Temperature])

  The temperature measurements from which the average is computed.

#### Published Topics

...



## Bugs & Feature Requests

Please report bugs and request features using the https://github.com/Fawo011/hero_chassis_controller
