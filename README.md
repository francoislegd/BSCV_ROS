# BSCV_ROS
BSCV Project in Robotics using ROS, Turtlebot and Phantom-X Pincher arm



# INTRODUCTION

### Description of the project
This report will describe in details the project given during Bachelor in Computer Vision's second semester in Robotics. In the given scenario we had to move a Turtlebot to a target position while avoiding obstacles, then put an object on it by performing a pick and place action with a Phantom-X arm before making the turtlebot return to its original place with the object. The report will focus on every aspects of the project, from the available material and code to the final implementation.    


### What is ROS ?
ROS (Robot Operating System) is an open source middleware that allows its users to perform robot programming. As middleware, it possess characteristics of both software and hardware, since it is able to perform various actions like hardware abstraction and low level controls. However this characteristic is of a great interest since it brings important tasks such as the creation of new commandlines or package management, which have proved to be essential tools for the realization of the project. Different versions of ROS exists with some differences, so for compatibility reasons we are using the Indigo realease. (insert ROS logo)


ROS presents the advantage to support multiple languages implementations such as roscpp for C++ or rospy for Python, as well as specific libraries in these languages, so one can write actions called "nodes" and then integrate them inside a package with some flexibility. However one of the drawback of ROS is that it is officially supported only on Linux Ubuntu 14.04 (for the indigo version) even if the community built around supports other platforms like MAC OS or Windows. This is due to the reason that the libraries tools have a linux oriented implementation and will not work with another exploiting system. 
Other various actions can be performed with sets of nodes called "packages" such as graphic visualization or network connexions (between active nodes). (insert graph image)

### Robots used for the scenario

#### Turtlebot Description
The mobile robot used in the Project is a Turtlebot, created in 2010 at Willow Garage by Melonee Wise and Tully Foote. Here we used the turtlevot version 2 , similar to version 1 but with a better base. It use the kobuki base built by Yujinrobot (a  
(insert turtlebot pic)


#### Phantom-X description
4 servo-motors plus a gripper to grab the object. assembled by previous year students.
(insert arm pic)


### Scenario description
Move the turtlebot to a table while avoiding obstacles, take a cube with the arm, put it on the turtlebot, then go back to the original position.


# REPOSITORY CONTENT

### File list
List of all the files and their dependencies in this repository

### Original Code links
Links to original, unmodified files packages

### Information links
Links to the differents sections of this readme


# IMPLEMENTATION

### Setup Material
Turtlebot, Phantom-X pincher arm, ROS indigo, Computer as workstation, Lidar system

### Starting bases

#### Packages
Turtlebo_bringup, Turtlebot_navigation, turtlebot_arm, Turtlebot_arm_moveit_demos

#### Executable scripts
go to a specific position.py, pick_and_place.py

### Original Commandlines and Topics

#### Turtlebot

#### Phantom-X



# MODIFICATIONS

### Scripts of interest

#### Turtlebot
global costmap common param.yaml, local costmap common param.yaml, global common params.yaml,

#### Pick And Place
turtlebot_arm_moveit_demos/bin/pick_and_place.py

### Building a map

### New Packages
Our_project Package
Designed to be launchd on the turtlebot, contains one launch file that set up the rplidar
Project_ws Package
Designed to be launched from the workstation, for both turtlebot and Robotic arm 

### Launch files Description

#### Our project Package
Turtlebot_project.launch for setting up the lidar on the turtlebot, needs to be rerun for path planning

#### Project_ws Package
Project_WS.launch for entering set up coordinates and path plannification, from origin point to target destination.
Robotic_arm.launch, for activating the pick and place action.
Go_back.launch for entering target destination's coordinates as a starting point, to another point with coordinates that ease the next step.
Go_back2.launch, for entering previous point as starting coordinates and origin point as target destination.


# INSTRUCTIONS TO RUN THE PROCESS

### Download and Install

### Building

### Commands




# RESULT

# PROBLEMS ENCOUNTERED

### On the Turtlebot

### On the Robotic arm



# POINTS OF IMPROVEMENTS



# CONCLUSION



# REFERENCES

