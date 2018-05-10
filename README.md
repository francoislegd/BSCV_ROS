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
The mobile robot used in the Project is a Turtlebot, created in 2010 at Willow Garage by Melonee Wise and Tully Foote. 
(insert turtlebot pic)


#### Phantom-X description
4 servo-motors plus a gripper to grab the object. assembled by previous year students.
(insert arm pic)


### Scenario description
Move the turtlebot to a table while avoiding obstacles, take a cube with the arm, put it on the turtlebot, then go back to the original position.


# REPOSITORY CONTENT

### File list

### Original Code links

### Information links


# IMPLEMENTATION

### Setup Material

### Starting bases

#### Packages

#### Executable scripts


### Original Commandlines and Topics

#### Turtlebot

#### Phantom-X



# MODIFICATIONS

### Scripts of interest

#### Turtlebot

#### Pick And Place


### New Packages

#### Our_project Package

#### Project_ws Package


### Launch files Description



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

