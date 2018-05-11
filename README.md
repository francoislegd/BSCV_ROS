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
The mobile robot used in the Project is a Turtlebot. The original design was created in 2010 at Willow Garage by Melonee Wise and Tully Foote. Here we used the turtlevot version 2 , similar to version 1 but with a better base. It uses the kobuki base built by Yujinrobot (a South Korean firm originally based on conception of vacuum cleaner robots), an Asus laptop with ROS indigo installed (using Ubuntu 14.04), and an embedded Kinect V1 camera in order to perform obstacle avoidance. It's autonomy of roughly 3 hours also makes it very useful and versatile since it allows operations or modifications to be done whithout time constraint.  
(insert turtlebot pic)

As seen on the previous image, a modification was performed on the Robot by previous students. In addition to the Kinnect camera, a Lidar system has been added as an alternate way to perform obstacle avoidance and mapping. As it is easier to implement, meaning in particular less topics and associated parameters to take into account than with the Kinnect, we had to use it to implement the given scenario. 

#### Phantom-X description
The second system we had to program is a Phantom-X pincher robotic arm. Its conception includes 4 servo-motors plus a gripper to grab objects, giving it 4 degrees of freedom in space. In the project we used it in order to move a cube from a table to a The turtlebot.
(insert arm pic)


### Scenario description
Our goal was to complete the following scenario: 

**1.** Move the turtlebot to a table while avoiding obstacles (insert scenario pics for each)

**2.** take a cube with the arm and put it on the turtlebot

**3.** then go back to another position while transporting the cube (here it will be the departure point in 1.).

**These steps should be performed semi-autonomously, so that the user has to write in the terminal a single command for each action to perform.** 

# REPOSITORY CONTENT

### File list
Here is the list of all the files and dependancies contained in this repository we needed to build the project:
(List of all the files and their dependencies in this repository)

### Original Code links
Here are some links to some raw material we further had to modify :
Links to original, unmodified external files packages :

Packages for the Phantom-X robotic arm [arbotix & turtlebot_arm](https://github.com/NathanCrombez/PhantomXPincherArmROS) 

Packages useful to make the Lidar device work, especially [turtlebot_le2i](https://github.com/roboticslab-fr/rplidar-turtlebot2) 

### Information links
Links differents sections in the readme file for more detailed explanations :

[INTRODUCTION](#INTRODUCTION)



# ORIGINAL CONTENT
Here we will describe the original content we had the access prior to its implementation in the project.

### Setup Material
We had to start with the following material : 
- Turtlebot2
- Phantom-X pincher arm
- ROS indigo (and Ubuntu 14.04) 
- Computer as workstation
- Lidar device (will put links to the intro)

### Starting bases
Here we will discuss about the original files and actions before the modifications

#### Packages
For this project, some packages were already installed in order to teleoperate the turtlebot : OMPL, AMCL, Move_Base and Turtlebot Navigation. The original suggestions carried by our preliminary survey were respected since we were requiring the use of the OMPL, AMCL and Move_Base packages.

Some necessary packages however needed to be installed in order to activate the Lidar device on the turtlebot and to teleoperate the robotic arm. They were present in the  rplidar-turtlebot2-indigo meta-package (especially the turtlebot_le2i package); and as arbotix and robotic_arm packages for the Phantom-X. The robotic_arm package also respect the preliminar survey since most of its sub-packages are derivated from the suggested Move_it package.

Turtlebot_bringup, Turtlebot_navigation, turtlebot_arm, Turtlebot_arm_moveit_demos

#### Executable scripts
go to a specific position.py, pick_and_place.py

### Original Commandlines and Topics

#### Turtlebot
Package Rplidar installation after downloading:

'scp rlidar-turtlebot2-indigo.zip turtlebot@192.0.100:/'


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

