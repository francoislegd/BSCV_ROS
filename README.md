# BSCV_ROS
BSCV Project in Robotics using ROS, Turtlebot and Phantom-X Pincher arm



# INTRODUCTION

### Description of the project
This report will describe in details the project given during Bachelor in Computer Vision's second semester in Robotics. In the given scenario we had to move a Turtlebot to a target position while avoiding obstacles, then put an object on it by performing a pick and place action with a Phantom-X arm before making the turtlebot return to its original place with the object. The report will focus on every aspects of the project, from the available material and code to the final implementation.    


### What is ROS ?
ROS (Robot Operating System) is an open source middleware that allows its users to perform robot programming. As middleware, it possess characteristics of both software and hardware, since it is able to perform various actions like hardware abstraction and low level controls. However this characteristic is of a great interest since it brings important tasks such as the creation of new commandlines or package management, which have proved to be essential tools for the realization of the project. Different versions of ROS exists with some differences, so for compatibility reasons we are using the Indigo realease. 
![ROS logo](https://github.com/francoislegd/BSCV_ROS/blob/master/Pictures/indigoigloo_600.png)


ROS presents the advantage to support multiple languages implementations such as roscpp for C++ or rospy for Python, as well as specific libraries in these languages, so one can write actions called "nodes" and then integrate them inside a package with some flexibility. However one of the drawback of ROS is that it is officially supported only on Linux Ubuntu 14.04 (for the indigo version) even if the community built around supports other platforms like MAC OS or Windows. This is due to the reason that the libraries tools have a linux oriented implementation and will not work with another exploiting system. 
Other various actions can be performed with sets of nodes called "packages" such as graphic visualization or network connexions (between active nodes). (insert graph image)

### Robots used for the scenario

#### Turtlebot Description
The mobile robot used in the Project is a Turtlebot. The original design was created in 2010 at Willow Garage by Melonee Wise and Tully Foote. Here we used the turtlevot version 2 , similar to version 1 but with a better base. It uses the kobuki base built by Yujinrobot (a South Korean firm originally based on conception of vacuum cleaner robots), an Asus laptop with ROS indigo installed (using Ubuntu 14.04), and an embedded Kinect V1 camera in order to perform obstacle avoidance. It's autonomy of roughly 3 hours also makes it very useful and versatile since it allows operations or modifications to be done whithout time constraint.  
![turtlebot pic](https://github.com/francoislegd/BSCV_ROS/blob/master/Pictures/DSC_0031.JPG)

As seen on the previous image, a modification was performed on the Robot by previous students. In addition to the Kinnect camera, a Lidar system has been added as an alternate way to perform obstacle avoidance and mapping. As it is easier to implement, meaning in particular less topics and associated parameters to take into account than with the Kinnect, we had to use it to implement the given scenario. 

#### Phantom-X description
The second system we had to program is a Phantom-X pincher robotic arm, manufactured by Trossen Robotics. Its conception includes 4 servo-motors plus a gripper to grab objects, giving it 4 degrees of freedom in space. In the project we used it in order to move a cube from a table to a The turtlebot.
![arm pic](https://github.com/francoislegd/BSCV_ROS/blob/master/Pictures/DSC_0035.JPG)


### Scenario description
Our goal was to complete the following scenario: 

**1.** Move the turtlebot to a table while avoiding obstacles
![first pic](https://github.com/francoislegd/BSCV_ROS/blob/master/Pictures/Scenario_pic1.jpg)

**2.** take a cube with the arm and put it on the turtlebot
![second pic](https://github.com/francoislegd/BSCV_ROS/blob/master/Pictures/Scenario_pic2.jpg)

**3.** then go back to another position while transporting the cube (here it will be the departure point in 1.).
![third pic](https://github.com/francoislegd/BSCV_ROS/blob/master/Pictures/Scenario_pic3.jpg)

**These steps should be performed semi-autonomously, so that the user has to write in the terminal a single command for each action to perform.** 

# REPOSITORY CONTENT

### File list
Here is the list of all the files and dependancies contained in this repository we needed to build the project:
(List of all the files and their dependencies in this repository)

### Original Code links
Here are some links to some raw material we further had to modify :
Links to original, unmodified external files packages :

Package installation instruction for the Phantom-X robotic arm [arbotix & turtlebot_arm instructions](https://github.com/NathanCrombez/PhantomXPincherArmROS)
Or alternatively [arbotix_ros](https://github.com/NathanCrombez/arbotix_ros) and [turtlebot_arm](https://github.com/NathanCrombez/turtlebot_arm)

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

Some necessary packages however needed to be installed in order to activate the Lidar device on the turtlebot and to teleoperate the robotic arm. They were present in the  rplidar-turtlebot2-indigo meta-package (especially the turtlebot_le2i package); and as arbotix and robotic_arm packages for the Phantom-X. The robotic_arm package also respects the preliminar survey since most of its sub-packages are derivated from the suggested Move_it package.

#### Executable scripts
These packages also contains original executables scripts of interest as .launch files for general actions, or as .py files for specific instruction nodes. These latter includes in particular some demonstrations pre-implemented that will reveal to be the easiest way to perform the wanted action to turn the scenario successful.  

### Basic commands

#### Turtlebot
Package Rplidar installation after downloading:

`scp rlidar-turtlebot2-indigo.zip turtlebot@192.0.100:/`

Then in order to be runned in ROS we needed the building instructions in the integrated catkin workspace as follows :

`cd ~/catkin_ws`

`catkin_make`

`rospack profile`
 
Then we could perform on the turtlebot the inizialization of the Lidar sensor by launching the basic package command, which also allows to verify if the system works properly :

`roslaunch turtlebot_le2i rplidar_minimal.launch`
 
From there a verification can be done with the next launch command in another terminal :
 
`roslaunch rplidar_le2i view_robot_rplidar.launch`

Further instructions have been performed but since they are part of the project they will be treated in the following section.
(add link to the concerned section). 

#### Phantom-X
The Phantom-X arm needed two packages in order to be programmed. We downloaded the arbotix package in order to make verifications to see if the system works properly by testing its articulations. We installed the turtlebot_arm package as it contains a bringup sub-package that will be essential for the rest of the project.

Once downloaded, we wrote the setup instructions in the following order:

`roslaunch turtlebot_arm_bringup arm.launch`
 
Then the testing node is run using rosrun (as it is a python script):
 
`rosrun arbotix_python arbotix_gui`

![screencap of the result](https://alliance.seas.upenn.edu/~cis700ii/dynamic/team4/wp-content/uploads/sites/6/2015/11/arbGui.png)

We could also teleoperate the arm trough the rviz window by setting manually a planned end position, with a simulation and a run (setting done in the planning menu down left of the window). 
![menu](https://github.com/francoislegd/BSCV_ROS/blob/master/Pictures/manual_control.png)

Before opening the Rviz window we had to launch the turtlebot_arm_moveit_config package through the command :

`roslaunch turtlebot_arm_moveit_config turtlebot_arm_moveit.launch`


# MODIFICATIONS AND PROJECT IMPLEMENTATION

### Scripts of interest

#### Turtlebot
global costmap common param.yaml, local costmap common param.yaml, global common params.yaml,

#### Pick And Place
To perform the pick an place action, we searched for already implemented script that we could adapt to our context. In the sub-package turtlebot_arm_moveit_demos, we found in the bin directory a python program which is a demo for a pick and place action. As the pick and place is already implemented, we had to only perform minor to moderate adaptations.

Here are the commands to find and edit the original pick and place script on the workstation :

`roscd turtlebot_arm_moveit_demos/bin/`

`gedit pick_and_place.py`

### Building a map
We used gmapping package to perform this action, after launching the turtlebot_le2i rplidar_minimal.launch instruction.
at the same time we launch the rviz instruction in order to make move in the space the robot and to build the entire map of the room

### Setting and parameterizing reference coordinates
Originally put on a black square reference , memorized, we go also to the pose8position topic in order to take it. we use also amcl package to initialize coordinates on the map.

### Obstacle avoidance
we changed inflation radius in order to allow more displacement freedom to the robot. Inflation surrounds all obstacles and is also present in the local costmap

### Path Planning 
use of A star algorithm instead of dijkstra, while being a little bit less precise it is also quicker to perform 

### Pick_and_Place adaptations
As a demonstration python script was already available


# CREATED PACKAGES
Our_project Package
Designed to be launchd on the turtlebot, contains one launch file that set up the rplidar
Project_ws Package
Designed to be launched from the workstation, for both turtlebot and Robotic arm 

### Our project Package scripts
Turtlebot_project.launch for setting up the lidar on the turtlebot, needs to be rerun for path planning

### Project_ws Package scripts
Project_WS.launch for entering set up coordinates and path plannification, from origin point to target destination.
Robotic_arm.launch, for activating the pick and place action.
Go_back.launch for entering target destination's coordinates as a starting point, to another point with coordinates that ease the next step.
Go_back2.launch, for entering previous point as starting coordinates and origin point as target destination.


# INSTRUCTIONS TO RUN THE PROCESS

### Download and Install
(will add git clone links and commands)

### Building
(will be similar to original commands)

### Commands

The first execution should be performed on the robot's laptop in order to connect the workstation to the Turtlebot.

`ssh turtlebot@192.168.0.100`

The second command will launch the subpackage turtlebot_le2i rplidar_minimal.launch script along the amcl package in which our created map has been already implemented, whose goal is to atach default coordinates.
(For the Go_back steps the instruction should be relaunched in order to erase the previous coordinates).

`roslaunch Our_project Turtlebot_project.launch`

The next instructions should be launched on the workastation.
For the first displaceent toward the table, the following command  will publish our setup coordinates relatives to the map inside the move/bas topics. At the same time it will display the map in Rviz with the relative position of the turtlebot. This will allows us to :
- visualize the displacement of the robot on the map
- the obstacle avoidance 
- the shortest path plannification.

`roslaunch project_ws Project_WS.launch`

![screencap of the displacement](https://github.com/francoislegd/BSCV_ROS/blob/master/Pictures/Project_WS_launched.png)

Then a third terminal should be open, in which the instruction for the robotic arm will be written. This instruction will display the virtual view of the robotic arm inside another Rviz terminal, while executing the modified pick_and_place python script that will perform the object displacement action.

`roslaunch project_ws Robotic_arm.launch`

![screencap of the rviz robotic arm action](https://github.com/francoislegd/BSCV_ROS/blob/master/Pictures/Performing_Robotic_arm_pick%26place.png)

Once these commands completed, the three processes should be closed. To perform the two step of the Go_back action, the map should be reloaded an the associated coordinates reset. This instruction is still written on the Turtlebot terminal.

`roslaunch our_project Turtlebot_project.launch` (second action)



The Go_back and Go_back2 launch files will perform the same actions as the Project_WS launch file, but with different coordinates.

`roslaunch project_ws Go_back.launch`

The setup instruction should relaunched on the turtlebot terminal a last time in order to move the turtlebot to its departure position. 

`roslaunch our_project Turtlebot_project.launch` (third action)

On the workstation :

`roslaunch project_ws Go_back2.launch`

These are the instructions that should be launched on the terminal to run the given scenario at the beginning of the project.



# RESULT
(add video link and few pics)
# PROBLEMS ENCOUNTERED

### On the Turtlebot
Original coordinates not precise enough, infation radius too small or to big (due to the conception of the robot, it was too far, or the lidar extension collided with the objects), solution, add a plate on which the object is placed, in order that almost no modifications have to be done on pose coordinate while not disturbing the lidar device or impairing obstacle avoidance with blind angles.

Go_back process had to be divided in two steps since it was not easy for the robot to perform due to big angular differences between original and target positions. So to avoid high risks of failure we added a new step that adda an intermediate target position.  

### On the Robotic arm
We faced several problems on the robotic arm since when we started to make it work wired connections were defectuous, explaining unwanted interruptions in the process. after repairs we faced another problem linked to the gripper articulation. In the pick_and_place python script in the moveit_demos package, we performed numerous modifications as wewere facing a problem during the pick and place action. The robot was picking the cube but was unable to perform a place action. We originally thought about unreachable place coordinates but that was not the case. We also removed the present objects in the virtual scene (two boxes and a table) but they were of no influence on the problem. 

We tried also different loop modifications for the place action in order to have a single attempt first, until we saw that it could be parameterized in an initialization argument. We then modified again the loop in order to force a true case for a place after a pick action is successfully done. It made appear an error linked to the gripper joint. we also discovered that the action was successfully done with the original object size in the demo script. We supposed that the problem comes from the  dimensions of our targeted object. 

This asumption has been succesfully verifie as we decreased the size of the object (in particular the y axis value). Since then the robotic arm was able to successfully perform its pick and place action. 

We also originally thought that for more ease it would have been beter to modify the target coordinates for pick and place actions in order to make the virtual scene in rviz correspond to the reality. But placing the robotic arm in the good position simply shortcut the problem. (add some python script modifs ?)


# POINTS OF IMPROVEMENTS

Speed, precision, making it more autonomous with a singe general launch file (but raises new problems)



# CONCLUSION
(will be concluded after the rest of the report is finished) 

# REFERENCES

