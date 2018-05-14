# BSCV_ROS
BSCV Project in Robotics using ROS, Turtlebot and Phantom-X Pincher arm



# INTRODUCTION

### Description of the project
This report will describe in details the project given during Bachelor in Computer Vision's second semester in Robotics. In the given scenario we had to move a Turtlebot to a target position while avoiding obstacles, then put an object on it by performing a pick and place action with a Phantom-X arm before making the turtlebot return to its original place with the object. The report will focus on every aspects of the project, from the available material and code to the final implementation.    


### What is ROS ?
ROS (Robot Operating System) is an open source middleware that allows its users to perform robot programming. As middleware, it possess characteristics of both software and hardware, since it is able to perform various actions like hardware abstraction and low level controls. However this characteristic is of a great interest since it brings important tasks such as the creation of new commandlines or package management, which have proved to be essential tools for the realization of the project. Different versions of ROS exists with some differences, so for compatibility reasons we are using the Indigo realease. 
![ROS logo](https://github.com/francoislegd/BSCV_ROS/blob/master/Pictures/indigoigloo_600.png)


ROS presents the advantage to support multiple languages implementations such as roscpp for C++ or rospy for Python, as well as specific libraries in these languages, so one can write actions called "nodes" and then integrate them inside a package with some flexibility. However one of the drawback of ROS is that it is officially supported only on Linux Ubuntu 14.04 (for the indigo version) even if the community built around supports other platforms like MAC OS or Windows. This is due to the reason that the libraries tools have a linux oriented implementation and will not work with another exploiting system. 
Other various actions can be performed with sets of nodes called "packages" such as graphic visualization or network connexions (between active nodes).

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

[1.zip file containing all the packages with their modified scripts](https://github.com/francoislegd/BSCV_ROS/blob/master/Project_Packages.zip)

[2.zip file containing the created map for the project]
(https://github.com/francoislegd/BSCV_ROS/blob/master/Project_map.zip)

[3.file containing the orginal code for the pick and place action with comments on the lines which have been modified in the project](https://github.com/francoislegd/BSCV_ROS/blob/master/unmodified_pick_and_place.zip)

[4.folder containg pics and screencaps](https://github.com/francoislegd/BSCV_ROS/tree/master/Pictures)

### Original Code links
Here are some links to some raw material we further had to modify :
Links to original, unmodified external files packages :

Package installation instruction for the Phantom-X robotic arm [arbotix & turtlebot_arm instructions](https://github.com/NathanCrombez/PhantomXPincherArmROS)
Or alternatively [arbotix_ros](https://github.com/NathanCrombez/arbotix_ros) and [turtlebot_arm](https://github.com/NathanCrombez/turtlebot_arm)

Packages useful to make the Lidar device work, especially [turtlebot_le2i](https://github.com/roboticslab-fr/rplidar-turtlebot2) 

### Information links
---Links differents sections in the readme file for more detailed explanations :

[INTRODUCTION](#INTRODUCTION)
[REPOSITORY CONTENT](#REPOSITORY-CONTENT)
[ORIGINAL CONTENT](#ORIGINAL-CONTENT)
[MODIFICATIONS AND PROJECT IMPLEMENTATION](#MODIFICATIONS-AND-PROJECT-IMPLEMENTATION)
[CREATED PACKAGES](#CREATED-PACKAGES)
[INSTRUCTIONS TO RUN THE PROCESS](#INSTRUCTIONS-TO-RUN-THE-PROCESS)
[RESULT](#RESULT)
[PROBLEMS ENCOUNTERED](#PROBLEMS-ENCOUNTERED)
[POINTS OF IMPROVEMENTS](#POINTS-OF-IMPROVEMENTS)
[CONCLUSION](#CONCLUSION)

# ORIGINAL CONTENT
Here we will describe the original content we had the access prior to its implementation in the project.

### Setup Material
We had to start with the following material : 
- Turtlebot2
- Phantom-X pincher arm
- ROS indigo (and Ubuntu 14.04) 
- Computer as workstation
- Lidar device (will put links to the intro).

### Starting bases
Here we will discuss about the original files and actions before the modifications.

#### Packages
For this project, some packages were already installed in order to teleoperate the turtlebot : Gmapping, AMCL, Move_Base and Turtlebot Navigation. The original suggestions carried by our preliminary survey were respected since we were requiring the use of the  AMCL and Move_Base packages.

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

#### Phantom-X
The Phantom-X arm needed two packages in order to be programmed. We downloaded the arbotix package in order to make verifications to see if the system works properly, by testing its articulations. We installed the turtlebot_arm package as it contains a bringup sub-package that will be essential for the rest of the project.

Once downloaded, we wrote the setup instructions in the following order:

`roslaunch turtlebot_arm_bringup arm.launch`
 
Then the testing node is run using rosrun (as it is a python script):
 
`rosrun arbotix_python arbotix_gui`

![screencap of the result](https://alliance.seas.upenn.edu/~cis700ii/dynamic/team4/wp-content/uploads/sites/6/2015/11/arbGui.png)

We could also teleoperate the arm trough the rviz window by setting manually a planned end position, which simulates the robotic arm and its movements (settings can be done in the menu down left of the window). 
![Rviz_window](https://github.com/francoislegd/BSCV_ROS/blob/master/Pictures/Rviz_Window.png)

The Rviz view is opened by launching the turtlebot_arm_moveit_config package through the command :

`roslaunch turtlebot_arm_moveit_config turtlebot_arm_moveit.launch`


# MODIFICATIONS AND PROJECT IMPLEMENTATION

### Scripts of interest

#### Turtlebot
##### Ros Navigation
To perform ROS navigation we need two things:
•	We need a map (Generated using the gmapping package)
•	We need to be able to localize robot on that map. Having a map alone without localization is useless, if the robot is unknown about its whereabouts. So in order to be able to achieve proper navigation task, robot should be able to know in which position of the Map it is located and with which orientation at every moment. This concept of knowing the whereabouts in the Map is referred to as ‘Localization’ in terms of ROS navigation.
To achieve the locomotion of a robot in a known map, we need some sort of mechanism in ROS environment. This is achieved by using a Navigation stack. This stack is composed of set of ROS nodes and algorithms that are vital to move the robot from one point to another, while making sure obstacles are avoided that may arise in its(robot’s) path. 
Navigation Stack, takes an input the current location of the robot, the goal pose, the Odometry data of Robot (wheel encoders etc..) and data from a sensor (in our case we used Laser sensor). An in return, it outputs the necessary velocity commands and forward them to the mobile base in order to move the robot to the designated goal, while making sure the turtlebot won’t crash against obstacles, or get lost in the process.
Following picture is taken from ROS wiki page that mentions the building blocks of Navigation Stack.

![NavigationStack](https://github.com/francoislegd/BSCV_ROS/blob/master/Pictures/overview_tf.png)

According to shown figure above, some functional b locks must be presented in order to work and communicate with the Navigation Stack. Following are brief explanations of all blocks which need to be provided as an input to ROS Navigation stack.
•	Odometry source:  Odometry data of robot gives the robot position with respect to its starting position. Usually main odometry sources are wheel encoders etc. The odom value should publish to the Navigation stack, which has a message type of nav_msgs/ Odometry. The odom message can hold the position and the velocity of the robot.
•	Sensor source: Sensors are used for two tasks in navigation: one for localizing the robot in the map (using for example the laser) and the other one to detect obstacles in the path of the robot (using the laser, sonars or point clouds).
•	Sensor transforms/tf: the data captured by the different robot sensors must be referenced to a common frame of reference (usually the base_link) in order to be able to compare data coming from different sensors. The robot should publish the relationship between the main robot coordinate frame and the different sensors' frames using ROS transforms.
•	Base_controller: The main function of the base controller is to convert the output of the Navigation stack, which is a Twist (geometry_msgs/Twist) message, into corresponding motor velocities for the robot.

##### Move_base node

This is the most important node of the Navigation Stack. The main function of move_base node is to mobilize a robot from its current position to a designated (goal) position. This node links the global planner and the local planner for path planning, connecting to rotate recovery package if the robot is stuck in some obstacle, and connecting global costmap and local costmap for getting the map of obstacles of the environment.
So basically for navigation stack to work, we have move_base node, we need to provide it odom data, transforms of robot and laser data. Our robot should be localized in the give map. Following all this available info, we can send the goal pose to move_base node and move_base will give as an output proper velocity commands in order to move the robot towards the goal position as can be seen in the diagram below:

![move_base](https://github.com/francoislegd/BSCV_ROS/blob/master/Pictures/move_base_diagram.png)

##### Localization
When we initially start to move the robot in the generated map, it is unaware of its position in the map, so therefore the robot will not move as expected, it generates numerous guesses as to where it is going to move in the map. These guesses are known as particles (represented by tiny small green arrows when viewed in Rviz). Each particle contains the full description of future possible pose. When the robot observes the environment it is in via sensor readings, it discards the particles that do not match with these readings, and generates more particles close to those that look more likely. This way, most particles in the end cover the most probable pose the robot is in. So more the robot moves in the map, the more data its sensor will collect from surroundings hence the localization will be more precise. This is known as Monte Carlo Localization (MCL) algorithm, or also particle filter localization.

##### AMCL Package
The AMCL (Adaptive Monte Carlo Localization) package provides the amcl node, which practices the MCL algorithm in order to track the localization of a robot moving in 2D space. This node subscribes to the data of the laser, the laser based map, and the transformation of the robot, and publishes its estimated positions in the map. On startup, the amcl node initializes its particles filter.
Previously we mentioned move_base is quite an essential node. This is due to the fact that main function of move_base is to move the robot from its current position to a goal position. Basically, this node is an implementation of a SimpleActionServer, which takes a goal pose with message type geometry_msgs/PoseStamped. Therefore we can send possible goals to this node by using a SimpleActionClient.
This Action server provides the topic move_base/goal, which is the input of the Navigation Stack. This topic is then used to provide the goal pose. When this move_base node receives a goal pose on /move_base/goal topic, it links to components such as global planner, local planner, recovery behaviors, and costmaps, and creates an output, which is a velocity command with the message type geometry_msgs/Twist, and sends it to the velocity topic in order to move the robot.

##### Global Planner
When a new goal is received by the move_base node, this goal is immediately sent to the global planner. Then, the global planner is in charge of calculating a safe path in order to arrive at that goal pose. This path is calculated before the robot start moving, so it will not take into account the readings that the robot sensors are acquiring while moving.
For calculating the path the global planner uses the costmap.

##### Costmap
A costmap is a map that represents places that are safe for the robot to be in a grid of cells. Usually, the values in the costmap are binary, representing either free space or places where the robot would be in collision.
Each cell in a costmap has an integer value in the range (0-255). There are some special values frequently used in this range, which work as follows:
•	255(No_information): Reserved for cells where not enough information is known.
•	254 (lethal_Obstacle): Indicates that a collision- causing obstacle was sensed in this cell.
•	253(Inscribed_inflated_obstacle): Indicates no obstacle, but moving the centre of the robot to this location will result in a collision.
•	0 (Free_space): Cells where there are no obstacles and, therefore, moving the centre of the robot this position will not result in a collision.
There exist two types of costmaps: global costmap and local costmap. The main difference between them is, basically the way they are built:
•	The globlal costmap is created from the static map. (The map generated using the gmapping package)
•	The local costmap is created from the robot’s sensor readings.
So the global planner uses the global costmap in order to calculate the path to follow.

##### Global Costmap:
The global costmap is created from a user-generated static map. In this case the costmap is initialized to match the width, height, and obstacle information provided by the static map. This configuration is normally used in conjunction with a localization system, such as amcl.

##### Local Planner
Once the global planner has calculated the path to follow, this path is sent to local planner. The local planner, then, will execute each segment of the global plan. So given a plan to follow (provided by the global planner) and a map, the local planner will provide velocity commands in order to move the robot.
Unlike the global planner, the local planner monitors the odometer and the laser data, and chooses a collision-free local plan for the robot. So, the local planner can recomputed the robot’s path while on move in order to keep the robot from colliding with any obstacles, yet still allowing it to reach its destination.

##### Local costmap
The first thing to remember here is that the local planner uses the local costmap in order to calculate the local plans. 
Unlike the global costmap, the local costmap is created directly from the robot’s sensor readings. Given a width and height for the costmap, it keeps the robot in the center of the costmap as it moves throughout the environment, dropping obstacles information from the map as the robot moves.

#### Pick And Place
To perform the pick an place action, we searched for already implemented script that we could adapt to our context. In the sub-package turtlebot_arm_moveit_demos, we found in the bin directory a python program which is a demo for a pick and place action. As the pick and place is already implemented, we had to only perform minor to moderate adaptations.

Here are the commands to find and edit the original pick and place script on the workstation :

`roscd turtlebot_arm_moveit_demos/bin/`

`gedit pick_and_place.py`

### Building a map
We used gmapping package to perform this action.This entire process is relatively easier. We followed the procedure mentioned on ros wike [page](http://wiki.ros.org/turtlebot_navigation/Tutorials/Build%20a%20map%20with%20SLAM).Instead of launching `roslaunch turtlebot_bringup minimal.launch
`First we launched `roslaunch turtlebot_le2i rplidar_minimal.launch` on turtlebot followed by `roslaunch turtlebot_navigation gmapping_demo.launch` , again executed on turtlebot. On the workstation `roslaunch turtlebot_rviz_launchers view_navigation.launch` command was executed. After having Rviz open, using the keyboard directional keys, robot was moved in the room to create the map. Only after the whole map of room was generated, it was saved using command 'rosrun map_server map_saver -f /ros/indigo/catkin_ws/map/hassan_lidar.yaml'. One thing to note here is that make sure map is saved on the turtlebot, and not on the workstation otherwise when amcl demo is launced for autonomous navigation it wouldn't be able to find the map.
So once a map is generated, following the command mentioned on roswike [page](http://wiki.ros.org/turtlebot_navigation/Tutorials/Autonomously%20navigate%20in%20a%20known%20map) turtlebot autonomous navigation can be achieved.


### Obstacle avoidance
Generally when turtlebot is used with ros [turtlebot_navigation](https://github.com/turtlebot/turtlebot_apps/tree/indigo/turtlebot_navigation) package and using the command `roslaunch turtlebot_navigation amcl_demo.launch map_file:=/tmp/my_map.yaml` mentioned here on roswiki [page](http://wiki.ros.org/turtlebot_navigation/Tutorials/Autonomously%20navigate%20in%20a%20known%20map) for autonomous navigation, it will be seen that turtlebot would be able to move to desired goal position while making sure that it avoids any obstacles in the way. But we realized this only works if you are working with kinnect sensor, in case of Lidr we found that obstacle avoidance was not working.
So some changes were made in the ROS turtlebot_navigation parameters. In the [costmap_common_param](https://github.com/turtlebot/turtlebot_apps/blob/indigo/turtlebot_navigation/param/costmap_common_params.yaml) we cahnged the max_obstacle_height from 0.25 to 1.00.Because max_obstacle_height represents maximum height of any obstacle to be inserted into the costmap in meters. This parameter should be set to slighlty higher than the height of the robot. So after calucalting the mounted Lidr height on turtlebot with measuring tape we agreed for 1.00 value.

Another changes were made in [global_costmap_params](https://github.com/turtlebot/turtlebot_apps/blob/indigo/turtlebot_navigation/param/global_costmap_params.yaml). 

`plugins:                                                             `

`    - {name: static_layer,            type: "costmap_2d::StaticLayer"}    `

`    - {name: obstacle_layer,          type: "costmap_2d::VoxelLayer"}`

`    - {name: inflation_layer,         type: "costmap_2d::InflationLayer"}`

was replaced with just: `map_type : costmap` and similarly in the [local_costmap_params](https://github.com/turtlebot/turtlebot_apps/blob/indigo/turtlebot_navigation/param/local_costmap_params.yaml) follwong plugins:

`plugins:                                                             `

`    - {name: obstacle_layer,      type: "costmap_2d::VoxelLayer"}    `

`    - {name: inflation_layer,     type: "costmap_2d::InflationLayer"}` were replaced with just: `map_type : costmap`.

After making these changes we were able to achive obstacle avoidance with just lidr sensor.



### Path Planning 
Use of A star algorithm instead of dijkstra, while being a little bit less precise it is also quicker to perform.This was achieved by changing the parameter use_dijkstra value in [global_planner_params.yaml](https://github.com/turtlebot/turtlebot_apps/blob/indigo/turtlebot_navigation/param/global_planner_params.yaml) from True to False.

### Pick_and_Place adaptations
As a demonstration python script was already available in the sub-package turtlebot_arm_moveit_demos, we simply had to look at the code and perform some adaptations inside in order to implement it in the scenario. At the beginning we were originally searching inside the topic list of the robotic arm when the turtlebot_arm_moveit_config subpackage was run. 
![pick_showing_part of the list](https://github.com/francoislegd/BSCV_ROS/blob/master/Pictures/topic%20list%20of%20the%20robotic_arm.jpg)

However multiple publications inside various topics (such as `group/execute_trajectory`, `group/move_group` or `group/pickup ` and `group/place`) was not the best way to perform the pick and place scenario, since it was easier to modify the already existing code. 

In the pick_and_place demo script inside the subpackage turtlebot_arm_moveit_demos/bin directory, we originally wanted modifiy the virtual scene displayed in Rviz in order to make it correspond to what we saw in reality. Some implemented objects in the original script were commented and the dimensions of the table were changed. (put a pick) These change were removed since they were not useful (the robotic arm could be moved in a better configuration).

As we wanted a successful pick and place action in our scenario, we had to perform and keep the following changes from the starting code:

*1. Line 93 : the tolerance was set to 1 radian instead of 0.1 to allow more freedom to the action regarding orientation in the action.
This increases the chance of successful pick and place.*

` # Allow some leeway in position (meters) and orientation (radians)`

` arm.set_goal_position_tolerance(0.04)`
  
` arm.set_goal_orientation_tolerance(1)`

*2. Lines 105 and 108 : the max attempt values were changed from 3 to 1 since we wanted the process to be performed a single time.*

` # Set a limit on the number of pick attempts before bailing`

` max_pick_attempts = 1`
  
  
` # Set a limit on the number of place attempts`

` max_place_attempts = 1`


***3. Line 171 : the size of the object has been put to [0.07, 0.007, 0.10] for length, width and height instead of [0.017, 0.017, 0.017]. The reason was that the width was not accepted by the gripper, causing each time a failure in the place process, even when the pick sub-action was successful.***

` # Set the target size [l, w, h]`

` target_size = [0.07, 0.007, 0.10]`

*4. Lines 178, 186 and 194 : we lowered the height of the table and all associated objects by adding -0.08, in order to add the difference between the base of the robotic arm and the table.*

`table_pose.pose.position.z = (table_ground + table_size[2] / 2.0)-0.08`

`box1_pose.pose.position.z = (table_ground + table_size[2] + box1_size[2] / 2.0)-0.08`

`box2_pose.pose.position.z = (table_ground + table_size[2] + box2_size[2] / 2.0)-0.08`


*5. Line 203 : we lowered the height of the target by -0.075 to perform a proper pick action without colliding the table (instead of -0.08).*

`target_pose.pose.position.z = (table_ground + table_size[2] + target_size[2] / 2.0)-0.075`

*6. Lines 226 and 227 : x and y place coordinates were changed from -0.03 to -0.15 and -0.15 to -0.23 respectively to optimize the place sub-action on the turtlebot.*

`place_pose.pose.position.x = table_pose.pose.position.x - 0.15`

`place_pose.pose.position.y = -0.23`

These changes allowed us to implement a relatively reliable pick and place action for the scenario of this project. 


# CREATED PACKAGES

We created two packages that had to be implemented on their specific machine.

### Our project Package scripts
Our_project package contains the Turtlebot_project.launch script that should be run on the turtlebot since it is designed to launch the Lidar by integrating the launch command of the rplidar_le2i package. This launch file also call the map we created, that will be set for the Rviz visualization, and launch the amcl_demo package.

`<launch>`

	<!--include another launch file -->
	<include file = "$(find turtlebot_le2i)/launch/rplidar_minimal.launch" />
	<include file = "$(find turtlebot_navigation)/launch/amcl_demo.launch">
	<arg name= "map_file" value="/home/turtlebot/ros/indigo/catkin_ws/map/hassan_lidar.yaml"/>
	</include>
	
`</launch>`


### Project_ws Package scripts
The project_ws package is designed to be installed on the workstation. It contains 4 launch files, three for the navigation of the turtlebot (Project_WS.launch, Go_back.launch and Go_back2.launch), and one for the Phantom-X pincher arm. for entering set up coordinates and path plannification, from origin point to target destination.
- Project_WS.launch is run  to perform the first displacement to the table.
- Go_back.launch was made for entering target destination's coordinates as a starting point, to another point with coordinates that ease the return step.
- Go_back2.launch, for entering previous point as starting coordinates and origin point as target destination.
These three packages share the same script with changes only to the coordinates published in the specified topics. They allow the user to see the robot's displacement in Rviz.

`<launch>`

	<!--include another launch file -->
	<include file = "$(find turtlebot_rviz_launchers)/launch/view_navigation.launch"/>
	<!--Start a node-->
	<!--node pkg="turtlebot" name="go_to_specific_point_on_map" type= "go_to_specific_point_on_map.py">
	</node>-->

	<node pkg="rostopic" type="rostopic" name="rostopic" args="pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped 
'{header: {seq: 0, stamp:{secs: 0, nsecs: 0}, frame_id: 'map'}, pose: {pose:{position: {x: -4.96073627472, y: 1.06203043461 , z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.371774223976, w: 0.928323179926}}}}'"/>


	<node pkg="rostopic" type="rostopic" name="rostopicgoal" args="pub -1 /move_base_simple/goal geometry_msgs/PoseStamped 
'{header: {seq: 0, stamp:{secs: 0, nsecs: 0}, frame_id: 'map'}, pose:{position: {x: 0.318982511759, y: -0.757121920586 , z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.360750805748, w: 0.932662241196}}}'"/>

`</launch>`


The Robotic_arm.launch file activate the bringup, launch the rviz window, display and perform the pick and place action by calling the script pick_and_place. 

`<launch>`

	<!--include robotic arm launch files -->
	<include file = "$(find turtlebot_arm_bringup)/launch/arm.launch"/>
	<include file = "$(find turtlebot_arm_moveit_config)/launch/turtlebot_arm_moveit.launch"/>

	<!--Start a node-->
	<node pkg="turtlebot_arm_moveit_demos" name="originpick_and_place" type= "original_pick_and_place.py">
	</node>

`</launch>`

# INSTRUCTIONS TO RUN THE PROCESS

### Download and Install

`cd ~/ros/indigo/catkin_ws/src`

`git clone https://github.com/francoislegd/BSCV_ROS.git`

We should unzip the files Project_map and Project_packages, and move the others outside the ros directory.
Then the extracted packages should be placed inside the src directory.
Our_project package should be copied to the turtlebot and then be deleted from the directory.

`ssh turtlebot@192.168.0.100`

`scp Our_project turtlebot@192.168.0.100:/`

We should displace the file manually to the /ros/indigo/catkin_ws/src directory.

### Building

On the workstation from the src directory:

`cd .. && catkin_make`

On the turtlebot :

`ssh turtlebot@192.168.0.100`

`cd /ros/indigo/catkin_ws/src`

`sudo cd .. && catkin_make`

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
When the turtlebot launch files are launched we can also run the command
`rosrun rqt_graph rqt_graph`
which displays all the interactions between the different active nodes.
![graph_turtlebot](https://github.com/francoislegd/BSCV_ROS/blob/master/Pictures/rosgraph_turtlebot.png)

The same can be done for the robotic arm.
![graph_Phantom-X](https://github.com/francoislegd/BSCV_ROS/blob/master/Pictures/rosgraph_Phantom-X.png)

[The implementation of the project can be seen here](https://www.youtube.com/watch?v=_z6LXOft7aQ)


# PROBLEMS ENCOUNTERED

### On the Turtlebot

First problem we had was when turtlebot was initally given the command to move from its initial position towards goal or going back to its initial position after picking up the cube, sometimes it used to move away from its calculated path, and once this used to happen, it used to stop moving. What we did was here simpy reduced the max_vel_x in this [planner](https://github.com/turtlebot/turtlebot_apps/blob/indigo/turtlebot_navigation/param/dwa_local_planner_params.yaml) from 0.5 to 0.3. This way with reduced speed turtlebot didn't use to diverge from its trajectory and thus was able to fill the assigned tasks successfully.

After the cube has been placed on the turtlebot, it had to return back to its initial position. But the problem was since it was too close the table(obstacle) somehow it was not possible for turtlebot to perform the 180 degree turn, also due to set inflation radius, recovery behaviour were not satisfactory, We tried different inflation radius and cost_scaling factor, but then the problem arose when turtlebot had to cross the small room and enter into main room towards the table to recieve the cube there was always edge collision with the door edges.So therefore for going back to its initial position Go_back process was divided into two parts to increase the success chances.

In our lauch file Project_WS we mentioned the initial pose coordinates: '<node pkg="rostopic" type="rostopic" name="rostopic" args="pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped 
'{header: {seq: 0, stamp:{secs: 0, nsecs: 0}, frame_id: 'map'}, pose: {pose:{position: {x: -4.96073627472, y: 1.06203043461 , z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.371774223976, w: 0.928323179926}}}}'"/>' so that turtlebot everytime on launch of this launch file is able to localize itslef automatically rather than bieng manually localized using 2D pose estimate from Rviz. But the problem was with it most of the time on lauch of this launch file, it was observed that the turtlebot was not being properly localized, the map overlayed on map seemed unmatching, therefore we changed the coordinates of the initial pose of x, y and a in the [amcl_demo.launch](https://github.com/turtlebot/turtlebot_apps/blob/indigo/turtlebot_navigation/launch/amcl_demo.launch) turtlebot_navigation package.
as can be seen below:

`<!-- AMCL -->`

  `<arg name="custom_amcl_launch_file" default="$(find turtlebot_navigation)/launch/includes/amcl/$(arg 3d_sensor)_amcl.launch.xml"/>`
  
  `<arg name="initial_pose_x" default="-4.96073627472"/> <!-- Use 17.0 for willow's map in simulation -->`
  
  `<arg name="initial_pose_y" default="1.06203043461"/> <!-- Use 17.0 for willow's map in simulation -->`
  
  `<arg name="initial_pose_a" default="0.928323179926"/>`



### On the Robotic arm
We faced several problems on the robotic arm since when we started to make it work wired connections were defectuous, explaining unwanted interruptions in the process. after repairs we faced another problem linked to the gripper articulation. In the pick_and_place python script in the moveit_demos package, we performed numerous modifications as we were facing a problem during the pick and place action. The robot was picking the cube but was unable to perform a place action. We originally thought about unreachable place coordinates but that was not the case. We also removed the present objects in the virtual scene (two boxes and a table) but they were of no influence on the problem. 

We tried also different loop modifications for the place action in order to have a single attempt first, until we saw that it could be parameterized in an initialization argument. We then modified again the loop in order to force a true case for a place after a pick action is successfully done. It made appear an error linked to the gripper joint.
![error_pick_message](https://github.com/francoislegd/BSCV_ROS/blob/master/Pictures/error_message_after%20forcing.jpg).

we also discovered that the action was successfully done with the original object size in the demo script. We supposed that the problem comes from the  dimensions of our targeted object. This asumption has been succesfully verified as we decreased the size of the object (in particular the y axis value). Since then the robotic arm was able to successfully perform its pick and place action. 

We also originally thought that for more ease it would have been beter to modify the target coordinates for pick and place actions in order to make the virtual scene in rviz correspond to the reality. But placing the robotic arm in the good position simply shortcut the problem.


# POINTS OF IMPROVEMENTS

Speed, precision, making it more autonomous with a singe general launch file are some points of improvements, but it also raises new problems like how to avoid relaunching for each displacement the lidar package.
Instead of using the lidar, we can use together lidar and kinnect for better performances.
Kinnect sensor can also be used in parallel with the robotic arm. This way it will make the pick and place scenario more flexible as we do not have to hardcode coordinates of the cube. 



# CONCLUSION
We were successufully able to implement obstacle avoidance only using lidar. Following this we were able to perform a reliable pick and place process. In the end we were able to implement the turtlebot autonomous navigation with pick and place scenario. 

# REFERENCES
ROS WIKI PAGES (http://wiki.ros.org/)
