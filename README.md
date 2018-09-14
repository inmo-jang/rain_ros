# Haptic Teleoperation in RAIN hub project
---
## Overview
This is a repository of visual/haptic-teleoperated industrial robotic manipulators in RAIN hub project (https://rainhub.org.uk/). 
At the moment, this source file is a gazebo model for UR5 and Robotiq 3-finger gripper. 

** Auther/Maintainer: Inmo Jang (inmo.jang@manchester.ac.uk), Robotics for Extreme Environment Group, University of Manchester**

---
## Installation

### Building from Source

#### Dependencies
The dependencies required are as follows: 

(1) Robotiq 3-finger gripper package (https://inmojang@bitbucket.org/inmojang/robotiq.git)
         
        rosdep install robotiq_modbus_tcp
        sudo apt-get install ros-kinetic-soem
        git clone https://github.com/smits/soem.git
 
 
(2) Universal robot package (https://inmojang@bitbucket.org/inmojang/universal_robot.git)
           
        sudo apt-get install ros-kinetic-universal-robot
        sudo apt-get install ros-kinetic-moveit
         
Note: Some of the dependencies are originally from other repositories (please refer to each repo's git), but modified for this project. 



#### Buliding
To build from source, clone the latest version from this repository into your catkin workspace and compile the package.
    
	        cd catkin_ws/src
            catkin_make

### Packages Overview

This repository consists of following packages:

* ***rain*** the meta-package for the teleoperation system meta package
* ***rain_description*** contains the urdf description of the integrated robot (UR5 + the gripper).
* ***rain_gazebo*** contains the launch file for the gazebo simulator.


---
## Usage

Launch the Gazebo simulator:

        roslaunch rain_gazebo ur5_robotiq.launch

#### Control the gripper

##### Via Keyboard Teleoperation

To control the gripper, in another terminal, excute below in the command line:
        
        rosrun robotiq_s_model_control SModelSimpleController_gazebo.py
        
Here, SModelSimpleController_gazebo.py has one line modification from the original file(SModelSimpleController.py) in the robotiq package.


##### Via ***rostopic pub*** 

Closing the hand halfway:

        rostopic pub --once right_hand/command robotiq_s_model_control/SModel_robot_output {1,0,1,0,0,0,127,255,0,155,0,0,255,0,0,0,0,0,0}

Fully open the hand:

        rostopic pub --once right_hand/command robotiq_s_model_control/SModel_robot_output {1,0,1,0,0,0,0,255,0,155,0,0,255,0,0,0,0,0,0}

Change the grasping mode to pinch and close the gripper:

        rostopic pub --once right_hand/command robotiq_s_model_control/SModel_robot_output {1,1,1,0,0,0,255,255,0,155,0,0,255,0,0,0,0,0,0}

Switch to wide mode and fully open the hand:

        rostopic pub --once right_hand/command robotiq_s_model_control/SModel_robot_output {1,2,1,0,0,0,0,255,0,155,0,0,255,0,0,0,0,0,0}

Change to scissor mode and close the fingers:

        rostopic pub --once right_hand/command robotiq_s_model_control/SModel_robot_output {1,3,1,0,0,0,255,255,0,155,0,0,255,0,0,0,0,0,0}

Open fingers:

        rostopic pub --once right_hand/command robotiq_s_model_control/SModel_robot_output {1,3,1,0,0,0,0,255,0,155,0,0,255,0,0,0,0,0,0}




#### Control the arm

##### Via Keyboard Teleoperation

Execute below in the command line to control each joint using a keyboard: 

        rosrun ur_driver gazebo_teleop_key.py



![picture](rain/UR5_robotiq.png)
  