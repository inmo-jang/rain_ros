# Haptic Teleoperation in RAIN hub project (Phase 2)

---
## Instruction for Demo

Currently, this branch is in the middle of integrating CyberGrasp/CyberGlove. However, I am trying to keep the existing interfaces as they are for Phase 1 (release/demo_ver1.0). 

For unity, you need to clone the respository rain_unity3d-phase2

For demo, follow the instruction below. 

### (1) Preparation in ROS

Open networks, Run joint_speed publishing rate regulator, and unity..

	roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=172.22.22.2

	rosrun robotiq_s_model_control SModelTcpNode.py 192.168.1.11

	roslaunch usb_cam usb_cam_1.launch

	rosrun ur_driver ur5_joint_speed_regulator.py

	roslaunch rain_unity ur5_robotiq_unity_real.launch

### (2) Run Control Nodes in ROS, while Unity is running 

After running rain_unity3d in Unity (Windows), do the followings

	rosrun ur_driver ur5_teleop_leap_vel_mode0.py real

	rosrun ur_driver ur5_teleop_leap_vel_mode1.py real

	rosrun robotiq_s_model_control SModelController_vr.py real
    
    rosrun robotiq_s_model_control HapticController.py





---
## Overview
This is a repository of visual/haptic-teleoperated industrial robotic manipulators in RAIN hub project (https://rainhub.org.uk/). 
At the moment, this source file is a gazebo model for UR5 and Robotiq 3-finger gripper. 

** Author/Maintainer: Inmo Jang (inmo.jang@manchester.ac.uk), Robotics for Extreme Environment Group, University of Manchester**


## Key Information for Demo

According to *ur_modern_driver*, we can control a UR using joint velocities via *joint_speed* topic. 







### Demo 1. Teleoperation of a real UR5 via VR/LEAP (via joint_speed)


#### (1) Bring up 

Bring up the UR5(which uses v3.X):

        roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=172.22.22.2
        
        
Bring up the Gripper:

        rosrun robotiq_s_model_control SModelTcpNode.py 192.168.1.11


#### (2) Bridge ROS and Unity

launch the following launch file (rosbridge):

        roslaunch rain_unity ur5_robotiq_unity_real.launch


In the Unity side, run the scene (*Scene_181109.unity*) with rosbridge. And, we need to deactivate some publishers/subscribers of "Rosconnecter", otherwise overflow errors come up. 


#### (3) Run Control Nodes

In the ROS side again, launch one of the following controller nodes:

- Mode 0 Control Node (You should put additoinal argument 'gazebo' or 'real')

        rosrun ur_driver ur5_teleop_leap_vel_mode0.py [gazebo or real]

- Mode 1 Control Node 

        rosrun ur_driver ur5_teleop_leap_vel_mode1.py [gazebo or real] 

- Gripper Control Mode (Only being activated in Mode 1)

        rosrun robotiq_s_model_control SModelController_vr.py [gazebo or real] 
        

Then, launch the following to get the haptic feedback from the gripper (This only works for the real robot):

        rosrun robotiq_s_model_control HapticController.py


### Demo 2-1. Teleoperation of a Gazebo UR5 via VR/LEAP (via joint_speed)

This way is to enable the gazebo model to have the same interfaces as the real robot does, by using **gazebo_joint_speed_interface.py**. 

#### (1) Bring up the Gazebo robot

In the ROS side, launch the Gazebo simulator:

        roslaunch rain_gazebo ur5_robotiq.launch

Here, we need an interface to the UR gazebo model because it currently accepts a desired joint position as an input. 
Thus, after launching the gazebo model, you need to run

        rosrun ur_driver gazebo_joint_speed_interface.py
        
        rosrun ur_driver ur5_joint_speed_regulator.py

#### Then, follow (2) and (3) in Demo 1.




### Demo 2-2. Teleoperation of a Gazebo UR5 via VR/LEAP (via action client. Nov2018 version)

In the ROS side, launch the Gazebo simulator:

        roslaunch rain_unity ur5_robotiq_unity.launch

In the Unity side, run the scene (*Scene_181109.unity*) with rosbridge. 

In the ROS side again, launch one of the following controller nodes:

- Mode 0 Control Node

        rosrun ur_driver gazebo_teleop_leap_mode0_vr.py


- Mode 1 Control Node

        rosrun ur_driver gazebo_teleop_leap_mode1_vr.py

- Gripper Control Mode (Only being activated in Mode 1)

        rosrun robotiq_s_model_control SModelController_gazebo_vr.py


Result is : https://www.youtube.com/watch?v=TQSg8v2cMcE


Please refer to branch **ver1.1 to ver 2.0** for more detailed description (e.g., how to set the networks of the robot/gripper, etc.).  
        



        