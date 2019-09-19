# HoloLens Robot Controller 
3D Vision, ETHZ, Spring 2019 - http://www.cvg.ethz.ch/teaching/3dvision/

#### Authors: 
* Arda Duzceker
* Jonas Hein
* Octavio Siller
* Sophokles Ktistakis

Are you ready to change your perception on what a remote controller is?

HoloLens Robot Controller is a system that is specially developed for Microsoft HoloLens and Trimbot. It enables you to comfortably control a robot (called Trimbot) using either a holographic joystick or by air-tapping to a location anywhere in the room to command the robot to move there. Also, while wearing the HoloLens, we show you what the robot "sees" and "knows" as a hologram of 3D points. You can then observe which areas the robot knows well and which areas the robot has not learned about yet. So, you can decide on your commands deliberatively.

How does this work? In very simple terms of computer vision, devices can maintain a map of their environment that is created and constantly updated using the sensor data (e.g. cameras, depth-sensors). In our case we have two independent devices and therefore two independent maps. In order to show the user the robot map or to understand where the user air-tapped, we calculate in the background the position and orientation of the robot in the map of the HoloLens. This 3D transformation can then be used to calculate where the robot must go to reach the tapped position.

The main milestones of the project are:
1. Setting up a network communication between the two devices
   * ROS messages interface for HoloLens: https://github.com/dwhit/ros-sharp
2. Tracking the robot in HoloLens
   * ArUco marker detection and tracking: https://github.com/qian256/HoloLensARToolKit
3. Aligning the respective point cloud maps and coordinate systems
   * Integration of Iterative Closest Point (ICP) algorithm into ROS node: http://wiki.ros.org/pcl
4. Displaying the robot's map on top of the real environment in HoloLens
5. Being able to send desired commands to the robot

**For a detailed explanation of the project and the implementation, please refer to the project [report](/report.pdf) or [poster](/poster.pdf)**

**For a guide on installation and deployment, please refer to the [Unity-App-README](unity_app/README.md) and the [ROS-Node-README](ros_node/README.md)**
