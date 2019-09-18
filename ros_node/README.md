# Hololens Robot Controller - ROS node

## Prerequisites
The installation process slightly differs based on the OS you are running. Both Ubuntu 16.04LTS and 18.04LTS should work fine. If you have any problems please refer to the Troubleshooting section below!

### Ubuntu 16.04LTS
*  Install ROS kinetic and the rosbridge_suite: `sudo apt install ros-kinetic-desktop-full ros-kinetic-rosbridge-suite`
*  Initialize ROS: `sudo rosdep init`, followed by `rosdep update`
*  OpenCV 3.4.6: [Download Link](https://github.com/opencv/opencv/archive/3.4.6.zip), [Linux Install Tutorial](https://docs.opencv.org/3.4.6/d7/d9f/tutorial_linux_install.html)
*  Eigen Library 3.3.4: `sudo apt install libeigen3-dev`
*  Point Cloud Library 1.8.0: [Download Link](https://www.dropbox.com/s/9llzm20pc4opdn9/PCL-1.8.0-Linux.deb?dl=0), [Linux Install Tutorial](https://larrylisky.com/2016/11/03/point-cloud-library-on-ubuntu-16-04-lts/)
   *  You might need to install missing dependencies first (flann, pcap, boost, opengl): `sudo apt install libflann-dev libpcap-dev libboost-all-dev libglu1-mesa-dev freeglut3-dev mesa-common-dev`

### Ubuntu 18.04LTS
*  Install ROS melodic and the rosbridge_suite: `sudo apt install ros-melodic-desktop-full ros-melodic-rosbridge-suite`
*  Initialize ROS: `sudo rosdep init`, followed by `rosdep update`
*  OpenCV 3.4.6: [Download Link](https://github.com/opencv/opencv/archive/3.4.6.zip), [Linux Install Tutorial](https://docs.opencv.org/3.4.6/d7/d9f/tutorial_linux_install.html)
*  Eigen Library 3.3.4: `sudo apt install libeigen3-dev`
*  Point Cloud Library 1.8.0: [Download Link](https://www.dropbox.com/s/9llzm20pc4opdn9/PCL-1.8.0-Linux.deb?dl=0), [Linux Install Tutorial](https://larrylisky.com/2016/11/03/point-cloud-library-on-ubuntu-16-04-lts/)
  *  You might need to install missing dependencies first (flann, pcap, boost, opengl): `sudo apt install libflann-dev libpcap-dev libboost-all-dev libglu1-mesa-dev freeglut3-dev mesa-common-dev`

### Troubleshooting
*  If you cannot start the rosbridge_server (i.e. you get the *RLException: [rosbridge_server] is not a launch file name*), try installing the following python modules: `pip install rospkg bson pymongo`
*  If the client can connect to the rosbridge_server, but you none of the clients ROS topics are visible via `rostopic list`, make sure that you have **only** the correct version of tornado installed. For me `pip install tornado==4.5.3` worked fine ([more information here](https://github.com/RobotWebTools/rosbridge_suite/blob/develop/TROUBLESHOOTING.md)).

## Development

### Building the ros node
*  Go to the ros\_node workspace `cd ros_node`
*  Run `catkin_make`

### Running the ros node
Important parameters (e.g. the topic names) are set in the .launch file. Use the following commands to run the ros node: 

*  First terminal: `roscore`
*  Second terminal: `roslaunch rosbridge_server rosbridge_websocket.launch`
*  Third terminal: `source ./devel/setup.sh ; roslaunch hololens_robot_controller hololens_robot_controller.launch`

### Additional useful commands:
*  Replay rosbag file: `rosbag play <path_to_bagfile>`
*  View contents of rosbag file: `rosbag info <path_to_bagfile>`
*  View list of currently known topics: `rostopic list`
*  Show more information about a topic: `rostopic info <topic>`
*  Print published messages: `rostopic echo <topic>`
