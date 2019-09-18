# Hololens Robot Controller - Unity App
## Prerequisites
*  Unity 2018

## First start
*  Open the Unity project (select the `unity_app/HololensRobotController` directory)
*  In the menu bar, click on `Mixed Reality Toolkit -> Configure -> Apply Mixed Reality Project Settings`
*  In the same menu, click on `Mixed Reality Toolkit -> Configure -> Apply UWP Capability Settings`
*  Create the Visual Studio project by building the App via `File -> Build Settings... -> Build`
*  In the file dialog, select the `unity_app/HololensRobotController/App` directory
*  After building is finished, open the Visual Studio solution located at `unity_app/HololensRobotController/App/Hololens Robot Controller.sln`

## Code used
*  Communication between our Unity App and ROS: [ROS# for UWP](https://github.com/dwhit/ros-sharp) (which is based on [the original ros-sharp library](https://github.com/siemens/ros-sharp))
*  Accessing the hololens sensors: [Jeff's Code](https://gitlab.inf.ethz.ch/OU-POLLEFEYS/student-projects/3d-vision-course/2019/trimbot_hololens_group3/hololens_csharp_sensors_example)
*  Marker detection on Hololens: [HoloLensARToolKit](https://github.com/qian256/HoloLensARToolKit)
