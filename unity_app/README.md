# Hololens Robot Controller - Unity App
## Prerequisites
* Unity 2018.x.x or later (Developed with 2018.3.11f1)
* Microsoft Visual Studio (Developed with Community 2017 Edition)

## Reminder on external libraries and assets
* You do not have to install anything under this section by yourself, necessary files are already provided as in Assets folder.
* Communication between our Unity App and ROS: [ROS# for UWP](https://github.com/dwhit/ros-sharp) (which is based on [the original ros-sharp library](https://github.com/siemens/ros-sharp))
* Marker detection on Hololens: [HoloLensARToolKit](https://github.com/qian256/HoloLensARToolKit)
* **[WARNING!]** HoloToolkit-Examples folder is large and not utilized except shaders and some game objects at the moment. It needs to be cleaned up by checking the dependencies in the future. Apologies...

## Building the Unity project
* Open the Unity project (select the `unity_app/HololensRobotController` directory)
* In the menu bar, click on `Mixed Reality Toolkit -> Configure -> Apply Mixed Reality Project Settings`
* In the same menu, click on `Mixed Reality Toolkit -> Configure -> Apply UWP Capability Settings`
* Create the Visual Studio project by building the App via `File -> Build Settings... -> Build` (Check `Unity C# Projects`)
* In the file dialog, select the `unity_app/HololensRobotController/App` directory, create the directory if it does not exist.
* After build is finished, open the Visual Studio solution located at `unity_app/HololensRobotController/App/HololensRobotController.sln` (be careful with the directory, the correct solution is in the App folder, not the one in the root.)

## Deploying the application from Visual Studio
* 

