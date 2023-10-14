# ARDVARC Unity Simulator
## Introduction
This directory contains the source code for the ARDVARC Unity simulator as well as the most recent builds.

## Setup
This section concerns how to set up your environment to either develop or use the ARDVARC Unity simulator. Certain subsections (marked as `Development`) are not required if you only wish to use the simulator.
### Clone Repository
As always, start by cloning this repository onto your local machine. This can be done with a command such as the following:
```
git clone https://github.com/ARDVARC/ARDVARC.git
```
### Unity Setup (Development)
This subsection is only required if you intend to develop the ARDVARC Unity simulator.

As this simulator is built in Unity, development of the simulator requires a working installation of the Unity editor. To install the Unity editor, it is recommended that you download and install [Unity Hub](https://unity.com/download), a GUI tool that makes managing Unity projects and editors very straightforward.

In the "Installs" tab of Unity Hub, choose to install Unity editor 2022.3.10f1.

Finally, in the "Projects" tab, choose "Open > Add project from disk" and select the `ARDVARC_Unity_Project` folder from the cloned git repository.

## ROS Setup
Getting the ARDVARC Unity simulator to work with ROS requires both a valid ROS setup as well as two specific ROS packages. To get a valid ROS setup on your OS, consult either the [ROS Installation](https://wiki.ros.org/ROS/Installation) page or the [ROS Download Tutorial Document](../../ROS.md).

The first of the two required packages is the [ROS TCP Endpoint package](https://github.com/Unity-Technologies/ROS-TCP-Endpoint). To add this package to your ROS environment, clone the repo into the `src` directory of your catkin workspace using a command such as the following:
```
cd ~/catkin_ws/src && git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
```
Once the package is cloned, run the following commands to build the package and set everything else up:
```
cd ~/catkin_ws/ && catkin_make && source devel/setup.bash
```
I'd also recommend adding the `source devel/setup.bash` command to your `.bashrc`.

For more details on setting up the ROS TCP Endpoint package, consult the [ROS–Unity Demo Setup](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/setup.md#-ros-environment).

The second required package is our very own `ros_ardvarc_unity_sim` package. Like before, to add this package to your ROS environment, clone the repo into the `src` directory of your catkin workspace using a command such as the following:
```
cd ~/catkin_ws/src && git clone https://github.com/ARDVARC/ros_ardvarc_unity_sim.git
```
Once the package is cloned, run the following commands to build the package:
```
cd ~/catkin_ws/ && catkin_make
```

### Test Setup (Development)
This subsection is only required if you intend to develop the ARDVARC Unity simulator.

To test if your environment is properly configured to develop the simulator, run the following command:
```
roslaunch ros_tcp_endpoint endpoint.launch
```
Ideally that'd just work but probably it'll complain about some stuff, see [Troubleshooting](#troubleshooting).

Once you have that working, open the ARDVARC Unity project through Unity Hub and run it in the Unity editor. You should see a response in the `ros_tcp_endpoint` node.

### Test Setup (Users)
To test if your environment is properly configured to run the simulator, run the following command:
```
roslaunch ros_tcp_endpoint endpoint.launch
```
Ideally that'd just work but probably it'll complain about some stuff, see [Troubleshooting](#troubleshooting).

Once you have that working, open the most recent build of the simulator in the repo (under `\Unity_Simulator\Builds`) and run the `.exe` file. You should see a response in the `ros_tcp_endpoint` node.

## Troubleshooting
I've tried to set this all up two times on different machines and both times I got the same couple of complaints about missing Python modules. I found [a stack overflow answer with zero upvotes](https://stackoverflow.com/a/74409815) that gives the following solution, which worked for me both times:
```
sudo apt install python-yaml && sudo apt install python-is-python3
```