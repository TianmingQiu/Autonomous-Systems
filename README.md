# Introduction to Autonomous Systems  (Technik Autonomer Systeme)

Tuesday, 30. January 2018

Leture in TU MUnich in winter semester 2017/18

Supvisor: [ PD Dr.-Ing. habil. Dirk Wollherr ](https://www.lsr.ei.tum.de/en/team/dozenten/dirk-wollherr/)  
Tutor: [M.Sc. Khoi Hoang Dinh](https://www.lsr.ei.tum.de/?id=419), and [M.Sc. Zengjie Zhang](https://www.lsr.ei.tum.de/en/team/wissenschaftliche-mitarbeiter/zengjie-zhang/)

**Group 12: Tianming Qiu, Zhangyi Hu**

## Task Description:
- In this project we use a remote control car with LIDARs to implement autonomous driving tasks.
- Task 1: AUTONOMOUS DRIVING: drive the car around the base floor hallways autonomously based on a given map using AMCL.
- Task 2: SLALOM: pass through four traffic cones smoothly.

## Code Explain:
- The operating system currently running on the linux board of the car is Ubuntu 16.04. And we use ROS to control the hardware. The ROS [Navigation Stack](wiki.ros.org/navigation) will be used as a platform.

- **Package 1: simple_navigation_goals**:  This packages contains the waypoints setting for both tasks, which are used as the intermediate goals during the autonomous driving. And include different nodes for different waypoints setting strategies, both mannual and adaptive ones.  
It includes several nodes:
	- *simple_navigation_goals*: used for task 1
	- *t2side_navigation_goals*: task 2 but with waypoints one step beside the traffic cones on left and right
	- *t2middle_navigation_goals*: task 2 with waypoints in middle of traffic cones
	- *adaptive_navigation_goals*: task 2 with adaptive waypoints which calulated by laser point cloud data
	
- **Package 2: tas**: Some launch files for hardware calling, interface to *rviz* and load the map.

- **Package 3: tas_autonomous_control**: Stratigies for velocity and steering control.

- **Package 4: wii_control**: Used for wii remote control communication, but later we donnot ues wii any longer.


## Contribution:

| Contribution      | Tianming Qiu        | Zhangyi Hu  |
| ------------- |:-------------:| :-----:|
| Map Modification   | +++ | + |
| Navigation through Intermediate Goals   | +     | ++ |
|Setting Wayside Landmarks | +    |  ++|
|Tuning Parameters | +   |   +++ |
|Adaptive Waypoints| +++    |  ++ |
|Trailer Behavior Analysis|+ |+++ |

We also share some useful references in the **GitLab Wiki** for group discussing.