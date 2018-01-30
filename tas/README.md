# TAS Package

This package contains the roslaunch files about navigation and the files concering the configuration of the system.

## Roslanch files

With roslaunch files we can start one or several ROS nodes at one time and configure 
necessary parameterts easily. The typical roslaunch files that we apply in our project 
are explained as follows.

### hardware_16.launch

All the neessary hardware devices being used in the navigation process, such as 
laser scanners, can be activated by this roslaunch file. It is always the first file to be launched.

hardware_16.launch can be launched with the following command

```
roslaunch tas hardware_16.launch
```

When adding a new device on the car, the corresponding ROS node for this device 
can be added into this roslaunch file by users.

### move_base.launch
move_base.launch reads in the map and initializes a ROS node for amcl. Operations 
such as changing the local planner or changing the initial position of the car can 
be exected within this file.

move_base.launch can be launched with the following command

```
roslaunch tas move_base.launch
```

### odom.launch
The roslaunch file odom.launch initializes a ROS node for hector_mapping, which 
is utilized to provide odometry for our car.

odom.launch can be launched with the following command

```
roslaunch tas odom.launch
```

## Folder map_server

This folder stores the map in form of an image and its corresponding configuration file.

If a new map is recorded and ready to be used for navigation, it can be added to the 
folder map_server with a special name. Besides, the user should also write a specific 
configuration file for the new map (e.g., New_map.pgm and New_map.yaml).

## Folder move_base

This folder consists of configuration files for the costmaps and the local planner.

These configuration files contain detailed parameters which users can change to their 
desired values according to the environment.

Here is a tuning guide for users: 
[ROS Navigation Tuning Guide](http://kaiyuzheng.me/documents/navguide.pdf)

Based on this tuning guide, users can get better understanding of the important parameters 
and get started more easily.

## Folder rviz

This folder mainly contains the rviz file which is loaded when the user wants to 
visualize the map and the real-time localization of the car in RViz.

## What we have done

In this package we added a new map which was recorded by ourselves, and we also 
tuned the parameters in those four configuration files concerning costmaps and local 
planners, in order to realize successful and smooth autonomous driving on our car.





























