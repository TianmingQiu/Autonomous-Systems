# TAS Package

This package contains the roslaunch files about navigation and the files concering the configuration of the system.

## Roslanch files

With roslaunch files we can start one or several ROS nodes at one time and configure necessary parameterts easily. The typical roslaunch files that we apply in our project are explained as follows.

### hardware_16.launch

All the neessary hardware devices being used in the navigation process, such as laser scanners, can be activated by this roslaunch file. It is always the first file to be launched.

hardware_16.launch can be launched with the following command

```
roslaunch tas hardware_16.launch
```

When adding a new device on the car, the corresponding ROS node for this device can be added into this roslaunch file by users.

### move_base.launch
move_base.launch reads in the map and initializes a ROS node for amcl.

move_base.launch can be launched with the following command

```
roslaunch tas move_base.launch
```