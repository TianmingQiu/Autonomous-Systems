### This is the TAS package for the cars. Please refer to the TAS tutorial for specific information on installation and initialization.
====

## Repository structure

#### \simple_navigation_goals		```ROS node for autonomous goal selection```
 	\src
	\include
	CMakeLists.txt
	package.xml
#### \tas_automonous_control			```ROS node for control command transmission```
	\src
	CMakeLists.txt
	package.xml
#### \wii_control				```ROS node for wii controller```
	\src
	CMakeLists.txt
	package.xml
#### \tas
	\launch				```Main launch files```
		\config			```Configuration files```
		hardware.launch
		move_base_slam.launch
		odom_slam.launch
	\shell_scripts
	CMakeLists.txt
	package.xml
