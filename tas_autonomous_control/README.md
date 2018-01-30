# Package tas_autonomous_control

This package contains the C++ file which set velocity commands for the autonomous car.

## tas_autonomous_control_node.cpp
In this file, the velocity commands sent to the car are applied according to the 
real sotuations. The velocities for moving forward, moving backward and turnign left or right 
can be set individually. Normally, the value of 1500 represents the static state.

Users can change the values of velocities based on their requirement. The conditions 
for steering angles can also be changed, since the original condition is too loose 
for the mechanical capatity of the car.

## What we have done
In this part we mainly add more detailed conditions for steering angles and set the 
corresponding velocities higher. The velocities for moving forward and backward were
changed based on the mechanical behaviour of the car, since the performance of the car 
was not always stable.