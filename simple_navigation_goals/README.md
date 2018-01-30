# Package simple_navigation_goals

- This packages contains the waypoints setting for both tasks, which are used as the intermediate goals during the autonomous driving. 
- It also includes different nodes for different waypoints setting strategies, both mannual and adaptive ones. 

- It includes several `main.cpp` which linked to different nodes:
	- **main.cpp** --> ***simple_navigation_goals***: used for task 1, just change the value of `std::vector<geometry_msgs::Pose> waypoints`
	- **main_t2side.cpp** --> ***t2side_navigation_goals***: task 2 but with waypoints one step beside the traffic cones on left and right
	- **main_t2middle.cpp** --> ***t2middle_navigation_goals***: task 2 with waypoints in middle of traffic cones
	- **main_adp.cpp** --> ***adaptive_navigation_goals***: task 2 with adaptive waypoints which calulated by laser point cloud data. This node will subscribe the `/scan` topic from LIDAR node `huyoko_front` and process the point cloud data at first and then send the waypoints information to `move_base`.