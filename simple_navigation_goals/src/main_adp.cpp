/**
 * This node sends fixed goals to move base via ROS Action API and receives feedback via callback functions.
 */

#include <iostream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf/tf.h"


using namespace std;

geometry_msgs::Pose slalom_position;
double car_map_x = 0.0;
double car_map_y = 0.0;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/**
 * Callback function
 */
void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
}

/**
 * Callback function, called once when the goal becomes active
 */
void activeCb() {
    ROS_INFO("Goal just went active");
}

/**
 * Callback function, called every time feedback is received for the goal
 */
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
    ROS_INFO("[X]:%f [Y]:%f [W]: %f [Z]: %f", feedback->base_position.pose.position.x,feedback->base_position.pose.position.y,feedback->base_position.pose.orientation.w, feedback->base_position.pose.orientation.z);
}


void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan_data){
    /*double scan_raw_data[1081];
    for(int i = 0; i<1081; i ++){
        scan_raw_data[i] = scan_data->ranges[i];
    }*/


    geometry_msgs::Pose slalom_position;
    double scan_data_def[720];
    int scan_bias = 180;

    // Calculate the differnce squence (remove the head and tail, only keep the 180 degrees data)
    for(int i = 0; i < 721; i++){
        
        scan_data_def[i] = scan_data->ranges[scan_bias + i + 1] - scan_data->ranges[scan_bias + i];
    }
    
    double max = scan_data_def[0];
    double min = scan_data_def[0];
    double max_dist = scan_data->ranges[0];
    double min_dist = scan_data->ranges[0];
    
    // Find the max and min
    for(int i = 0; i < 721; i++){
        if (scan_data_def[i] > max){
            max = scan_data_def[i];
            max_dist = scan_data->ranges[scan_bias + i];
        }
        if (scan_data_def[i] < min){
            min = scan_data_def[i];
            min_dist = scan_data->ranges[scan_bias + i + 1];
        }
    }
    
    slalom_position.position.x = 0.5 * (max_dist + min_dist);
    cout << "Distance between laser scanner and slalom_position:" << slalom_position.position.x << endl;
}


void map_frame(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial_pose){
	car_map_x = initial_pose->pose.pose.position.x;
	car_map_y = initial_pose->pose.pose.position.y;
	cout << "car initialpose x: " << initial_pose->pose.pose.position.x << endl;
	cout << "car initialpose y: " << initial_pose->pose.pose.position.y << endl;
}


/**
 * Main function
 */
int main(int argc, char** argv){
    ros::init(argc, argv, "adaptive_navigation_goals"); // init and set name
    std::vector<geometry_msgs::Pose> waypoints; // vector of goals, with position and orientation

    /********************************************/

    // based on frame /laser, same direction as the car:
    geometry_msgs::Pose slalom_position;
    slalom_position.position.x = 0;
    slalom_position.position.y = 0;
    slalom_position.position.z = 0;
    slalom_position.orientation.x = 0;
    slalom_position.orientation.y = 0;
    slalom_position.orientation.z = 0;
    slalom_position.orientation.w = 1;

    // based on frame /map:
    geometry_msgs::Pose slalom_position_map;
    slalom_position_map.position.x = 0;
    slalom_position_map.position.y = 0;
    slalom_position_map.position.z = 0;
    slalom_position_map.orientation.x = 0;
    slalom_position_map.orientation.y = 0;
    slalom_position_map.orientation.z = 0; //0.692
    slalom_position_map.orientation.w = 1; //0.722

    // 

    ros::NodeHandle n;

    ros::Subscriber scan_sub = n.subscribe("/scan", 1000, processLaserScan);

    sleep(1);
    ros::spinOnce();
    ros::Subscriber map_sub = n.subscribe("/initialpose", 1000, map_frame);

    sleep(30);
    ros::spinOnce();

    /********************************************/

    /*******************************************/
    // Create tf listener (from laser to base_link):
    /*tf::TransformListener tf_listener;

    int count = 5;

    ros::Rate rate(10.0);
    while (n.ok()){
        tf::StampedTransform transform;
        try{
            tf_listener.lookupTransform("/base_link","/laser",ros::Time(0),transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        // In the long corridor, use:
        slalom_position_map.position.x = car_map_x + slalom_position.position.x + transform.getOrigin().x();
        slalom_position_map.position.y = car_map_y + slalom_position.position.y + transform.getOrigin().y();

        // In the middle corridor, use:
        //slalom_position_map.position.x = car_map_y + slalom_position.position.x + transform.getOrigin().x();
        //slalom_position_map.position.y = car_map_x + slalom_position.position.y + transform.getOrigin().y();

        rate.sleep();

        count--;

        if (count == 0)
            break;
    }*/

    /*******************************************/
    // In the long corridor, use:
    slalom_position_map.position.x = car_map_x + slalom_position.position.x - 0.28;
    slalom_position_map.position.y = car_map_y + slalom_position.position.y;


    cout << "slalom_position_map: " << slalom_position_map << endl;


    // Calculate the waypoints accoding to the above computed coordinate:
    // Orientation of the intermediate goals are kept unchanged and parallel to the X-axis of frame /map;
    geometry_msgs::Pose waypoint1;
    waypoint1.position.x = slalom_position_map.position.x + 0.75;
    waypoint1.position.y = slalom_position_map.position.y; // set at the middle between 1st and 2nd slalom
    waypoint1.position.z = 0.000;
    waypoint1.orientation.x = 0;
    waypoint1.orientation.y = 0;
    waypoint1.orientation.z = slalom_position_map.orientation.z;
    waypoint1.orientation.w = slalom_position_map.orientation.w;
    waypoints.push_back(waypoint1);

    geometry_msgs::Pose waypoint2;
    waypoint2.position.x = waypoint1.position.x +1.50;
    waypoint2.position.y = slalom_position_map.position.y; // translation with value of 1.50m to next goal
    waypoint2.position.z = 0.000;
    waypoint2.orientation.x = 0;
    waypoint2.orientation.y = 0;
    waypoint2.orientation.z = slalom_position_map.orientation.z;
    waypoint2.orientation.w = slalom_position_map.orientation.w;
    waypoints.push_back(waypoint2);

    geometry_msgs::Pose waypoint3;
    waypoint3.position.x = waypoint2.position.x + 1.50;
    waypoint3.position.y = slalom_position_map.position.y; // translation with value of 1.50m to next goal
    waypoint3.position.z = 0.000;
    waypoint3.orientation.x = 0;
    waypoint3.orientation.y = 0;
    waypoint3.orientation.z = slalom_position_map.orientation.z;
    waypoint3.orientation.w = slalom_position_map.orientation.w;
    waypoints.push_back(waypoint3);

    geometry_msgs::Pose waypoint4;
    waypoint4.position.x = waypoint3.position.x + 1.50;
    waypoint4.position.y = slalom_position_map.position.y; // translation with value of 1.50m to next goal
    waypoint4.position.z = 0.000;
    waypoint4.orientation.x = 0;
    waypoint4.orientation.y = 0;
    waypoint4.orientation.z = slalom_position_map.orientation.z;
    waypoint4.orientation.w = slalom_position_map.orientation.w;
    waypoints.push_back(waypoint4);

    

    MoveBaseClient ac("move_base", true); // action client to spin a thread by default

    while (!ac.waitForServer(ros::Duration(5.0))) { // wait for the action server to come up
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map"; // set target pose frame of coordinates

    for(int i = 0; i < waypoints.size(); ++i) { // loop over all goal points, point by point
        goal.target_pose.header.stamp = ros::Time::now(); // set current time
        goal.target_pose.pose = waypoints.at(i);
        ROS_INFO("Sending goal");
        ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); // send goal and register callback handler
        ac.waitForResult(); // wait for goal result

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("The base moved to %d goal", i);
        } else {
            ROS_INFO("The base failed to move to %d goal for some reason", i);
        }
    }
    return 0;
}
