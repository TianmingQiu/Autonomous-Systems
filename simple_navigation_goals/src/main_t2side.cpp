/**
 * This node sends fixed goals to move base via ROS Action API and receives feedback via callback functions.
 */

#include <iostream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;

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

/**
 * Main function
 */
int main(int argc, char** argv){
    ros::init(argc, argv, "t2side_navigation_goals"); // init and set name
    std::vector<geometry_msgs::Pose> waypoints; // vector of goals, with position and orientation

    geometry_msgs::Pose waypoint1;//left to the first marker
    waypoint1.position.x = 16;
    waypoint1.position.y = 7.154;
    waypoint1.position.z = 0.000;
    waypoint1.orientation.x = 0;
    waypoint1.orientation.y = 0;
    waypoint1.orientation.z = 0.692;
    waypoint1.orientation.w = 0.722;
    waypoints.push_back(waypoint1);

    geometry_msgs::Pose waypoint2;//right to the second marker
    waypoint2.position.x = 18;
    waypoint2.position.y = 8.808;
    waypoint2.position.z = 0.0;
    waypoint2.orientation.x = 0;
    waypoint2.orientation.y = 0;
    waypoint2.orientation.z = 0.691;
    waypoint2.orientation.w = 0.723;
    waypoints.push_back(waypoint2);

    geometry_msgs::Pose waypoint3;//left to the third marker
    waypoint3.position.x = 16;
    waypoint3.position.y = 10.932; //11.093
    waypoint3.position.z = 0.000;
    waypoint3.orientation.x = 0;
    waypoint3.orientation.y = 0;
    waypoint3.orientation.z = 0.692;
    waypoint3.orientation.w = 0.722;
    waypoints.push_back(waypoint3);

    geometry_msgs::Pose waypoint4;//right to the fourth marker
    waypoint4.position.x = 18;
    waypoint4.position.y = 12.062;
    waypoint4.position.z = 0.000;
    waypoint4.orientation.x = 0;
    waypoint4.orientation.y = 0;
    waypoint4.orientation.z = 0.724;
    waypoint4.orientation.w = 0.690;
    waypoints.push_back(waypoint4);

    geometry_msgs::Pose waypoint5;//after the last cone
    waypoint5.position.x = 17;
    waypoint5.position.y = 13.562;
    waypoint5.position.z = 0.000;
    waypoint5.orientation.x = 0;
    waypoint5.orientation.y = 0;
    waypoint5.orientation.z = 0.724;
    waypoint5.orientation.w = 0.690;
    waypoints.push_back(waypoint5);

    /*geometry_msgs::Pose waypoint5;//enter the room straightly
    waypoint5.position.x = 17.933;
    waypoint5.position.y = 12.844;
    waypoint5.position.z = 0.000;
    waypoint5.orientation.x = 0;
    waypoint5.orientation.y = 0;
    waypoint5.orientation.z = 0;
    waypoint5.orientation.w = 1;
    waypoints.push_back(waypoint5);

    geometry_msgs::Pose waypoint6;//do not turn right in room
    waypoint6.position.x = 20.786;
    waypoint6.position.y = 12.366;
    waypoint6.position.z = 0.000;
    waypoint6.orientation.x = 0;
    waypoint6.orientation.y = 0;
    waypoint6.orientation.z = 0.012;
    waypoint6.orientation.w = 1;
    waypoints.push_back(waypoint6);

    geometry_msgs::Pose waypoint7;//middle of the room
    waypoint7.position.x = 18.817;
    waypoint7.position.y = 12.808;
    waypoint7.position.z = 0.000;
    waypoint7.orientation.x = 0;
    waypoint7.orientation.y = 0;
    waypoint7.orientation.z = 0.009;
    waypoint7.orientation.w = 1;
    waypoints.push_back(waypoint7);

    geometry_msgs::Pose waypoint8;//ready to leave the room
    waypoint8.position.x = 19.774;
    waypoint8.position.y = 3.839;
    waypoint8.position.z = 0.000;
    waypoint8.orientation.x = 0;
    waypoint8.orientation.y = 0;
    waypoint8.orientation.z = -0.961;
    waypoint8.orientation.w = 0.278;
    waypoints.push_back(waypoint8);

    geometry_msgs::Pose waypoint9;//leave the room straightly
    waypoint9.position.x = 18.106;
    waypoint9.position.y = 3.099;
    waypoint9.position.z = 0.000;
    waypoint9.orientation.x = 0;
    waypoint9.orientation.y = 0;
    waypoint9.orientation.z = -1;
    waypoint9.orientation.w = 0;
    waypoints.push_back(waypoint9);

    geometry_msgs::Pose waypoint5;//bottom of U
    waypoint5.position.x = 16.840;
    waypoint5.position.y = 8.301;
    waypoint5.position.z = 0.000;
    waypoint5.orientation.x = 0;
    waypoint5.orientation.y = 0;
    waypoint5.orientation.z = 0.704;
    waypoint5.orientation.w = 0.710;
    waypoints.push_back(waypoint5);


    geometry_msgs::Pose waypoint6;//on the upper corner
    waypoint6.position.x = 17.122;
    waypoint6.position.y = 14.105;
    waypoint6.position.z = 0.000;
    waypoint6.orientation.x = 0;
    waypoint6.orientation.y = 0;
    waypoint6.orientation.z = 0.789;
    waypoint6.orientation.w = 0.614;
    waypoints.push_back(waypoint6); 

    geometry_msgs::Pose waypoint7;//destination
    waypoint7.position.x = 10.009;
    waypoint7.position.y = 15.615;
    waypoint7.position.z = 0.000;
    waypoint7.orientation.x = 0;
    waypoint7.orientation.y = 0;
    waypoint7.orientation.z = 1;
    waypoint7.orientation.w = 0.014;
    waypoints.push_back(waypoint7);     

    geometry_msgs::Pose waypoint13;
    waypoint13.position.x = 14.276;
    waypoint13.position.y = 15.412;
    waypoint13.position.z = 0.000;
    waypoint13.orientation.x = 0;
    waypoint13.orientation.y = 0;
    waypoint13.orientation.z = 1;
    waypoint13.orientation.w = 0.019;
    waypoints.push_back(waypoint13); 
*/
    
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
