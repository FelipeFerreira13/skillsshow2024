/************************************
 * Author: Felipe Ferreira
 * Release version: 1.0.0.0
 * Release date: 27/06/2024
 * 
 * Modified by: Felipe Ferreira
 * Last modification date: 08/07/2024
 * New version:

*************************************/

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <tf/transform_broadcaster.h>

#include "base_controller/move_goal.h"
 
// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

geometry_msgs::Quaternion odom_quat;
static const double PI = 3.14159265;

bool move( base_controller::move_goal::Request &req, base_controller::move_goal::Response &res );

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "base_controller");

  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("base_controller/goal", move);

  ros::spin();

  return 0;
}

bool move( base_controller::move_goal::Request &req, base_controller::move_goal::Response &res ){
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  geometry_msgs::Quaternion SetTheta(double theta);
  
  // set up the frame parameters
  goal.target_pose.header.frame_id = "odom";
  goal.target_pose.header.stamp = ros::Time::now();
  
  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = req.x;
  goal.target_pose.pose.position.y = req.y;
  goal.target_pose.pose.orientation = SetTheta(req.th);

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  
  // Wait an infinite time for the results
  ac.waitForResult();
  
  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, Robot reached the goal");
  else
  {
    ROS_INFO("The base failed to reach first goal for some reason");
    return 0;
  }
}

geometry_msgs::Quaternion SetTheta(double theta){
  theta = theta * ( PI / 180);  //Degrees to RADs
  odom_quat = tf::createQuaternionMsgFromYaw( theta );
  return odom_quat;
}