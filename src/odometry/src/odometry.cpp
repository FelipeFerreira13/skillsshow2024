/************************************
 * Author: Felipe Ferreira
 * Release version: 1.0.0.0
 * Release date: 27/06/2024
 * 
 * Modified by: Felipe Ferreira
 * Last modification date: 08/07/2024
 * New version: 1.0.0.1

*************************************/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>

#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Imu.h>

#include "odometry/pose_odom.h"

static const double PI = 3.14159265;

static double x = 0.0;
static double y = 0.0;
static double th = 0.0;

static double vx = 0.0;
static double vy = 0.0;
static double vth = 0.0;

static double thDiff = 0;

static double navxAngle = 0;
static double navxDiff = 0;

void setPosition( double x, double y, double th);
double Quotient_Remainder( double x, double y );
double ToDegrees(double x);
double ToRadian(double x);
double AngleLogic( double x, double ref );
void rotate( double x, double y, double phi, double *v);

bool reference( odometry::pose_odom::Request &req, odometry::pose_odom::Response &res );

void vx_Callback(const std_msgs::Float32::ConstPtr& msg){ vx = msg->data; }
void vy_Callback(const std_msgs::Float32::ConstPtr& msg){ vy = msg->data; }
void vth_Callback(const std_msgs::Float32::ConstPtr& msg){ vth = msg->data; }

void angleCallback(const std_msgs::Float32::ConstPtr& msg){navxAngle = msg->data * -1;}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry");

  ROS_INFO("odometry node is now started");

  ros::NodeHandle n;
  
  ros::Subscriber vx_sub  = n.subscribe("vx_local",  1, vx_Callback );
  ros::Subscriber vy_sub  = n.subscribe("vy_local",  1, vy_Callback );
  ros::Subscriber vth_sub = n.subscribe("vth_local", 1, vth_Callback);

  ros::Subscriber angle_sub  = n.subscribe("navx/angle", 1, angleCallback);

  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 5);
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu/data", 1);
  ros::Publisher robot_pos = n.advertise<geometry_msgs::Vector3>("robot/position", 1);

  ros::ServiceServer service = n.advertiseService("odometry/set_position", reference);
  
  tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::Quaternion odom_quat;
  sensor_msgs::Imu imu_data;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  double last_x = 0;
  double last_y = 0;
  double last_th = 0;

  ros::Duration(5).sleep();

  setPosition(0.0, 0.0, 0.0); //Set initial Position

  ros::Rate r(10);
  while(n.ok()){

    ros::spinOnce(); // check for incoming messages
    current_time = ros::Time::now();

    double dt = (current_time - last_time).toSec();

    th = Quotient_Remainder((navxAngle + navxDiff), 360);

    double v[2] = {0, 0};
    rotate(vx, vy, (th * (PI/180.0)), v);

    x  = x  + ( v[0] * dt );
    y  = y  + ( v[1] * dt );

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf::createQuaternionMsgFromYaw((PI/180) * th);

    geometry_msgs::Vector3 pos;
    pos.x = x;
    pos.y = y;
    pos.z = th;
    robot_pos.publish( pos );

    imu_data.header.stamp = current_time;
    imu_data.header.frame_id = "imu_link";
    
    imu_data.orientation = odom_quat;
    imu_pub.publish(imu_data);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = (x - last_x) / dt;
    odom.twist.twist.linear.y = (y - last_y) / dt;
    double th_velocity = (PI/180) * AngleLogic(Quotient_Remainder( ( th - last_th ), 360 ), 360) / dt;
    odom.twist.twist.angular.z = th_velocity;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    last_x = x;
    last_y = y;
    last_th = th;

    r.sleep();
  }
}

bool reference( odometry::pose_odom::Request &req, odometry::pose_odom::Response &res ){

  ROS_INFO("Robot Position: %f, %f, %f", req.x, req.y, req.th);

  setPosition( req.x, req.y, req.th );

  return true;
}

//Set a robot position (x[m], y[m] and theta[°])
void setPosition( double set_x, double set_y, double set_th){

  x = set_x;
  y = set_y;

  navxDiff = (set_th - navxAngle);

}

double Quotient_Remainder( double x, double y ){
  double Quotient = floor( x / y );
  double Remainder = x - (y * Quotient);

  return Remainder;
}

double ToDegrees(double x){
  return ( (x / M_PI) * 180);
}

double ToRadian(double x){
  return ( (x * M_PI) / 180);
}

double AngleLogic( double x, double ref ){
  if ( x > ref/2 ) { return (x - ref); }
  else if ( x < (-1 * ref/2) ) { return (x + ref); }
  else { return x; }
}

void rotate( double x, double y, double phi, double *v){
  v[0] = (cos(phi) * x) + (sin(phi) * y * -1);
  v[1] = (sin(phi) * x) + (cos(phi) * y);
}