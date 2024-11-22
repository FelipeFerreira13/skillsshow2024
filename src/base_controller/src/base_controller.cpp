/************************************
 * Author: Felipe Ferreira
 * Release version: 1.0.0.0
 * Release date: 27/06/2024
 * 
 * Modified by: Felipe Ferreira
 * Last modification date: 19/08/2024
 * New version: Add simple move

*************************************/

#include "base_controller.h"
 
// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "base_controller");

  ros::NodeHandle n;

  ros::ServiceServer simple_move_srv = n.advertiseService("base_controller/simple_move/goal", simple_move);

  ros::Subscriber odom_sub   = n.subscribe( "robot/position", 1, odomCallback );

  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);


  ros::spin();

  return 0;
}

bool simple_move( base_controller::move_goal::Request &req, base_controller::move_goal::Response &res ){

    bool reach_linear_tol  = false;
    bool reach_angular_tol = false;

    double desired_vx;
    double desired_vy;
    double desired_vth;

    ROS_INFO("d_x: %f", req.x);
    ROS_INFO("d_y: %f", req.y);
    ROS_INFO("d_th: %f", req.th);

    ROS_INFO("x: %f", x_global);
    ROS_INFO("y: %f", y_global);
    ROS_INFO("th: %f", th_global);

    ros::Rate loop_rate(5);

    do{
        ros::spinOnce();

        float desired_position[3] = { req.x, req.y, req.th };  // [m], [m], [degrees]
        float current_position[3] = { x_global , y_global, th_global };  // [m], [m], [degrees]

        float x_diff  = desired_position[0] - current_position[0];
        float y_diff  = desired_position[1] - current_position[1];
        float th_diff = desired_position[2] - current_position[2];

        if      ( th_diff < -180 ) { th_diff = th_diff + 360; }
        else if ( th_diff >  180 ) { th_diff = th_diff - 360; }

        double max_linear_speed = 0.10;   // m/s
        double max_ang_speed = 0.75;      // rad/s

        double linear_dist_offset  = 0.15;  // [m]
        double angular_dist_offset = 15;    // [degrees]


        desired_vx = (x_diff / linear_dist_offset) * max_linear_speed;
        desired_vx =  std::max( std::min( desired_vx, max_linear_speed ), -1 * max_linear_speed );
        if( abs(x_diff) < linear_tolerance ){ desired_vx = 0; }

        desired_vy = (y_diff / linear_dist_offset) * max_linear_speed;
        desired_vy =  std::max(  std::min( desired_vy, max_linear_speed ), -1 * max_linear_speed );
        if( abs(y_diff) < linear_tolerance ){ desired_vy = 0; }

        desired_vth = (th_diff / angular_dist_offset) * max_ang_speed; 
        desired_vth =  std::max(  std::min( desired_vth, max_ang_speed ), -1 * max_ang_speed );
        if( abs(th_diff) < angular_tolerance ){ desired_vth = 0; }

        rotate( desired_vx, desired_vy, -(th_global/180) * PI ); // Rotate from Global Frame to Local Frame

        ROS_INFO("x: %f, y: %f, th: %f", x_global, y_global, th_global);

        geometry_msgs::Twist cmd;
        cmd.linear.x  = desired_vx;
        cmd.linear.y  = desired_vy;
        cmd.angular.z = desired_vth;
        cmd_vel_pub.publish( cmd );
        
        loop_rate.sleep();

    }while( desired_vx != 0 || desired_vy != 0 || desired_vth != 0 );

}  

void rotate( double & x, double & y, double phi){
  float x_ = (cos(phi) * x) + (sin(phi) * y * -1);
  float y_ = (sin(phi) * x) + (cos(phi) * y);
  x = x_;
  y = y_;
}

void odomCallback( const geometry_msgs::Vector3::ConstPtr& msg ){
  x_global  = msg->x;
  y_global  = msg->y;
  th_global = msg->z;
}