/************************************
 *  Author: Felipe Ferreira
 * Release version: 1.0.0.0
 * Release date: 27/06/2024
 * 
 * Modified by: Felipe Ferreira
 * Last modification date: 02/10/2024
 * New version: 1.0.1.0

*************************************/

#include "main.h"

int main(int argc, char **argv)
{
    ros::init(argc,argv,"main");
    ros::NodeHandle nh;

    ROS_INFO("main node is now started");

    move_goal_c    = nh.serviceClient<base_controller::move_goal> ("base_controller/goal" );
    set_position_c = nh.serviceClient<odometry::pose_odom>        ("odometry/set_position");
    set_height_c   = nh.serviceClient<oms::set_height>            ("oms/set_height"       );
    reset_height_c = nh.serviceClient<oms::reset>                 ("oms/reset"            );
    set_gripper_c  = nh.serviceClient<oms::set_gripper>           ("oms/set_gripper"      );

    ros::Subscriber start_sub = nh.subscribe("/robot/digital_in/start_button/state", 1, startCallback);


    ros::Duration(10).sleep();


    do{ 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }while ( start_button );


    // Main Logic

    // reset_height( -1 );

    // oms_driver( 30 );

    // set_gripper( GRIPPER_OPEN );

    // set_position( 0.3, 0.3, 90 );  

    // position_driver( 0.3, 1.0, 90 );

    // oms_driver( 20 );

    // set_gripper( GRIPPER_CLOSE );


    ros::shutdown();

    return 0;
};
