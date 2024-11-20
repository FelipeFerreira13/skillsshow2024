/************************************
 * Author: Felipe Ferreira
 * Release version: 1.0.0.0
 * Release date: 16/08/2024
 * 
 * Modified by: Felipe Ferreira
 * Last modification date: 
 * New version:

*************************************/

#include "object_management.h"

int main(int argc, char **argv)
{
    system("/usr/local/frc/bin/frcKillRobot.sh"); //Terminal call to kill the robot manager used for WPILib before running the executable.
    ros::init(argc, argv, "oms_node");

    ros::NodeHandle nh; //Internal reference to the ROS node that the program will use to interact with the ROS system

    ros::ServiceServer service_driver  = nh.advertiseService("oms/set_height",   oms_driver);
    ros::ServiceServer service_reset   = nh.advertiseService("oms/reset",         oms_reset);
    ros::ServiceServer service_gripper = nh.advertiseService("oms/set_gripper", set_gripper);

    setAngle     = nh.serviceClient<vmxpi_ros::Float>("channel/16/servo/set_angle");
    setAngleElev = nh.serviceClient<vmxpi_ros::Float>("channel/17/servo/set_angle");


    ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 1, cmd_vel_callback);

    ros::Subscriber limit_high_sub = nh.subscribe("/robot/digital_in/limit_switch_high/state", 1, limit_high_Callback);
    ros::Subscriber limit_low_sub  = nh.subscribe("/robot/digital_in/limit_switch_low/state",  1, limit_low_Callback );

    ros::Subscriber stop_sub  = nh.subscribe("channel/10/digital_in/state", 1, stop_Callback );

    set_m_pwm      = nh.advertise<std_msgs::Float32>("motor/3/set_motor_pwm", 1);

    get_height_pub = nh.advertise<std_msgs::Float32>("oms/height", 1);

    ros::spin();
    return 0;
}


bool oms_driver( oms::set_height::Request &req, oms::set_height::Response &res ){

    vmxpi_ros::Float msg;

    ros::Rate loop_rate(10);

    if( req.height == 1 ){
        while( limit_high_state ){ 
            ros::spinOnce();
            loop_rate.sleep(); 
            msg.request.data = -25;
            setAngleElev.call(msg);
        }
    }else if (req.height == -1){
        while( limit_low_state ){ 
            ros::spinOnce();
            loop_rate.sleep();
            msg.request.data = 25;
            setAngleElev.call(msg);
        }
    }
    msg.request.data = 0;
    setAngleElev.call(msg);
}

// -1 to send it down and 1 to send it up
bool oms_reset( oms::reset::Request &req, oms::reset::Response &res ){
    
    ros::Rate loop_rate(5);

    if( req.direction == 1 ){
        elevatorMotor( 0.35 );
        while( limit_high_state ){ 
            ros::spinOnce();
            loop_rate.sleep(); 
        }
        height = high_height;
        ROS_INFO("new height is %f", height);
        elevatorMotor( 0 );
        return true;
    }else if (req.direction == -1){
        elevatorMotor( -0.35 );
        while( limit_low_state ){ 
            ros::spinOnce();
            loop_rate.sleep();
        }
        height = low_height;
        ROS_INFO("new height is %f", height);
        elevatorMotor( 0 );
        return true;
    }else{
        return false;
    }
}

bool set_gripper( oms::set_gripper::Request &req, oms::set_gripper::Response &res ){
    vmxpi_ros::Float msg;
    msg.request.data = req.angle;
    setAngle.call( msg );

    return true;
}

void elevatorMotor( double pwm ){
    std_msgs::Float32 msg;
    msg.data = pwm;
    set_m_pwm.publish(msg);
}

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& cmd_vel){
    float cmd_vel_z = cmd_vel->linear.z;

    vmxpi_ros::Float msg;

    if ( cmd_vel_z != prev_cmd_z ){
        float pwm = (cmd_vel_z * 100) / 15;

        if( cmd_vel_z > 0 && limit_high_state ){
            msg.request.data = cmd_vel_z;
        }else if( cmd_vel_z < 0 && limit_low_state ){
            msg.request.data = cmd_vel_z;
        }else{
            msg.request.data = 0;
        }

        setAngleElev.call(msg);

        prev_cmd_z = cmd_vel_z;
    }

}