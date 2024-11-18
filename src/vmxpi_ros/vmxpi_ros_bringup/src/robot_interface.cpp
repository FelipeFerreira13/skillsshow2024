/************************************
 * Author: Felipe Ferreira
 * Release version: 1.0.0.0
 * Release date: 27/06/2024
 * 
 * Modified by: Felipe Ferreira
 * Last modification date: 08/07/2024
 * New version: 1.0.0.1

*************************************/

#include "TitanDriver_ros_wrapper.h"
#include "navX_ros_wrapper.h"
#include "encoder_ros.h"
#include "motor_ros.h"
#include "Sharp_ros.h"
#include "Servo_ros.h"
#include "IOwd_ros.h"
#include "DI_ros.h"
#include "DO_ros.h"
#include <unistd.h>


#include <dynamic_reconfigure/server.h>
#include <vmxpi_ros_bringup/MotorSpeedConfig.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <vmxpi_ros/Float.h>
#include <cmath>

#include <time.h>
#include <math.h>

#include <geometry_msgs/Twist.h>

#include "vmxpi_ros_motor/pwm.h"


static double leftVelocity, rightVelocity, backVelocity, elevatorVelocity; 
static double desired_back_speed, desired_right_speed, desired_left_speed;
static double left_enc, right_enc, back_enc, elevator_enc;
static double PI = 3.14159265;
static double wheelRadius = 0.051; //Wheel Radius
static double frameRadius = 0.15;  //Frame Radius
static double ticksPerRev = 1464;  //Encoder Ticks per Revolution

double cmd_vel_x, cmd_vel_y, cmd_vel_th;
double Rpm_conversion = ((2 * PI) / 60.0) * wheelRadius; //RPM to m/s

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& cmd_vel){
    // Convert the velocity command to individual wheel commands
    cmd_vel_x = cmd_vel->linear.x;
    cmd_vel_y = cmd_vel->linear.y;
    cmd_vel_th = cmd_vel->angular.z; }

// Callbacks for Encoder count values
void enc0Callback(const std_msgs::Int32::ConstPtr& msg){ back_enc     = msg->data; }
void enc1Callback(const std_msgs::Int32::ConstPtr& msg){ left_enc     = msg->data; }
void enc2Callback(const std_msgs::Int32::ConstPtr& msg){ right_enc    = msg->data; }
void enc3Callback(const std_msgs::Int32::ConstPtr& msg){ elevator_enc = msg->data; }

class PID{
    public:
        double tau = 0.02, T = 0.1;
        double kP, kI, kD, error;
        double limMin = -1.0, limMax = 1.0, limMinInt = -0.5, limMaxInt = 0.5;
        bool atSetpoint;
        double integrator, prevError, differentiator, prevMeasurement, output;
        double sumError;


        void PIDReset()
        {
            kP = 0.0;
            kI = 0.0;
            kD = 0.0;
            error = 0.0;
            integrator = 0.0;
            prevError = 0.0;
            differentiator = 0.0;
            prevMeasurement = 0.0;
            atSetpoint = false;
            output = 0.0;
            sumError = 0;
        }
        void setPID(double Kp, double Ki, double Kd)
        {
            kP = Kp;
            kI = Ki;
            kD = Kd;
        }
        void setPIDLimits(double LimMin, double LimMax)
        {
            limMin = LimMin; 
            limMax = LimMax; 
        }

        double calculate(double setPoint, double measurement)
        {
            if (setPoint > limMax)    { setPoint = limMax; }
            else if (setPoint < limMin)   { setPoint = limMin; }

            //Error
            error = setPoint - measurement;

            //Proportional
            double proportional = kP * error;

            if ((error == 0) && (prevError == 0) ){//|| error/abs(error) != prevError/abs(prevError)){
                sumError = 0;
            }
            else{
                sumError = sumError + error;

                double maxSum = 1.0;
                if( sumError > maxSum ){ sumError = maxSum; }
                else if( sumError < -maxSum ){ sumError = -maxSum; }
            }

            integrator = kI * sumError;
            //Integral
            //integrator = integrator + 0.5 * kI * T * (error + prevError);

            //Band limit derivative
            differentiator = -(2.0 * kD * (measurement - prevMeasurement) + (2.0 * tau - T) * differentiator) / (2.0 * tau + T);

            //Compute
            output = proportional + integrator;

            //Clamp
            if (output > limMax)    { output = limMax; }
            else if (output < limMin)   { output = limMin; }

            //Store variables
            prevError = error;
            prevMeasurement = measurement;
            
            //Return final value
            return output;
        }
};

PID PID_x;
PID PID_y;
PID PID_th;

class DynamicReconfig {
    bool flag = true;
    double error, vx, vy, vth;

    double previous_time;
    int previous_enc_l, previous_enc_r, previous_enc_b;

public:

    ros::ServiceClient set_m_speed;
    ros::ServiceClient enable_client, disable_client;
    ros::ServiceClient resetAngle, res_encoder_client;

    ros::ServiceClient set_m0_pwm, set_m1_pwm, set_m2_pwm, set_m3_pwm;
    ros::ServiceClient stop_m0_pwm, stop_m1_pwm, stop_m2_pwm, stop_m3_pwm;

    ros::Subscriber enc0_pub, enc1_pub, enc2_pub, enc3_pub;

    ros::Publisher vx_local_pub, vy_local_pub, vth_local_pub, error_pub;

    DynamicReconfig(ros::NodeHandle *nh) {

        set_m_speed = nh->serviceClient<vmxpi_ros::MotorSpeed>("titan/set_motor_speed");

        set_m0_pwm = nh->serviceClient<vmxpi_ros_motor::pwm>("motor/0/set_pwm");
        set_m1_pwm = nh->serviceClient<vmxpi_ros_motor::pwm>("motor/1/set_pwm");
        set_m2_pwm = nh->serviceClient<vmxpi_ros_motor::pwm>("motor/2/set_pwm");
        set_m3_pwm = nh->serviceClient<vmxpi_ros_motor::pwm>("motor/3/set_pwm");

        stop_m0_pwm = nh->serviceClient<vmxpi_ros_motor::pwm>("motor/0/stop_motor");
        stop_m1_pwm = nh->serviceClient<vmxpi_ros_motor::pwm>("motor/1/stop_motor");
        stop_m2_pwm = nh->serviceClient<vmxpi_ros_motor::pwm>("motor/2/stop_motor");
        stop_m3_pwm = nh->serviceClient<vmxpi_ros_motor::pwm>("motor/3/stop_motor");

        enc0_pub = nh->subscribe("titan/encoder0/count", 1, enc0Callback);
        enc1_pub = nh->subscribe("titan/encoder1/count", 1, enc1Callback);
        enc2_pub = nh->subscribe("titan/encoder2/count", 1, enc2Callback);
        enc3_pub = nh->subscribe("titan/encoder3/count", 1, enc3Callback);

        // enc0_pub = nh->subscribe("channel/0/encoder/count", 1, enc0Callback);
        // enc1_pub = nh->subscribe("channel/1/encoder/count", 1, enc1Callback);
        // enc2_pub = nh->subscribe("channel/2/encoder/count", 1, enc2Callback);
        // enc3_pub = nh->subscribe("channel/3/encoder/count", 1, enc3Callback);

        vx_local_pub  = nh->advertise<std_msgs::Float32>("vx_local",  10);
        vy_local_pub  = nh->advertise<std_msgs::Float32>("vy_local",  10);
        vth_local_pub = nh->advertise<std_msgs::Float32>("vth_local", 10);

        error_pub = nh->advertise<std_msgs::Float32>("error", 1);

        resetAngle = nh->serviceClient<std_srvs::Empty>("reset_navx");
    }

    void DirectKinematics()
    {
        //Forward Kinematics
        vx   = ( (      0     * backVelocity ) + ( (1.0/sqrt(3.0)) * rightVelocity) + ( (-1.0/sqrt(3.0)) * leftVelocity) );                // [m/s]
        vy   = ( ( (-2.0/3.0) * backVelocity ) + (    (1.0/3.0)    * rightVelocity) + (     (1.0/3.0)    * leftVelocity) );                // [m/s]
        vth  = ( ( ( 1.0/3.0) * backVelocity ) + (    (1.0/3.0)    * rightVelocity) + (     (1.0/3.0)    * leftVelocity) ) / frameRadius ; // [rad/s]

    }

    void InverseKinematis(double desired_vx, double desired_vy, double desired_vth)
    {
        //Inverse Kinematics
        desired_back_speed  = ( (-cos(PI/2)    * desired_vx) + (-sin(PI/2)    * desired_vy) + ( frameRadius * desired_vth) );  // [m/s]
        desired_right_speed = ( (-cos(7*PI/6)  * desired_vx) + (-sin(7*PI/6)  * desired_vy) + ( frameRadius * desired_vth) );  // [m/s]
        desired_left_speed  = ( (-cos(11*PI/6) * desired_vx) + (-sin(11*PI/6) * desired_vy) + ( frameRadius * desired_vth) );  // [m/s]

        double max = 0.700;   // m/s to PWM

        desired_back_speed  = desired_back_speed  / max;
        desired_right_speed = desired_right_speed / max;
        desired_left_speed  = desired_left_speed  / max;
        
    }

    void GetWheelsSpeed(){

        double current_time = ros::Time::now().toSec();
        double delta_time = double(current_time - previous_time);
        previous_time = current_time;

        double current_enc_l = left_enc;
        double delta_enc_l = current_enc_l - previous_enc_l;
        previous_enc_l = current_enc_l;

        double current_enc_r = right_enc;
        double delta_enc_r = current_enc_r - previous_enc_r;
        previous_enc_r = current_enc_r;

        double current_enc_b = back_enc;
        double delta_enc_b = current_enc_b - previous_enc_b;
        previous_enc_b = current_enc_b;

        //Wheels Velocity
        leftVelocity  = -1 * (((2 * PI * wheelRadius * delta_enc_l) / (ticksPerRev * delta_time)));   // [m/s]
        rightVelocity = -1 * (((2 * PI * wheelRadius * delta_enc_r) / (ticksPerRev * delta_time)));   // [m/s]
        backVelocity  = -1 * (((2 * PI * wheelRadius * delta_enc_b) / (ticksPerRev * delta_time)));   // [m/s]

        if ( isnan(leftVelocity)  || isinf(leftVelocity) ) { leftVelocity  = 0; }
        if ( isnan(rightVelocity) || isinf(rightVelocity) ){ rightVelocity = 0; }
        if ( isnan(backVelocity)  || isinf(backVelocity) ) { backVelocity  = 0; }

    }

    void PubVelocity()
    {
        std_msgs::Float32 msg;
        msg.data = vx;
        vx_local_pub.publish(msg);
        msg.data = vy;
        vy_local_pub.publish(msg);
        msg.data = vth;
        vth_local_pub.publish(msg);

        msg.data = error;
        error_pub.publish(msg);
    }

    void reset()
    {
        stop_motors();
        std_srvs::Empty msg2;
        resetAngle.call(msg2); // Resets yaw variable
        vx = 0;
        vy = 0;
        vth = 0;
        PubVelocity();
    }
    
    void stop_motors()
    {      
        // vmxpi_ros_motor::pwm msg1;
        // msg1.request.pwm = 0.0;

        // stop_m0_pwm.call(msg1);
        // stop_m1_pwm.call(msg1);
        // stop_m2_pwm.call(msg1);
        // stop_m3_pwm.call(msg1);

        // ros::Duration(1).sleep();

        // set_m0_pwm.call(msg1);
        // set_m1_pwm.call(msg1);
        // set_m2_pwm.call(msg1);
        // set_m3_pwm.call(msg1);

        vmxpi_ros::MotorSpeed msg1;

        msg1.request.speed = 0.0;
        msg1.request.motor = 0;
        set_m_speed.call(msg1);

        msg1.request.speed = 0.0;
        msg1.request.motor = 1;
        set_m_speed.call(msg1);

        msg1.request.speed = 0.0;
        msg1.request.motor = 2;
        set_m_speed.call(msg1);

    }

    //Set the wheels speed according to linear (m/s) and angular (RAD/s) velocities
    void setMovements(double desired_vx, double desired_vy, double desired_vth)
    {
        if (flag == true) {
            reset();
            PID_x.PIDReset();
            PID_y.PIDReset();
            PID_th.PIDReset();
            flag = false;
        }
        
        DirectKinematics();

        PID_x.setPID(0.85, 0.25, 0.0);
        PID_x.setPIDLimits(-0.300, 0.300);
        double x_drive = PID_x.calculate(desired_vx, vx);

        PID_y.setPID(0.85, 0.25, 0.0);
        PID_y.setPIDLimits(-0.300, 0.300);
        double y_drive = PID_y.calculate(desired_vy, vy);
                    
        PID_th.setPID(0.85, 0.25, 0.0);
        PID_th.setPIDLimits(-1.5, 1.5);
        double angle_drive = PID_th.calculate(desired_vth, vth);

        InverseKinematis(x_drive, y_drive, angle_drive);
                
    }

    void publish_motors()
    {
        // static double desired_left_speed_prev = 0, desired_back_speed_prev = 0, desired_right_speed_prev = 0;

        // if ( desired_left_speed == desired_left_speed_prev && 
        //            desired_back_speed == desired_back_speed_prev && desired_right_speed == desired_right_speed_prev ){}
        // else if ( desired_left_speed == 0 && desired_back_speed == 0 && desired_right_speed == 0 ){
        //     stop_motors();                    
        // }else{
        //     vmxpi_ros_motor::pwm msg;
            
        //     msg.request.pwm = desired_left_speed;
        //     set_m0_pwm.call(msg);

        //     msg.request.pwm = desired_back_speed;
        //     set_m1_pwm.call(msg);

        //     msg.request.pwm = desired_right_speed;
        //     set_m2_pwm.call(msg);
        // }

        // desired_left_speed_prev  = desired_left_speed;
        // desired_back_speed_prev  = desired_back_speed;
        // desired_right_speed_prev = desired_right_speed;

        vmxpi_ros::MotorSpeed msg1;

        msg1.request.speed = -desired_back_speed;
        msg1.request.motor = 0;
        set_m_speed.call(msg1);

        msg1.request.speed = -desired_left_speed;
        msg1.request.motor = 1;
        set_m_speed.call(msg1);

        msg1.request.speed = -desired_right_speed;
        msg1.request.motor = 2;
        set_m_speed.call(msg1);
    }

    void callback(vmxpi_ros_bringup::MotorSpeedConfig &config, uint32_t level) {

        reset();

        ros::Rate loop_rate(5);

        while (ros::ok()){

            GetWheelsSpeed();

            setMovements(cmd_vel_x, cmd_vel_y, cmd_vel_th); // (Linear velocity, angular velocity)

            publish_motors();
            PubVelocity();

            ros::spinOnce();
            loop_rate.sleep();

        }

        reset();
    }
};

int main(int argc, char **argv) {
    
    system("/usr/local/frc/bin/frcKillRobot.sh");
    
    ROS_INFO_STREAM("Main thread: " << syscall(SYS_gettid));

    ros::init(argc, argv, "robot_interface");

    ROS_INFO("robot_interface node is now started");

    ros::NodeHandle nh;
    VMXPi vmx(true, (uint8_t) 50);
    VMXErrorCode vmxerr;

    ROS_INFO("VMX is now started");

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 1, cmd_vel_callback);

    SharpROS sensor_left        ( &nh, &vmx, 22 );
    SharpROS sensor_right       ( &nh, &vmx, 23 );
    SharpROS sensor_front_left  ( &nh, &vmx, 24 );
    SharpROS sensor_front_right ( &nh, &vmx, 25 );

    DigitalInputROS limit_switch_high ( &nh, &vmx, 8 , "limit_switch_high" );
    DigitalInputROS limit_switch_low  ( &nh, &vmx, 9 , "limit_switch_low"  );
    DigitalInputROS stop_button       ( &nh, &vmx, 10, "stop_button"       );
    DigitalInputROS start_button      ( &nh, &vmx, 11, "start_button"      );

    EncoderRos encoder_0(&nh, &vmx, 0);     // Back
    EncoderRos encoder_1(&nh, &vmx, 1);     // Right
    EncoderRos encoder_2(&nh, &vmx, 2);     // Left
    EncoderRos encoder_3(&nh, &vmx, 3);     // Elevator

    // MotorRos motor_0(&nh, &vmx, 0, 21, 20);   // Left
    // MotorRos motor_1(&nh, &vmx, 1, 19, 18);   // Back
    // MotorRos motor_2(&nh, &vmx, 2, 17, 16);   // Right
    // MotorRos motor_3(&nh, &vmx, 3, 15, 14);   // Elevator

    ServoROS gripper  (&nh, &vmx, 16);  // Gripper 
    ServoROS elevation(&nh, &vmx, 17);  // Elevation


    navXROSWrapper navx_local(&nh, &vmx);
    ROS_INFO("navX_local driver is now started");

    TitanDriverROSWrapper titan(&nh, &vmx);
    ROS_INFO("Titan driver is now started");

    DynamicReconfig cfg(&nh);
    dynamic_reconfigure::Server<vmxpi_ros_bringup::MotorSpeedConfig> server;
    dynamic_reconfigure::Server<vmxpi_ros_bringup::MotorSpeedConfig>::CallbackType f;
    f = boost::bind(&DynamicReconfig::callback, &cfg, _1, _2);
    server.setCallback(f);

    ROS_INFO("ROS SHUTDOWN");
    ros::waitForShutdown();
    return 0;
}