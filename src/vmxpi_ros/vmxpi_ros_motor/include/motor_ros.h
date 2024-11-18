#pragma once

#include <ros/ros.h>
#include "VMXPi.h"
#include "vmxpi_utils.h"
#include <std_msgs/Float32.h>
#include <thread>
#include <unistd.h>
#include <sys/syscall.h>

#include "vmxpi_ros_motor/pwm.h"

class MotorRos : public Utils{
	private:	 

		int motor;

		int INA;
		int INB;
		
		double duty_cycle = 0;

		VMXIO *io; // IO class of VMXPi
		VMXErrorCode vmxerr;
		VMXPi *vmx;

		VMXResourceHandle pwmgen_res_handle_a;
		VMXResourcePortIndex res_port_index_a;

		VMXResourceHandle pwmgen_res_handle_b;
		VMXResourcePortIndex res_port_index_b;

		ros::Subscriber motor_pwm;
		ros::ServiceServer motor_pwm_srv;
		ros::ServiceServer stop_motor;
       
	public:
		MotorRos( ros::NodeHandle *nh, VMXPi *vmx, int motor, int INA, int INB );
    	~MotorRos();
		
		// set a pwm from -1 to 1
		void SetMotorPWM( const std_msgs::Float32::ConstPtr& msg );
		bool StopMotor( vmxpi_ros_motor::pwm::Request &req, vmxpi_ros_motor::pwm::Response &res );
		bool SetMotorPWMSrv( vmxpi_ros_motor::pwm::Request &req, vmxpi_ros_motor::pwm::Response &res );
};