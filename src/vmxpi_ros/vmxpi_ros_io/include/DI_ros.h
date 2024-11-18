#pragma once

#include <ros/ros.h>
#include "VMXPi.h"
#include "vmxpi_utils.h"
#include <vmxpi_ros/FloatRes.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <sys/syscall.h>

class DigitalInputROS : public Utils {
	private:
		VMXIO *io;
		VMXTime *time;
		uint8_t channel_index;
		VMXResourceHandle digitalin_res_handle;


		ros::Publisher di_cm_pub, di_in_pub, di_pub;

		std::thread runth;

		bool GetRawValue();
	public:
		DigitalInputROS(ros::NodeHandle *nh, VMXPi *vmx, uint8_t channel, std::string name);
		
		~DigitalInputROS();
		
		bool Get();
		double GetCentimeters(uint32_t diff);
		double GetInches(uint32_t diff);
		
		void Run_t();
};
