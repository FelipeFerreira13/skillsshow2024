#pragma once

#include <ros/ros.h>
#include "VMXPi.h"
#include "vmxpi_utils.h"
#include <std_msgs/Int32.h>
#include <thread>
#include <unistd.h>
#include <sys/syscall.h>

class EncoderRos : public Utils {
	private:
		int encoder_channel_index;

        VMXIO *io;
        VMXErrorCode vmxerr;
        
		VMXResourceIndex encoder_index;
		VMXResourceHandle encoder_res_handle;

        ros::Publisher encoder_count_pub;

    	std::thread runth;


	public:
		EncoderRos( ros::NodeHandle *nh, VMXPi *vmx, uint8_t channel );
		~EncoderRos();
		
		int GetEncoderCount( );	

        void Run_t();
};