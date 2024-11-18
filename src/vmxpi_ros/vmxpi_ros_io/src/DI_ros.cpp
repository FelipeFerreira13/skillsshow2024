#include "DI_ros.h"

DigitalInputROS::DigitalInputROS(ros::NodeHandle *nh, VMXPi *vmx, uint8_t channel, std::string name) :
	io{&vmx->getIO()}, time{&vmx->getTime()}, channel_index(channel) {
	VMXErrorCode vmxerr;
	DIOConfig dio_config;
	if (!io->ActivateSinglechannelResource(VMXChannelInfo(channel_index, VMXChannelCapability::DigitalInput), &dio_config, 
		digitalin_res_handle, &vmxerr)) {
		printf("Error Activating Singlechannel Resource DIO for Channel index %d.\n", channel_index);
    DisplayVMXError(vmxerr);
	} else {
		ROS_INFO("Digital Input Channel %d activated on Resource type %d, index %d\n", channel_index,
						EXTRACT_VMX_RESOURCE_TYPE(digitalin_res_handle),
						EXTRACT_VMX_RESOURCE_INDEX(digitalin_res_handle));
	}


	
	std::string topic_name = "robot/digital_in/" + name;
	// di_cm_pub = nh->advertise<std_msgs::Float32>(topic_name + "get_cm", 1);
	// di_in_pub = nh->advertise<std_msgs::Float32>(topic_name + "get_inch", 1);
	di_pub = nh->advertise<std_msgs::Bool>(topic_name + "/state", 1);

	
	runth = std::thread(&DigitalInputROS::Run_t, this);
}

DigitalInputROS::~DigitalInputROS() {
	runth.join();
}

bool DigitalInputROS::Get() {
    VMXErrorCode vmxerr;
    bool high;
    if(io->DIO_Get(digitalin_res_handle, high, &vmxerr)) {
		if(high) {
			ROS_INFO("High");
		} else {
			ROS_INFO("Low");
			DisplayVMXError(vmxerr);
		}
    }
    return high;
}

double DigitalInputROS::GetCentimeters(uint32_t diff) {
	return (double)diff / 58;
}

double DigitalInputROS::GetInches(uint32_t diff) {
	return (double)diff / 148;
}

bool DigitalInputROS::GetRawValue() {
	VMXErrorCode vmxerr;
	bool state;
	uint32_t diff, begin = 0, end = 0;

	// while (io->DIO_Get(digitalin_res_handle, state, &vmxerr)) {
	// 	if (state) {
	// 		begin = time->GetCurrentMicroseconds();
	// 		//ROS_INFO("begin: %" PRIu64"\n", begin);
	// 		break;
	// 	}
	// 	//ROS_INFO("begin state: %s \n", state ? "HIGH" : "LOW");
	// }
		
	// while (io->DIO_Get(digitalin_res_handle, state, &vmxerr)) { 
	// 	if (!state) {
	// 		end = time->GetCurrentMicroseconds();
	// 		//ROS_INFO("end: %" PRIu64"\n", end);
	// 		break;
	// 	}
	// 	//ROS_INFO("end state: %s \n", state ? "HIGH" : "LOW");
	// }
	
	diff = end - begin;

	io->DIO_Get(digitalin_res_handle, state, &vmxerr);

	return state;
}

void DigitalInputROS::Run_t() {

	ros::Rate r(5);
  	ROS_INFO_STREAM("Digital Input pub thread: " << syscall(SYS_gettid));

  	while (ros::ok()) {
		bool diff = this->GetRawValue();
		std_msgs::Bool msg;

		msg.data = diff;
		di_pub.publish(msg);
	}
}
