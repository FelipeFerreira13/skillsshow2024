#include "encoder_ros.h"

EncoderRos::EncoderRos( ros::NodeHandle *nh, VMXPi *vmx, uint8_t channel ) :
    io{&vmx->getIO()}, encoder_index(channel) {


    if( channel < 4 && channel > -1 ){

        VMXChannelInfo enc_channels[2] = { 
            {VMXChannelInfo( (encoder_index * 2) + 0, VMXChannelCapability::EncoderAInput)},
            {VMXChannelInfo( (encoder_index * 2) + 1, VMXChannelCapability::EncoderBInput)}};
        EncoderConfig encoder_cfg(EncoderConfig::EncoderEdge::x4);

        if (!io->ActivateDualchannelResource(enc_channels[0], enc_channels[1], &encoder_cfg, encoder_res_handle, &vmxerr)) {
            ROS_WARN("Failed to Activate Encoder Resource %d.\n", encoder_index);
            DisplayVMXError(vmxerr);
            io->DeallocateResource(encoder_res_handle, &vmxerr);
        } else {
            ROS_INFO("Successfully Activated Encoder Resource %d with VMXChannels %d and %d\n", encoder_index, enc_channels[0].index, enc_channels[1].index);
        }
    
        VMXResourceHandle encoder_res_handle;
        if (!io->GetResourceHandle(VMXResourceType::Encoder, encoder_index, encoder_res_handle, &vmxerr)) {
            DisplayVMXError(vmxerr);
        }

        std::string topic_name = "channel/" + std::to_string(encoder_index) + "/encoder/";
	    encoder_count_pub = nh->advertise<std_msgs::Int32>(topic_name + "count", 1);

	    runth = std::thread(&EncoderRos::Run_t, this);

    }else{
        ROS_WARN("Invalid Encoder input channel (0-3)\n");
    }
}

EncoderRos::~EncoderRos(){
    runth.join();
}

int EncoderRos::GetEncoderCount( ){
	int counter;
	if (io->Encoder_GetCount(encoder_res_handle, counter, &vmxerr)) {
		VMXIO::EncoderDirection encoder_direction;
		if(io->Encoder_GetDirection(encoder_res_handle, encoder_direction, &vmxerr)) {
			// printf("%s\t", (encoder_direction == VMXIO::EncoderForward) ? "F" : "R");
		} else {
			ROS_WARN("Error retrieving Encoder %d direction.\n", encoder_index);
			DisplayVMXError(vmxerr);
		}
	} else {
		ROS_WARN("Error retrieving Encoder %d count.\n", encoder_index);
		DisplayVMXError(vmxerr);
	}
	return counter;
}


void EncoderRos::Run_t() {
    ros::Rate r(10);
    ROS_INFO_STREAM("Encoder pub thread: " << syscall(SYS_gettid));
    while (ros::ok()) {
        std_msgs::Int32 msg;
        msg.data = GetEncoderCount();
        encoder_count_pub.publish(msg);
        r.sleep();
    }
}
