#include "motor_ros.h"

MotorRos::MotorRos(ros::NodeHandle *nh, VMXPi *vmx, int motor_, int INA_, int INB_) : 
    io{&vmx->getIO()}, motor(motor_), INA(INA_), INB(INB_) {
    
    if( INA < 22 && INA > 11 && INB < 22 && INB > 11 ){

        PWMGeneratorConfig pwmgen_cfg(1000 /* Frequency in Hz */);

        int dio_channel_index = INA;

        if (!io->ActivateSinglechannelResource(VMXChannelInfo(dio_channel_index, VMXChannelCapability::PWMGeneratorOutput), &pwmgen_cfg, 
                pwmgen_res_handle_a, &vmxerr)) {
            ROS_WARN("Error Activating Singlechannel Resource PWM Generator for Channel index %d.\n", dio_channel_index);
            DisplayVMXError(vmxerr);

        } else {
            ROS_INFO("Successfully Activated PWMGenerator Resource %d with VMXChannel %d\n", EXTRACT_VMX_RESOURCE_INDEX(pwmgen_res_handle_a), dio_channel_index);
            if (!io->PWMGenerator_SetDutyCycle(pwmgen_res_handle_a, res_port_index_a, 50, &vmxerr)) {
                ROS_WARN("Failed to set DutyCycle for PWMGenerator Resource %d, Port %d.\n", EXTRACT_VMX_RESOURCE_INDEX(pwmgen_res_handle_a), res_port_index_a);
                DisplayVMXError(vmxerr);
            }
        }

        dio_channel_index = INB;
        
        if (!io->ActivateSinglechannelResource(VMXChannelInfo(dio_channel_index, VMXChannelCapability::PWMGeneratorOutput), &pwmgen_cfg, 
                pwmgen_res_handle_b, &vmxerr)) {
            ROS_WARN("Error Activating Singlechannel Resource PWM Generator for Channel index %d.\n", dio_channel_index);
            DisplayVMXError(vmxerr);

        } else {
            ROS_INFO("Successfully Activated PWMGenerator Resource %d with VMXChannel %d\n", EXTRACT_VMX_RESOURCE_INDEX(pwmgen_res_handle_b), dio_channel_index);
            if (!io->PWMGenerator_SetDutyCycle(pwmgen_res_handle_b, res_port_index_b, 50, &vmxerr)) {
                ROS_WARN("Failed to set DutyCycle for PWMGenerator Resource %d, Port %d.\n", EXTRACT_VMX_RESOURCE_INDEX(pwmgen_res_handle_b), res_port_index_b);
                DisplayVMXError(vmxerr);
            }
        }

        std::string topic_name = "motor/" + std::to_string(motor_);
        motor_pwm  = nh->subscribe(topic_name + "/set_motor_pwm", 1, &MotorRos::SetMotorPWM, this);
        motor_pwm_srv = nh->advertiseService(topic_name + "/set_pwm", &MotorRos::SetMotorPWMSrv, this);
        stop_motor = nh->advertiseService(topic_name + "/stop_motor", &MotorRos::StopMotor, this);

    }else{
        ROS_WARN("Invalid Analog input channel (12-21)\n");
    }
}

MotorRos::~MotorRos(){
    if (!INVALID_VMX_RESOURCE_HANDLE(pwmgen_res_handle_a)) {
        if (!io->DeallocateResource(pwmgen_res_handle_a, &vmxerr)) {
            ROS_WARN("Error Deallocating PWM Resource.\n");
        } else {
            ROS_INFO("Successfully Deallocated PWM Resource Index %d\n", EXTRACT_VMX_RESOURCE_INDEX(pwmgen_res_handle_a));
        }
    }
    if (!INVALID_VMX_RESOURCE_HANDLE(pwmgen_res_handle_b)) {
        if (!io->DeallocateResource(pwmgen_res_handle_b, &vmxerr)) {
            ROS_WARN("Error Deallocating PWM Resource.\n");
        } else {
            ROS_INFO("Successfully Deallocated PWM Resource Index %d\n", EXTRACT_VMX_RESOURCE_INDEX(pwmgen_res_handle_b));
        }
    }
}


bool MotorRos::SetMotorPWMSrv( vmxpi_ros_motor::pwm::Request &req, vmxpi_ros_motor::pwm::Response &res ){
    // printf("duty_cycle_ set to %f \n", pwm);

    float pwm = req.pwm;

    double duty_cycle_ = ( abs(pwm) * 255.0 );

    int ina = 0, inb = 0;

    if      ( pwm > 0 ) { ina = duty_cycle_; inb = 0;           }
    else if ( pwm < 0 ) { ina = 0;           inb = duty_cycle_; }
    else if ( pwm == 0 ){ ina = 255;         inb = 255;         }

    if (!io->PWMGenerator_SetDutyCycle(pwmgen_res_handle_a, res_port_index_a, ina, &vmxerr)) {
        ROS_WARN("Failed to set DutyCycle for PWMGenerator Resource %d, Port %d.\n", EXTRACT_VMX_RESOURCE_INDEX(pwmgen_res_handle_a), res_port_index_a);
        DisplayVMXError(vmxerr);
        return false;
    }
        if (!io->PWMGenerator_SetDutyCycle(pwmgen_res_handle_b, res_port_index_b, inb, &vmxerr)) {
        ROS_WARN("Failed to set DutyCycle for PWMGenerator Resource %d, Port %d.\n", EXTRACT_VMX_RESOURCE_INDEX(pwmgen_res_handle_b), res_port_index_b);
        DisplayVMXError(vmxerr);
        return false;
    }

    duty_cycle = duty_cycle_;
    return true;
}

void MotorRos::SetMotorPWM( const std_msgs::Float32::ConstPtr& msg ){
    // printf("duty_cycle_ set to %f \n", pwm);

    float pwm = msg->data;

    double duty_cycle_ = ( abs(pwm) * 255.0 );

    int ina = 0, inb = 0;

    if      ( pwm > 0 ) { ina = duty_cycle_; inb = 0;           }
    else if ( pwm < 0 ) { ina = 0;           inb = duty_cycle_; }
    else if ( pwm == 0 ){ ina = 255;         inb = 255;         }

    if (!io->PWMGenerator_SetDutyCycle(pwmgen_res_handle_a, res_port_index_a, ina, &vmxerr)) {
        ROS_WARN("Failed to set DutyCycle for PWMGenerator Resource %d, Port %d.\n", EXTRACT_VMX_RESOURCE_INDEX(pwmgen_res_handle_a), res_port_index_a);
        DisplayVMXError(vmxerr);
        // return false;
    }
        if (!io->PWMGenerator_SetDutyCycle(pwmgen_res_handle_b, res_port_index_b, inb, &vmxerr)) {
        ROS_WARN("Failed to set DutyCycle for PWMGenerator Resource %d, Port %d.\n", EXTRACT_VMX_RESOURCE_INDEX(pwmgen_res_handle_b), res_port_index_b);
        DisplayVMXError(vmxerr);
        // return false;
    }

    duty_cycle = duty_cycle_;
    // return true;
}

bool MotorRos::StopMotor( vmxpi_ros_motor::pwm::Request &req, vmxpi_ros_motor::pwm::Response &res ){

    int ina = 0, inb = 0;

    if (!io->PWMGenerator_SetDutyCycle(pwmgen_res_handle_a, res_port_index_a, ina, &vmxerr)) {
        ROS_WARN("Failed to set DutyCycle for PWMGenerator Resource %d, Port %d.\n", EXTRACT_VMX_RESOURCE_INDEX(pwmgen_res_handle_a), res_port_index_a);
        DisplayVMXError(vmxerr);
        return false;
    }
        if (!io->PWMGenerator_SetDutyCycle(pwmgen_res_handle_b, res_port_index_b, inb, &vmxerr)) {
        ROS_WARN("Failed to set DutyCycle for PWMGenerator Resource %d, Port %d.\n", EXTRACT_VMX_RESOURCE_INDEX(pwmgen_res_handle_b), res_port_index_b);
        DisplayVMXError(vmxerr);
        return false;
    }

    duty_cycle = 0;
    return true;
}



