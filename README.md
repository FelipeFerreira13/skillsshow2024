# skillsshow2024

This Mobile Robotics project aimed at providing straightforward tools for controlling mobile robots and interfacing with peripherals. It offers simple solutions for controlling robot movement and facilitates easy integration with various hardware components such as sensors and actuators. With clear documentation and a modular architecture, developers can quickly implement these tools into their projects.

## CORE
Implements the main logic of the project, defining the move goals, sensors referencing, logic flow, etc.

## CAMERA
Implements the image processing solutions with its functions and services.

## ROBOT
Set the robot definitions for the frames transform and the simulation parameters, such as, size, transforms, mechanical aspect, etc.

## BASE_CONTROLLER
Defines the controls parameters and outputs for reaching desired goals.

### msgs
| **Name**            | **Request**                      | **Response** |
|---------------------|----------------------------------|--------------|
| base_controller::move_goal | float64 x, float64 y, float64 th | bool status  |

### Service Server
| **Topic**               | **msg**            | **Description**                             |
|-------------------------|--------------------|---------------------------------------------|
| `base_controller/goal` | base_controller::move_goal | Define the desired goal |

## ODOMETRY
Defines the frame transform between base_footprint and odom. It can use either the encoder or the lidar as reference for odometry.

### msgs
| **Name**            | **Request**                      | **Response** |
|---------------------|----------------------------------|--------------|
| odometry::pose_odom | float64 x, float64 y, float64 th | bool status  |

### Publisher
| **Topic**               | **msg**            | **Description**                             |
|-------------------------|--------------------|---------------------------------------------|
| `odom`                  | nav_msgs::Odometry | Publish the to odom the new odometry pose   |
| `imu/data`              | sensor_msgs::Imu   | Publish the imu angle based on the robot Th |

### Service Server
| **Topic**               | **msg**            | **Description**                             |
|-------------------------|--------------------|---------------------------------------------|
| `odometry/set_position` | odometry::pose_odom | Redifine the robot position in X, Y and Th |

## OMS
Implements the basic controls of the robot's object management system.

### msgs
| **Name**            | **Request**                      | **Response** |
|---------------------|----------------------------------|--------------|
| oms::set_height     | float64 height                   | bool status  |
| oms::reset          | int32 direction                  | bool status  |
| oms::set_gripper    | float32 angle                    | bool status  |


### Publisher
| **Topic**               | **msg**           | **Description**                      |
|-------------------------|-------------------|--------------------------------------|
| `motor/3/set_motor_pwm` | std_msgs::Float32 | Publish the desired motor pwm value  |
| `oms/height`            | std_msgs::Float32 | Publish the current elevation height |

### Service Server
| **Topic**               | **msg**            | **Description**                             |
|-------------------------|--------------------|---------------------------------------------|
| `oms/set_height` | oms::set_height | Driver the robot to the desired height |
| `oms/reset` | oms::reset | Send the oms either up (direction = 1) or down (direction=-1) to reset the oms elevation |
| `oms/set_gripper` | oms::set_gripper | Define the desired gripper position |


## VMXPI_ROS
Implements the hardware interface. ( For this project it was used a VMX-pi )

### Pinout
| **Pin** | **Device** | **Class** |
|:-:|--------------------|------------------|
| `0` and `1` | Encoder Back | EncoderRos |
| `2` and `3` | Encoder Right | EncoderRos |
| `4` and `5` | Encoder Left | EncoderRos |
| `6` and `7` | Encoder Elevator | EncoderRos |
| `8` | Limit Switch High | DigitalInputROS |
| `9` | Limit Switch Low | DigitalInputROS |
| `10` | Stop Button  | DigitalInputROS |
| `11` | Start Button | DigitalInputROS |
| `16` | Gripper Servo | ServoROS |
| `17` | Elevator Servo | ServoROS |
| M0 | Motor Back | MotorRos |
| M1 | Motor Left | MotorRos |
| M2 | Motor Right | MotorRos |
| `22` | Sensor Left | SharpROS |
| `23` | Sensor Right | SharpROS |
| `24` | Sensor Front Left | SharpROS |
| `25` | Sensor Front Right | SharpROS |







## CONTACT
If you need support or have any suggestions or proposals please let me know:

- Send email to felipeferreira1310@hotmail.com with a clear description of your needs.
- Github Issues.
