#ifndef BOLLER_HARDWARE_H_
#define BOLLER_HARDWARE_H_

#include <string>

#include <boost/asio.hpp>

#include <cstdlib>
#include <cstring>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "hardware_interface/robot_hw.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"

#define REDUCTION_RATIO                 15.0

#define CANBUS_BASE_FRAME_FORMAT        0b00000000
#define CANBUS_EXTENDED_FRAME_FORMAT    0b10000000
#define CANBUS_REMOTE_FRAME             0b00000000
#define CANBUS_DATA_FRAME               0b01000000

#define CAN_ID_GET_COUNT                0x05
#define CAN_ID_GET_SPEED                0x06
#define CAN_ID_GET_STATUS               0x07
#define CAN_ID_SET_SPEED                0x09


namespace iqr {
/**
* Class representing Boller hardware, allows for ros_control to modify internal state via joint interfaces
*/
class BollerHardware : public hardware_interface::RobotHW {
  public:
    BollerHardware(ros::NodeHandle nh);

    void UpdateJointsFromHardware();
    void WriteCommandsToHardware();

    bool RequestUpdate(uint8_t can_id);
    bool Connect();

    enum RoboteqClientCommandType {
      kCommand = 0x02,
      kQuery = 0x04
    };

    enum RoboteqCanOpenObjectDictionary {
      /* runtime commands */
      kSetVelocity = 0x2002,

      /* runtime queries */
      kReadAbsBLCounter = 0x2105,
      kReadBLMotorRPM = 0x210A
    };

  private:
    void ResetTravelOffset();
    void RegisterControlInterfaces();

    void SendCanOpenData(uint32_t node_id, RoboteqClientCommandType type, RoboteqCanOpenObjectDictionary index, uint8_t sub_index, uint32_t data, uint8_t data_length);

    ros::NodeHandle nh_;

    // ROS Control interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;

    int port_;
    std::string address_;

    // ROS Parameters
    double wheel_diameter_, max_accel_, max_speed_;

    double polling_timeout_;

    int socket_fd_;
    struct sockaddr_in device_addr_;

    /**
    * Joint structure that is hooked to ros_control's InterfaceManager, to allow control via diff_drive_controller
    */
    struct Joint {
      double position;
      double position_offset;
      double velocity;
      double effort;
      double velocity_command;

      Joint() :
        position(0), velocity(0), effort(0), velocity_command(0)
      { }
    } joints_[4];
};
}  // namespace iqr
#endif  // BOLLER_HARDWARE_H_
