#ifndef CASTER_HARDWARE_H_
#define CASTER_HARDWARE_H_

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

namespace iqr {
/**
* Class representing Caster hardware, allows for ros_control to modify internal state via joint interfaces
*/
class CasterHardware : public hardware_interface::RobotHW {
  public:
    CasterHardware();

    bool Connect();
    void Initialize(std::string node_name, std::string address, int port, uint32_t can_id);

    void UpdateHardwareStatus();
    void WriteCommandsToHardware();

    enum MotorIndex {
      kLeftMotor = 0x01,
      kRightMotor = 0x02, 
    };

    enum RoboteqClientCommandType {
      kCommand = 0x02,
      kQuery = 0x04
    };

    enum RoboteqCanOpenObjectDictionary {
      /* runtime commands */
      kSetVelocity = 0x2002,

      /* runtime queries */
      kReadAbsBLCounter = 0x2105,
      kReadBLMotorRPM = 0x210A,
      kReadStatusFlags = 0x2111,
      kReadFaultFlags = 0x2112,
      kReadMotorStatusFlags = 0x2122,
    };

  private:
    void ResetTravelOffset();
    void RegisterControlInterfaces();

    std::string ToBinary(size_t data, uint8_t length);

    bool Command(RoboteqCanOpenObjectDictionary query, uint8_t sub_index, uint32_t data, uint8_t data_length);
    bool Query(RoboteqCanOpenObjectDictionary query, uint8_t sub_index, uint8_t data_length, uint32_t *data_received);
    void SendCanOpenData(uint32_t node_id, RoboteqClientCommandType type, RoboteqCanOpenObjectDictionary index, uint8_t sub_index, uint32_t data, uint8_t data_length);

    std::string node_name_;

    // ROS Control interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;

    int port_;
    std::string address_;

    int can_id_;

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
#endif  // CASTER_HARDWARE_H_
