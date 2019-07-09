#ifndef CASTER_HARDWARE_SOCKETCAN_H_
#define CASTER_HARDWARE_SOCKETCAN_H_

#include <string>

#include <cstdlib>
#include <cstring>

#include <sys/types.h>

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <sensor_msgs/JointState.h>
#include <controller_manager/controller_manager.h>

#include <diagnostic_updater/diagnostic_updater.h>

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

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
    void Initialize(std::string node_name, ros::NodeHandle& nh, ros::NodeHandle& private_nh);

    void UpdateHardwareStatus();
    void WriteCommandsToHardware();

    void Clear();

    enum MotorIndex {
      kLeftMotor = 0x00,
      kRightMotor = 0x01, 
    };

    enum RoboteqClientCommandType {
      kCommand = 0x02,
      kQuery = 0x04,
      kResponseCommandSuccess = 0x06,
      kResponseMessageError = 0x08
    };

    enum RoboteqCanOpenObjectDictionary {
      /* runtime commands */
      kSetVelocity = 0x2002,
      kSetBLCounter = 0x2004,

      /* runtime queries */
      kReadAbsBLCounter = 0x2105,
      kReadBLMotorRPM = 0x210A,
      kReadStatusFlags = 0x2111,
      kReadFaultFlags = 0x2112,
      kReadMotorStatusFlags = 0x2122,
    };

    struct MotorStatus {
      int16_t rpm;
      int32_t counter;
      int32_t counter_offset;
      uint8_t status;
      bool counter_reset;

      MotorStatus() :
        rpm(0), counter(0), status(0), counter_reset(false)
      { }
    };

  private:
    void ResetTravelOffset();
    void RegisterControlInterfaces();

    std::string ToBinary(size_t data, uint8_t length);

    bool Command(RoboteqCanOpenObjectDictionary query, uint8_t sub_index, uint32_t data, uint8_t data_length);
    bool Query(RoboteqCanOpenObjectDictionary query, uint8_t sub_index, uint8_t data_length);
    void SendCanOpenData(uint32_t node_id, RoboteqClientCommandType type, RoboteqCanOpenObjectDictionary index, uint8_t sub_index, uint32_t data, uint8_t data_length);

    void CanReceiveCallback(const can_msgs::Frame::ConstPtr& msg);

    void ControllerTimerCallback(const ros::TimerEvent&);

    void MotorCheck(diagnostic_updater::DiagnosticStatusWrapper& status);
    void StatusCheck(diagnostic_updater::DiagnosticStatusWrapper& status);
    void ControllerCheck(diagnostic_updater::DiagnosticStatusWrapper& status);

    std::string node_name_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Timer timer_;
    controller_manager::ControllerManager *controller_manager_;

    std::string send_topic_, receive_topic_;
    
    ros::Publisher can_pub_;
    ros::Subscriber can_sub_;

    std::string left_wheel_joint_, right_wheel_joint_;

    // ROS Control interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;

    // diagnostic  update
    diagnostic_updater::Updater diagnostic_updater_;

    int can_id_;

    // ROS Parameters
    double wheel_diameter_, max_accel_, max_speed_;

    int8_t fault_flags_;
    int8_t status_flags_;
    MotorStatus motor_status_[2];

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
    } joints_[2];
};
}  // namespace iqr
#endif  // CASTER_HARDWARE_SOCKETCAN_H_
