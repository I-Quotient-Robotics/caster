#ifndef BOLLER_HARDWARE_H_
#define BOLLER_HARDWARE_H_

#include <string>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "hardware_interface/robot_hw.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"

namespace boller_base
{

  /**
  * Class representing Boller hardware, allows for ros_control to modify internal state via joint interfaces
  */
  class BollerHardware :
    public hardware_interface::RobotHW
  {
  public:
    BollerHardware(ros::NodeHandle nh, ros::NodeHandle private_nh, double target_control_freq);

    void updateJointsFromHardware();

    void writeCommandsToHardware();

  private:

    void resetTravelOffset();
    void registerControlInterfaces();
    double linearToAngular(const double &travel) const;
    double angularToLinear(const double &angle) const;
    void limitDifferentialSpeed(double &travel_speed_left, double &travel_speed_right);

    ros::NodeHandle nh_, private_nh_;

    // ROS Control interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;

    // ROS Parameters
    double wheel_diameter_, max_accel_, max_speed_;

    double polling_timeout_;

    /**
    * Joint structure that is hooked to ros_control's InterfaceManager, to allow control via diff_drive_controller
    */
    struct Joint
    {
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

}  // namespace boller_base
#endif  // BOLLER_HARDWARE_H_
