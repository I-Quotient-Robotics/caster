#include "boller_base/boller_hardware.h"

#include <boost/assign/list_of.hpp>

namespace
{
  const uint8_t LEFT = 0, RIGHT = 1;
};

namespace boller_base
{

  /**
  * Initialize Boller hardware
  */
  BollerHardware::BollerHardware(ros::NodeHandle nh, ros::NodeHandle private_nh, double target_control_freq)
    :
    nh_(nh),
    private_nh_(private_nh)
  {
    private_nh_.param<double>("wheel_diameter", wheel_diameter_, 0.003302);
    private_nh_.param<double>("max_accel", max_accel_, 5.0);
    private_nh_.param<double>("max_speed", max_speed_, 1.0);
    private_nh_.param<double>("polling_timeout_", polling_timeout_, 10.0);

    std::string port;
    private_nh_.param<std::string>("port", port, "/dev/prolific");

    registerControlInterfaces();
  }

  /**
  * Get current encoder travel offsets from MCU and bias future encoder readings against them
  */
  void BollerHardware::resetTravelOffset() {

  }

  /**
  * Register interfaces with the RobotHW interface manager, allowing ros_control operation
  */
  void BollerHardware::registerControlInterfaces()
  {
    ros::V_string joint_names = boost::assign::list_of("front_left_wheel")
      ("front_right_wheel")("rear_left_wheel")("rear_right_wheel");
    for (unsigned int i = 0; i < joint_names.size(); i++)
    {
      hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
                                                              &joints_[i].position, &joints_[i].velocity,
                                                              &joints_[i].effort);
      joint_state_interface_.registerHandle(joint_state_handle);

      hardware_interface::JointHandle joint_handle(
        joint_state_handle, &joints_[i].velocity_command);
      velocity_joint_interface_.registerHandle(joint_handle);
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
  }


  /**
  * Pull latest speed and travel measurements from MCU, and store in joint structure for ros_control
  */
  void BollerHardware::updateJointsFromHardware() {

    ROS_DEBUG_STREAM("Received travel information (L:" << 0.003 << " R:" << 0.003 << ")");
    for (int i = 0; i < 4; i++)
    {
      double delta = 0.003;

      // detect suspiciously large readings, possibly from encoder rollover
      if (std::abs(delta) < 1)
      {
        joints_[i].position += delta;
      }
      else
      {
        // suspicious! drop this measurement and update the offset for subsequent readings
        joints_[i].position_offset += delta;
        ROS_DEBUG("Dropping overflow measurement from encoder");
      }
    }

    ROS_DEBUG_STREAM("Received linear speed information (L:" << 0.003 << " R:" << 0.003 << ")");
    for (int i = 0; i < 4; i++)
    {
      if (i % 2 == LEFT)
      {
        joints_[i].velocity = linearToAngular(0.003);
      }
      else
      { // assume RIGHT
        joints_[i].velocity = linearToAngular(0.003);
      }
    }
  }

  /**
  * Get latest velocity commands from ros_control via joint structure, and send to MCU
  */
  void BollerHardware::writeCommandsToHardware()
  {
    double diff_speed_left = angularToLinear(joints_[LEFT].velocity_command);
    double diff_speed_right = angularToLinear(joints_[RIGHT].velocity_command);

    limitDifferentialSpeed(diff_speed_left, diff_speed_right);

    // horizon_legacy::controlSpeed(diff_speed_left, diff_speed_right, max_accel_, max_accel_);
  }

  /**
  * Scale left and right speed outputs to maintain ros_control's desired trajectory without saturating the outputs
  */
  void BollerHardware::limitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right)
  {
    double large_speed = std::max(std::abs(diff_speed_left), std::abs(diff_speed_right));

    if (large_speed > max_speed_)
    {
      diff_speed_left *= max_speed_ / large_speed;
      diff_speed_right *= max_speed_ / large_speed;
    }
  }

  /**
  * Boller reports travel in metres, need radians for ros_control RobotHW
  */
  double BollerHardware::linearToAngular(const double &travel) const
  {
    return travel / wheel_diameter_ * 2;
  }

  /**
  * RobotHW provides velocity command in rad/s, Boller needs m/s,
  */
  double BollerHardware::angularToLinear(const double &angle) const
  {
    return angle * wheel_diameter_ / 2;
  }


}  // namespace boller_base
