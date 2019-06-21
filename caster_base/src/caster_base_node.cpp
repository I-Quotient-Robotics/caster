#include "caster_base/caster_hardware.h"

#include "controller_manager/controller_manager.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "caster_base_node");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh, private_nh("~");

  // double control_frequency, diagnostic_frequency;
  // private_nh.param<double>("control_frequency", control_frequency, 10.0);
  // private_nh.param<double>("diagnostic_frequency", diagnostic_frequency, 1.0);

    // nh_.param<double>("max_accel", max_accel_, 5.0);
  // nh_.param<double>("max_speed", max_speed_, 1.0);
  // nh_.param<double>("wheel_diameter", wheel_diameter_, 0.003302);
  // nh_.param<double>("polling_timeout_", polling_timeout_, 10.0);

  int port, can_id;
  std::string node_name, address, left_wheel_joint, right_wheel_joint;

  private_nh.param<int>("port", port, 20001);
  private_nh.param<std::string>("address", address, "192.168.0.7");
  private_nh.param<int>("can_id", can_id, 1);
  private_nh.param<std::string>("left_wheel_joint", left_wheel_joint, "drive_wheel_left_joint");
  private_nh.param<std::string>("right_wheel_joint", right_wheel_joint, "drive_wheel_right_joint");
  node_name = ros::this_node::getName();

  iqr::CasterHardware caster;
  caster.Initialize(node_name, address, port, can_id, left_wheel_joint, right_wheel_joint);
  // ros::Duration(1).sleep();
  controller_manager::ControllerManager caster_controller_manager(&caster, nh);

  caster.Connect();

  ros::Duration period(0.02);

  while(ros::ok()) {
    caster.UpdateHardwareStatus();
    caster_controller_manager.update(ros::Time::now(), period);
    caster.WriteCommandsToHardware();
    period.sleep();
    ros::spinOnce();
  }

  return 0;
}
