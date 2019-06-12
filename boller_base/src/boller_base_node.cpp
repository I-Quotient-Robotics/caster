#include "boller_base/boller_hardware.h"

#include "controller_manager/controller_manager.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "boller_base_node");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;

  // double control_frequency, diagnostic_frequency;
  // private_nh.param<double>("control_frequency", control_frequency, 10.0);
  // private_nh.param<double>("diagnostic_frequency", diagnostic_frequency, 1.0);

  iqr::BollerHardware boller(nh);
  controller_manager::ControllerManager boller_controller_manager(&boller, nh);

  boller.Connect();

  ros::Duration period(0.1);

  while(ros::ok()) {
    // boller.UpdateJointsFromHardware();
    boller_controller_manager.update(ros::Time::now(), period);
    boller.WriteCommandsToHardware();
    period.sleep();
  }

  return 0;
}
