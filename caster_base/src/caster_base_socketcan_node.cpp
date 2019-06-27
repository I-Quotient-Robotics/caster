#include "caster_base/caster_hardware_socketcan.h"

#include "controller_manager/controller_manager.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "caster_base_node");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh, private_nh("~");
  std::string node_name = ros::this_node::getName();

  iqr::CasterHardware caster;
  caster.Initialize(node_name, nh, private_nh);
  controller_manager::ControllerManager caster_controller_manager(&caster, nh);

  // caster.Connect();

  ros::Duration period(0.1);

  while(ros::ok()) {
    // caster.UpdateHardwareStatus();
    caster_controller_manager.update(ros::Time::now(), period);
    caster.WriteCommandsToHardware();
    ros::spinOnce();
    period.sleep();
  }

  return 0;
}
