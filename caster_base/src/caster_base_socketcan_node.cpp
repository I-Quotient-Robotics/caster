#include "caster_base/caster_hardware_socketcan.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "caster_base_node");

  ros::AsyncSpinner spinner(3);

  ros::NodeHandle nh, private_nh("~");
  std::string node_name = ros::this_node::getName();

  iqr::CasterHardware caster;
  caster.Initialize(node_name, nh, private_nh);
  // controller_manager::ControllerManager caster_controller_manager(&caster, nh);

  // ros::Duration period(0.1);

  // while(ros::ok()) {

  //   // ros::spinOnce();
  //   period.sleep();
  // }


  spinner.start();
  ros::waitForShutdown();
  return 0;
}
