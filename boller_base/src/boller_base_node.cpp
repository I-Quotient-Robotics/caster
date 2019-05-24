// #include <boost/chrono.hpp>

#include "boller_base/boller_hardware.h"
#include "controller_manager/controller_manager.h"
#include "ros/callback_queue.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "boller_base_node");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh, private_nh("~");

  // ROS_INFO("first");

  double control_frequency, diagnostic_frequency;
  private_nh.param<double>("control_frequency", control_frequency, 10.0);
  // private_nh.param<double>("diagnostic_frequency", diagnostic_frequency, 1.0);

  boller_base::BollerHardware boller(nh, private_nh, control_frequency);
  controller_manager::ControllerManager boller_controller_manager(&boller, nh);

  // ROS_INFO("second");

  // ros::Rate loop_rate(10);
  ros::Duration period(0.1);

  while(ros::ok()) {
    // ROS_INFO("enter");
    boller.updateJointsFromHardware();
    boller_controller_manager.update(ros::Time::now(), period);
    boller.writeCommandsToHardware();
    
    // ros::spinOnce();
    period.sleep();
    // ROS_INFO("done");
  }

  return 0;
}
