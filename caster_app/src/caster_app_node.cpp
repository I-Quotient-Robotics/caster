#include <ros/ros.h>

#include <dock_server.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "caster_app_node");
  ros::NodeHandle n("~");
  iqr::DockServer dock_server(n, "dock");
  dock_server.Initialize();

  ros::Duration period(0.02);
  while(ros::ok()) {
    ros::spinOnce();
    period.sleep();
  }
  return 0;
}