#include "boller_base/boller_hardware.h"

#include <cstdlib>
#include <cstring>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "ros/callback_queue.h"
#include "controller_manager/controller_manager.h"

#define CANBUS_BASE_FRAME_FORMAT        0b00000000
#define CANBUS_EXTENDED_FRAME_FORMAT    0b10000000
#define CANBUS_REMOTE_FRAME             0b00000000
#define CANBUS_DATA_FRAME               0b01000000

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "boller_base_node");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;


  // int socket_fd;
  // struct sockaddr_in device_addr;
  // socket_fd = socket(AF_INET, SOCK_STREAM, 0);
  // if(socket_fd == -1) {
  //   ROS_INFO("%s\n", strerror(errno));
  //   return 0;
  // }

  // bzero(&device_addr, sizeof(device_addr));
  // device_addr.sin_family = AF_INET;
  // device_addr.sin_port = htons(20001);
  // device_addr.sin_addr.s_addr = inet_addr("192.168.0.7");
  // bzero(&(device_addr.sin_zero),sizeof(device_addr.sin_zero));

  // if (connect(socket_fd, (struct sockaddr *)&device_addr, sizeof(struct sockaddr_in)) == -1){
  //   ROS_INFO("%s\n", strerror(errno));
  //   return 0;
  // }

  // unsigned char buf[100];
  // int received_bytes;

  // double control_frequency, diagnostic_frequency;
  // private_nh.param<double>("control_frequency", control_frequency, 10.0);
  // private_nh.param<double>("diagnostic_frequency", diagnostic_frequency, 1.0);

  IQR::BollerHardware boller(nh);
  controller_manager::ControllerManager boller_controller_manager(&boller, nh);

  boller.Connect();

  ros::Duration period(0.1);

  while(ros::ok()) {
    boller.UpdateData();

    
    // boller.updateJointsFromHardware();
    boller_controller_manager.update(ros::Time::now(), period);
    boller.writeCommandsToHardware();
    period.sleep();
  }

  return 0;
}
