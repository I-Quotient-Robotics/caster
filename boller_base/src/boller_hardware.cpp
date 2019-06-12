#include "boller_base/boller_hardware.h"

#include <boost/assign/list_of.hpp>

#include <math.h>

const uint8_t LEFT = 0, RIGHT = 1;
const uint16_t kCanOpenHeader = 0x600;

/**
* Initialize Boller hardware
*/
iqr::BollerHardware::BollerHardware(ros::NodeHandle nh) {
  nh_ = nh;
  // nh_.param<double>("max_accel", max_accel_, 5.0);
  // nh_.param<double>("max_speed", max_speed_, 1.0);
  // nh_.param<double>("wheel_diameter", wheel_diameter_, 0.003302);
  // nh_.param<double>("polling_timeout_", polling_timeout_, 10.0);
  nh_.param<int>("port", port_, 20001);
  nh_.param<std::string>("address", address_, "192.168.0.7");

  RegisterControlInterfaces();
}

bool iqr::BollerHardware::Connect() {
  socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if(socket_fd_ == -1) {
    ROS_INFO("%s\n", strerror(errno));
    return false;
  }

  bzero(&device_addr_, sizeof(device_addr_));
  device_addr_.sin_family = AF_INET;
  device_addr_.sin_port = htons(port_);
  device_addr_.sin_addr.s_addr = inet_addr(address_.c_str());
  bzero(&(device_addr_.sin_zero),sizeof(device_addr_.sin_zero));

  struct timeval timeout = {3,0}; 
  setsockopt(socket_fd_, SOL_SOCKET,SO_SNDTIMEO, (char *)&timeout, sizeof(struct timeval));
  setsockopt(socket_fd_, SOL_SOCKET,SO_RCVTIMEO, (char *)&timeout, sizeof(struct timeval));

  if (connect(socket_fd_, (struct sockaddr *)&device_addr_, sizeof(struct sockaddr_in)) == -1) {
    ROS_INFO("%s\n", strerror(errno));
    return false;
  } else {
    ROS_INFO("Boller base connected");
  }

  return true;
}

bool iqr::BollerHardware::RequestUpdate(uint8_t can_id) {
  uint8_t buf[13];
  bzero(buf, 13);
  buf[0] = 0x01;
  buf[1] = 0x00;
  buf[2] = 0x00;
  buf[3] = 0x00;
  buf[4] = can_id;
  buf[5] = 0x01;
  buf[6] = 0x02;
  buf[7] = 0x03;
  buf[8] = 0x04;
  buf[9] = 0x05;
  buf[10] = 0x06;
  buf[11] = 0x07;
  buf[12] = 0x08;

  // ROS_INFO("Sending request");
  if(send(socket_fd_, buf, 13, 0) == -1) {
    // ROS_INFO("sending error1");
    return false;
  }

  // ROS_INFO("Sending request done.");

  return true;
}


/**
* Pull latest speed and travel measurements from driver, and store in joint structure for ros_control
*/
void iqr::BollerHardware::UpdateJointsFromHardware() {
  uint8_t buf[100];
  int received_bytes = 0, can_id;
  bzero(buf, 100);

  int count;
  ioctl(socket_fd_, FIONREAD, &count);
  // ROS_INFO("data rest: %d", count);

  while(true) {
    if(RequestUpdate(0x06) == false) {
      // ROS_INFO("sending error");
    } else {
      bzero(buf, 100);

      // ROS_INFO("start recv");
      if((received_bytes=recv(socket_fd_, buf, 13, 0)) != -1) {
        // ROS_INFO("recv complete");
        // ROS_INFO("get data:%d", received_bytes);
        if(received_bytes >= 13) {
          if(buf[0] | 0b10000000 == CANBUS_BASE_FRAME_FORMAT &&
            buf[0] | 0b01000000 == CANBUS_DATA_FRAME) {
            uint32_t data_length = buf[0] | 0b00001111;

            int32_t temp;
            uint32_t can_id = 0;
            memcpy(&temp, buf+1, 4);
            can_id = ntohl(temp);

            // ROS_INFO("CANID6 %d", can_id);
            // ROS_INFO("CAN ID: %02x, DATA %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X ", can_id, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11], buf[12]);

            if(can_id == 0x06) {
              // ROS_INFO("Get 06 data");
              memcpy(&temp, buf+5, 4);
              int32_t speed = htonl(temp);
              joints_[0].velocity = speed / 60.0 / REDUCTION_RATIO * M_PI * 2.0;
              joints_[2].velocity = joints_[0].velocity;

              memcpy(&temp, buf+9, 4);
              speed = htonl(temp);
              joints_[1].velocity = speed / 60.0 / REDUCTION_RATIO * M_PI * 2.0;
              joints_[3].velocity = joints_[1].velocity;
              break;
            }
          }
        }
      }
    }
  }

  while(true) {
    if(RequestUpdate(0x05) == false) {
      // ROS_INFO("sending error");
    } else {
      bzero(buf, 100);

      if((received_bytes=recv(socket_fd_, buf, 13, 0)) != -1) {
        // ROS_INFO("get data:%d", received_bytes);
        if(received_bytes >= 13) {
          if(buf[0] | 0b10000000 == CANBUS_BASE_FRAME_FORMAT &&
            buf[0] | 0b01000000 == CANBUS_DATA_FRAME) {
            uint32_t data_length = buf[0] | 0b00001111;

            int32_t temp;
            uint32_t can_id = 0;
            memcpy(&temp, buf+1, 4);
            can_id = ntohl(temp);

            // ROS_INFO("CANID5 %d", can_id);
            // ROS_INFO("CAN ID: %02x, DATA %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X ", can_id, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11], buf[12]);

            if(can_id == 0x05) {
              // ROS_INFO("Get 05 data");
              memcpy(&temp, buf+5, 4);
              int32_t position = htonl(temp);
              joints_[0].position = position / 30.0 / REDUCTION_RATIO * M_PI * 2.0;
              joints_[2].position = joints_[0].position;

              memcpy(&temp, buf+9, 4);
              position = htonl(temp);
              joints_[1].position = position / 30.0 / REDUCTION_RATIO * M_PI * 2.0;
              joints_[3].position = joints_[1].position;
              break;
            }
          }
        }
      }
    }
    // ROS_INFO("05 loop");
  }

  // ROS_INFO("position: %f, %f; velocity: %f, %f" , joints_[0].position, joints_[1].position, joints_[0].velocity, joints_[1].velocity);
}

/**
* Get current encoder travel offsets from MCU and bias future encoder readings against them
*/
void iqr::BollerHardware::ResetTravelOffset() {

}

/**
* Register interfaces with the RobotHW interface manager, allowing ros_control operation
*/
void iqr::BollerHardware::RegisterControlInterfaces() {
  ros::V_string joint_names = boost::assign::list_of("front_left_wheel")
    ("front_right_wheel")("rear_left_wheel")("rear_right_wheel");

  for (unsigned int i = 0; i < joint_names.size(); i++) {
    hardware_interface::JointStateHandle joint_state_handle(joint_names[i], &joints_[i].position, &joints_[i].velocity, &joints_[i].effort);
    joint_state_interface_.registerHandle(joint_state_handle);

    hardware_interface::JointHandle joint_handle(joint_state_handle, &joints_[i].velocity_command);
    velocity_joint_interface_.registerHandle(joint_handle);
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);
}

/**
* Get latest velocity commands from ros_control via joint structure, and send to MCU
*/
// void iqr::BollerHardware::WriteCommandsToHardware() {
//   // ROS_INFO("command: %f, %f", joints_[0].velocity_command, joints_[1].velocity_command);

//   uint8_t buf[13];
//   bzero(buf, 13);
//   buf[0] = 0x08;
//   buf[1] = 0x00;
//   buf[2] = 0x00;
//   buf[3] = 0x00;
//   buf[4] = 0x09;

//   int16_t speed = static_cast<int16_t>(joints_[0].velocity_command / M_PI / 2.0 * REDUCTION_RATIO * 60);
//   int16_t s_v = ntohl(speed);
//   memcpy(buf+5, &s_v, 4);

//   speed = static_cast<int>(joints_[1].velocity_command / M_PI / 2.0 * REDUCTION_RATIO * 60);
//   s_v = ntohl(speed);
//   memcpy(buf+9, &s_v, 4);

//   if(send(socket_fd_, buf, 13, 0) == -1) {
//     // ROS_INFO("sending error1");
//   }
// }

void iqr::BollerHardware::WriteCommandsToHardware() {
  int32_t speed[2];

  speed[0] = static_cast<int32_t>(joints_[0].velocity_command / M_PI / 2.0 * REDUCTION_RATIO * 60);
  // int16_t s_v = ntohl(speed);
  // memcpy(buf+5, &s_v, 4);
  SendCanOpenData(1, kCommand, kSetVelocity, 1, static_cast<uint32_t>(speed[0]), 4);

  speed[1] = static_cast<int32_t>(joints_[1].velocity_command / M_PI / 2.0 * REDUCTION_RATIO * 60);
  // s_v = ntohl(speed);
  // memcpy(buf+9, &s_v, 4);
  SendCanOpenData(1, kCommand, kSetVelocity, 2, static_cast<uint32_t>(speed[1]), 4);

  ROS_INFO("command: %f, %f; rad: %d, %d", joints_[0].velocity_command, joints_[1].velocity_command, speed[0], speed[1]);
}


void iqr::BollerHardware::SendCanOpenData(uint32_t node_id, RoboteqClientCommandType type, RoboteqCanOpenObjectDictionary index, uint8_t sub_index, uint32_t data, uint8_t data_length) {
  uint8_t buf[13];
  bzero(buf, 13);

  uint32_t canopen_id = kCanOpenHeader + node_id;

  buf[0] = 0x08;
  uint32_t d = ntohl(canopen_id);
  memcpy(buf+1, &d, 4);
  buf[5] = (type<<4) + ((4-data_length) << 2);
  memcpy(buf+6, &index, 2);
  memcpy(buf+8, &sub_index, 1);
  memcpy(buf+9, &data, data_length);

  if(send(socket_fd_, buf, 13, 0) == -1) {
    ROS_INFO("canopen send error");
  }
}