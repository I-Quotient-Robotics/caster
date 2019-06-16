#include "boller_base/boller_hardware.h"


#include <errno.h>
#include <boost/assign/list_of.hpp>

#include <math.h>

const uint8_t LEFT = 0, RIGHT = 1;
const uint16_t kCanOpenSendHeader = 0x600;
const uint16_t kCanOpenRecvHeader = 0x580;

/**
* Initialize Boller hardware
*/
iqr::BollerHardware::BollerHardware() {

}

void iqr::BollerHardware::Initialize(std::string node_name, std::string address, int port, uint32_t can_id) {
  node_name_ = node_name;
  address_ = address;
  port_ = port;
  can_id_ = can_id;

  RegisterControlInterfaces();

  ROS_INFO("address: %s, port: %d, can_id: %d", address_.c_str(), port_, can_id_);
  ROS_INFO("boller base initialized");
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

  /* set 30ms timeout */
  struct timeval timeout = {0, 30000}; 
  setsockopt(socket_fd_, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout, sizeof(struct timeval));
  setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(struct timeval));

  if (connect(socket_fd_, (struct sockaddr *)&device_addr_, sizeof(struct sockaddr_in)) == -1) {
    ROS_INFO("%s\n", strerror(errno));
    return false;
  } else {
    ROS_INFO("boller base connected");
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

void iqr::BollerHardware::UpdateHardwareStatus() {
  uint32_t left=-1, right=-1;
  uint32_t left_rpm=-1, right_rpm=-1;
  int16_t l_rpm=-1, r_rpm=-1;
  bool success = false;

    success = Query(kReadBLMotorRPM, static_cast<uint8_t>(kLeftMotor), 2, &left_rpm);


  // ros::Duration(1).sleep();

    success = Query(kReadBLMotorRPM, static_cast<uint8_t>(kRightMotor), 2, &right_rpm);
    // ROS_INFO("kkkk4");

  // ros::Duration(1).sleep();

    success = Query(kReadAbsBLCounter, static_cast<uint8_t>(kLeftMotor), 4, &left);
    // ROS_INFO("dddd1");

// ros::Duration(1).sleep();

    success = Query(kReadAbsBLCounter, static_cast<uint8_t>(kRightMotor), 4, &right);
    // ROS_INFO("kkkk2");

  // ros::Duration(0.05).sleep();
  // ros::Duration(1).sleep();

  l_rpm = static_cast<int16_t>(left_rpm);
  r_rpm = static_cast<int16_t>(right_rpm);

  ROS_INFO("motor counter: %d, %d, %d, %d", left, right, l_rpm, r_rpm);
  // SendCanOpenData(1, kQuery, kReadAbsBLCounter, static_cast<uint8_t>(kLeftMotor), 0xffff, 4);
  // SendCanOpenData(1, kQuery, kReadAbsBLCounter, static_cast<uint8_t>(kRightMotor), 0xffff, 4);

  // Query(kReadBLMotorRPM, static_cast<uint8_t>(kLeftMotor), 2, &received_data);
  // Query(kReadBLMotorRPM, static_cast<uint8_t>(kRightMotor), 2, &received_data);
  // SendCanOpenData(1, kQuery, kReadBLMotorRPM, static_cast<uint8_t>(kLeftMotor), 0xff, 2);
  // SendCanOpenData(1, kQuery, kReadBLMotorRPM, static_cast<uint8_t>(kRightMotor), 0xff, 2);
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
  Command(kSetVelocity, static_cast<uint8_t>(kLeftMotor), static_cast<uint32_t>(speed[0]), 4);
  // SendCanOpenData(1, kCommand, kSetVelocity, static_cast<uint8_t>(kLeftMotor), static_cast<uint32_t>(speed[0]), 4);

  speed[1] = static_cast<int32_t>(joints_[1].velocity_command / M_PI / 2.0 * REDUCTION_RATIO * 60);
  // s_v = ntohl(speed);
  // memcpy(buf+9, &s_v, 4);
  Command(kSetVelocity, static_cast<uint8_t>(kRightMotor), static_cast<uint32_t>(speed[1]), 4);
  // SendCanOpenData(1, kCommand, kSetVelocity, static_cast<uint8_t>(kRightMotor), static_cast<uint32_t>(speed[1]), 4);

  ROS_INFO("command: %f, %f; rad: %d, %d", joints_[0].velocity_command, joints_[1].velocity_command, speed[0], speed[1]);
}


void iqr::BollerHardware::SendCanOpenData(uint32_t node_id, RoboteqClientCommandType type, RoboteqCanOpenObjectDictionary index, uint8_t sub_index, uint32_t data, uint8_t data_length) {
  uint8_t buf[13];
  bzero(buf, 13);

  /* set type as standard frame, 8 bytes data */
  buf[0] = 0x08;

  /* set node id */
  uint32_t canopen_id = kCanOpenSendHeader + node_id;
  uint32_t d = ntohl(canopen_id);
  memcpy(buf+1, &d, 4);

  /* set command type and data length */
  buf[5] = (type<<4) + ((4-data_length) << 2);

  /* set index and sub index*/
  memcpy(buf+6, &index, 2);
  memcpy(buf+8, &sub_index, 1);

  /* set data */
  memcpy(buf+9, &data, data_length);

  /* send */
  // ROS_INFO("sending");
  int data_sent = -1;
  data_sent = send(socket_fd_, buf, 13, 0);
  // ROS_INFO("data_sent: %d", data_sent);
  if(data_sent != 13) {
    ROS_INFO("canopen send error");
  }
  // ROS_INFO("Sent");
}

bool iqr::BollerHardware::Query(RoboteqCanOpenObjectDictionary query, uint8_t sub_index, uint8_t data_length, uint32_t *data_received) {
  uint8_t buf[100];
  bzero(buf, 100);
  int received_bytes = 0;


  // ROS_INFO("------------------------------------");

    static int count = 0;
  // ROS_INFO("query count %d", count++);

  bool get_data = false;
  while(!get_data) {
    SendCanOpenData(can_id_, kQuery, query, sub_index, 0x0000, data_length);
    // SendCanOpenData(1, kQuery, kReadAbsBLCounter, static_cast<uint8_t>(kLeftMotor), 0xffff, 4);

    while(recv(socket_fd_, buf, 13, MSG_PEEK)>0) {
      received_bytes=recv(socket_fd_, buf, 13, 0);

      if(received_bytes >= 13) {
        if(buf[0] | 0b10000000 == CANBUS_BASE_FRAME_FORMAT &&
          buf[0] | 0b01000000 == CANBUS_DATA_FRAME) {
          // uint32_t data_length = buf[0] | 0b00001111;

          // ROS_INFO("CANID %d", can_id);
          // ROS_INFO("data: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X ", \
                    buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11], buf[12]);

          int32_t temp;
          uint32_t can_id = 0;
          memcpy(&temp, buf+1, 4);
          can_id = ntohl(temp);
          if(can_id != kCanOpenRecvHeader+can_id_) {
            ROS_WARN("incorrect can_id, expect %d, but %d received", kCanOpenRecvHeader+can_id_, can_id);
            // return false;
            continue;
          }

          /* buf[5] has bits0-1 is unknown, so set mask */ 
          if((buf[5]&0xFC) != (kQuery<<4)+((4-data_length) << 2)) {
            ROS_WARN("incorrect response status, expect 0x%X, but 0x%X received", (kQuery<<4)+((4-data_length) << 2), buf[5]&0xfc);
            // return false;
            continue;
          }

          uint32_t index = (buf[7] << 8) + buf[6];
          if(query != index) {
            ROS_WARN("incorrect index, expect 0x%X, but 0x%X received", query, index);
            // return false;
            continue;
          }
          
          if(sub_index != buf[8]) {
            ROS_WARN("incorrect sub_index, expect 0x%02X, but 0x%02X received", sub_index, buf[8]);
            // return false;
            continue;
          }

          memcpy(data_received, buf+9, 4);
          get_data = true;
          break;
        }
      }
    }
  }
  
  return true;
}

bool iqr::BollerHardware::Command(RoboteqCanOpenObjectDictionary query, uint8_t sub_index, uint32_t data, uint8_t data_length) {
  uint8_t buf[100];
  bzero(buf, 100);
  int received_bytes = 0;

  static int count = 0;
  ROS_INFO("command count %d", count++);

  SendCanOpenData(can_id_, kCommand, query, sub_index, data, data_length);

  if((received_bytes=recv(socket_fd_, buf, 13, 0)) != -1) {
    // ROS_INFO("recv complete");
    // ROS_INFO("get data:%d", received_bytes);
    if(received_bytes >= 13) {
      if(buf[0] | 0b10000000 == CANBUS_BASE_FRAME_FORMAT &&
        buf[0] | 0b01000000 == CANBUS_DATA_FRAME) {
        // uint32_t data_length = buf[0] | 0b00001111;

        // ROS_INFO("CANID %d", can_id);
        ROS_INFO("data: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X ", \
                  buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11], buf[12]);

        int32_t temp;
        uint32_t can_id = 0;
        memcpy(&temp, buf+1, 4);
        can_id = ntohl(temp);
        if(can_id != kCanOpenRecvHeader+can_id_) {
          ROS_WARN("incorrect can_id, expect %d, but %d received", kCanOpenRecvHeader+can_id_, can_id);
          // retursn false;
        }

        if(buf[5] != 0x60) {
          ROS_WARN("incorrect response status, expect 0x60, but 0x%X received", buf[5]);
          // return false;
        }

        uint32_t index = (buf[7] << 8) + buf[6];
        if(query != index) {
          ROS_WARN("incorrect index, expect 0x%X, but 0x%X received", query, index);
          // return false;
        }
        
        if(sub_index != buf[8]) {
          ROS_WARN("incorrect sub_index, expect 0x%02X, but 0x%02X received", sub_index, buf[8]);
          // return false;
        }
      }
    }
  }

  return true;
}