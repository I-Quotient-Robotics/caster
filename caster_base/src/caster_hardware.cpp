#include "caster_base/caster_hardware.h"

#include <math.h>
#include <errno.h>

const uint16_t kCanOpenSendHeader = 0x600;
const uint16_t kCanOpenRecvHeader = 0x580;

/**
* Initialize Caster hardware
*/
iqr::CasterHardware::CasterHardware() {

}

std::string iqr::CasterHardware::ToBinary(size_t data, uint8_t length) {
  std::string data_binary = "";
  for(int i=0; i<length*8; i++) {
    data_binary.append(std::to_string((data>>i)&0x01));
  }

  return data_binary;
}

void iqr::CasterHardware::Initialize(std::string node_name, std::string address, int port, \
                                     uint32_t can_id, std::string left_wheel_joint, std::string right_wheel_joint) {
  node_name_ = node_name;
  address_ = address;
  port_ = port;
  can_id_ = can_id;

  left_wheel_joint_ = left_wheel_joint;
  right_wheel_joint_ = right_wheel_joint;

  RegisterControlInterfaces();

  ROS_INFO("address: %s, port: %d, can_id: %d", address_.c_str(), port_, can_id_);
  ROS_INFO("caster base initialized");
}

bool iqr::CasterHardware::Connect() {
  socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if(socket_fd_ == -1) {
    ROS_WARN("%s\n", strerror(errno));
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
    ROS_WARN("%s\n", strerror(errno));
    return false;
  } else {
    ROS_INFO("caster base connected");

    // set motor counter to 0
    ClearBLCounter();
  }

  return true;
}

void iqr::CasterHardware::ClearBLCounter() {
  Command(kSetBLCounter, static_cast<uint8_t>(kLeftMotor), 0, 4);
  Command(kSetBLCounter, static_cast<uint8_t>(kRightMotor), 0, 4);
}

void iqr::CasterHardware::UpdateHardwareStatus() {
  bool success = false;
  uint32_t data;

  /* request motor speed */
  int16_t l_rpm=-1, r_rpm=-1;
  uint32_t left_rpm=-1, right_rpm=-1;
  success = Query(kReadBLMotorRPM, static_cast<uint8_t>(kLeftMotor), 2, &left_rpm);
  success = Query(kReadBLMotorRPM, static_cast<uint8_t>(kRightMotor), 2, &right_rpm);
  l_rpm = static_cast<int16_t>(left_rpm);
  r_rpm = static_cast<int16_t>(right_rpm);

  /* request motor counter */
  int32_t l_count=-1, r_count=-1;
  success = Query(kReadAbsBLCounter, static_cast<uint8_t>(kLeftMotor), 4, reinterpret_cast<uint32_t*>(&l_count));
  success = Query(kReadAbsBLCounter, static_cast<uint8_t>(kRightMotor), 4, reinterpret_cast<uint32_t*>(&r_count));

  /* request status flags 
   * f1 = Serial mode
   * f2 = Pulse mode
   * f3 = Analog mode
   * f4 = Power stage off
   * f5 = Stall detected
   * f6 = At limit
   * f7 = Unused
   * f8 = MicroBasic script running
  */
  uint8_t status_flag;
  success = Query(kReadStatusFlags, 0x00, 1, &data);
  status_flag = static_cast<uint8_t>(data);

  /* request fault flags
   * f1 = Overheat
   * f2 = Overvoltage
   * f3 = Undervoltage
   * f4 = Short circuit
   * f5 = Emergency stop
   * f6 = Brushless sensor fault
   * f7 = MOSFET failure
   * f8 = Default configuration loaded at startup
  */
  uint8_t fault_flag;
  success = Query(kReadFaultFlags, 0x00, 1, &data);
  fault_flag = static_cast<uint8_t>(data);

  /* request motor flags 
   * f1 = Amps Limit currently active
   * f2 = Motor stalled
   * f3 = Loop Error detected
   * f4 = Safety Stop active
   * f5 = Forward Limit triggered
   * f6 = Reverse Limit triggered
   * f7 = Amps Trigger activated
  */
  /* TODO: strange rules for opencan id */
  uint8_t left_motor_flag, right_motor_flag;
  success = Query(kReadMotorStatusFlags, 0x01, 4, &data);
  left_motor_flag = data;
  right_motor_flag = data >> 16;

  joints_[kLeftMotor-1].velocity = left_rpm / 60.0 / REDUCTION_RATIO * M_PI * 2.0;
  joints_[kRightMotor-1].velocity = right_rpm / 60.0 / REDUCTION_RATIO * M_PI * 2.0 * -1.0;

  joints_[kLeftMotor-1].position = l_count / 30.0 / REDUCTION_RATIO * M_PI * 2.0;
  joints_[kRightMotor-1].position = r_count / 30.0 / REDUCTION_RATIO * M_PI * 2.0 * -1.0;

  // ROS_INFO("motor counter: %d, %d, %d, %d", l_count, r_count, l_rpm, r_rpm);
  // ROS_INFO("motor counter: %f, %f, %d, %d", joints_[0].position, joints_[1].position, l_rpm, r_rpm);
  // ROS_INFO("status: %s, fault: %s, left: %s, right: %s", \
            ToBinary(status_flag, sizeof(status_flag)).c_str(), ToBinary(fault_flag, sizeof(fault_flag)).c_str(), \
            ToBinary(left_motor_flag, sizeof(left_motor_flag)).c_str(), ToBinary(right_motor_flag, sizeof(right_motor_flag)).c_str());
}

/**
* Get current encoder travel offsets from MCU and bias future encoder readings against them
*/
void iqr::CasterHardware::ResetTravelOffset() {

}

/**
* Register interfaces with the RobotHW interface manager, allowing ros_control operation
*/
void iqr::CasterHardware::RegisterControlInterfaces() {
  hardware_interface::JointStateHandle left_wheel_joint_state_handle(left_wheel_joint_, &joints_[0].position, &joints_[0].velocity, &joints_[0].effort);
  joint_state_interface_.registerHandle(left_wheel_joint_state_handle);

  hardware_interface::JointHandle left_wheel_joint_handle(left_wheel_joint_state_handle, &joints_[0].velocity_command);
  velocity_joint_interface_.registerHandle(left_wheel_joint_handle);

  hardware_interface::JointStateHandle right_wheel_joint_state_handle(right_wheel_joint_, &joints_[1].position, &joints_[1].velocity, &joints_[1].effort);
  joint_state_interface_.registerHandle(right_wheel_joint_state_handle);

  hardware_interface::JointHandle right_wheel_joint_handle(right_wheel_joint_state_handle, &joints_[1].velocity_command);
  velocity_joint_interface_.registerHandle(right_wheel_joint_handle);

  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);
}

void iqr::CasterHardware::WriteCommandsToHardware() {
  int32_t speed[2];

  speed[0] = static_cast<int32_t>(joints_[0].velocity_command / M_PI / 2.0 * REDUCTION_RATIO * 60);
  // int16_t s_v = ntohl(speed);
  // memcpy(buf+5, &s_v, 4);
  Command(kSetVelocity, static_cast<uint8_t>(kLeftMotor), static_cast<uint32_t>(speed[0]), 4);
  // SendCanOpenData(1, kCommand, kSetVelocity, static_cast<uint8_t>(kLeftMotor), static_cast<uint32_t>(speed[0]), 4);

  speed[1] = static_cast<int32_t>(joints_[1].velocity_command / M_PI / 2.0 * REDUCTION_RATIO * 60) * -1.0;
  // s_v = ntohl(speed);
  // memcpy(buf+9, &s_v, 4);
  Command(kSetVelocity, static_cast<uint8_t>(kRightMotor), static_cast<uint32_t>(speed[1]), 4);
  // SendCanOpenData(1, kCommand, kSetVelocity, static_cast<uint8_t>(kRightMotor), static_cast<uint32_t>(speed[1]), 4);

  // ROS_INFO("command: %f, %f; rad: %d, %d", joints_[0].velocity_command, joints_[1].velocity_command, speed[0], speed[1]);
}


void iqr::CasterHardware::SendCanOpenData(uint32_t node_id, RoboteqClientCommandType type, RoboteqCanOpenObjectDictionary index, uint8_t sub_index, uint32_t data, uint8_t data_length) {
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
    ROS_WARN("canopen send error, %s", strerror(errno));
  }
  // ROS_INFO("Sent");
}

bool iqr::CasterHardware::Query(RoboteqCanOpenObjectDictionary query, uint8_t sub_index, uint8_t data_length, uint32_t *data_received) {
  uint8_t buf[100];
  bzero(buf, 100);
  int received_bytes = 0;

  // static int count = 0;
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

bool iqr::CasterHardware::Command(RoboteqCanOpenObjectDictionary query, uint8_t sub_index, uint32_t data, uint8_t data_length) {
  uint8_t buf[100];
  bzero(buf, 100);
  int received_bytes = 0;

  // static int count = 0;
  // ROS_INFO("command count %d", count++);

  bool get_data = false;
  while(!get_data) {
    SendCanOpenData(can_id_, kCommand, query, sub_index, data, data_length);

    while(recv(socket_fd_, buf, 13, MSG_PEEK)>0) {
      received_bytes=recv(socket_fd_, buf, 13, 0);
      // ROS_INFO("recv complete");
      // ROS_INFO("get data:%d", received_bytes);
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
            continue;
            // retursn false;
          }

          if(buf[5] != 0x60) {
            ROS_WARN("incorrect response status, expect 0x60, but 0x%X received", buf[5]);
            continue;
            // return false;
          }

          uint32_t index = (buf[7] << 8) + buf[6];
          if(query != index) {
            ROS_WARN("incorrect index, expect 0x%X, but 0x%X received", query, index);
            continue;
            // return false;
          }

          if(sub_index != buf[8]) {
            ROS_WARN("incorrect sub_index, expect 0x%02X, but 0x%02X received", sub_index, buf[8]);
            continue;
            // return false;
          }

          get_data = true;
          break;
        }
      }
    }
  }

  return true;
}