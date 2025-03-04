#ifndef OMY_HAT_BRINGUP_NODE_HPP_
#define OMY_HAT_BRINGUP_NODE_HPP_

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

namespace dynamixel {

class OmyHatBringup : public rclcpp::Node
{
public:
  OmyHatBringup();
  ~OmyHatBringup();

private:
  void initializePower();
  void writeControlValue(uint16_t address, uint8_t value, const char* control_name);

  // Constants
  static constexpr uint16_t ADDR_POWER_ENABLE = 512;  // 1 byte
  static constexpr uint16_t ADDR_VOLTAGE_CONTROL_ENABLE = 513;  // 1 byte
  static constexpr float PROTOCOL_VERSION = 2.0;
  static constexpr uint8_t DXL_ID = 200;
  static constexpr int BAUDRATE = 4000000;
  static constexpr const char* DEVICENAME = "/dev/ttyAMA2";

  // Class members
  PortHandler *portHandler;
  PacketHandler *packetHandler;
};

}  // namespace dynamixel

#endif  // OMY_HAT_BRINGUP_NODE_HPP_ 
