#include "omy_hat_bringup/omy_hat_bringup_node.hpp"

namespace dynamixel {

OmyHatBringup::OmyHatBringup()
: Node("omy_hat_bringup"), portHandler(nullptr), packetHandler(nullptr)
{
  try {
    initializePower();
    RCLCPP_INFO(this->get_logger(), "Power initialization completed successfully, wait for Dynamixel Y to boot up...");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    RCLCPP_INFO(this->get_logger(), "Dynamixel Y booted up");
    rclcpp::shutdown();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Power initialization failed: %s", e.what());
    if (portHandler != nullptr) {
      portHandler->closePort();
    }
    rclcpp::shutdown();
  }
}

OmyHatBringup::~OmyHatBringup()
{
  if (portHandler != nullptr) {
    portHandler->closePort();
  }
}

void OmyHatBringup::initializePower()
{
  // Initialize PortHandler instance
  portHandler = PortHandler::getPortHandler(DEVICENAME);
  if (portHandler == nullptr) {
    throw std::runtime_error("Failed to get port handler");
  }
  
  // Initialize PacketHandler instance
  packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  if (packetHandler == nullptr) {
    throw std::runtime_error("Failed to get packet handler");
  }
  
  // Open port
  if (!portHandler->openPort()) {
    throw std::runtime_error("Failed to open port");
  }
  RCLCPP_INFO(this->get_logger(), "Port opened successfully: %s", DEVICENAME);
  
  // Set port baudrate
  if (!portHandler->setBaudRate(BAUDRATE)) {
    throw std::runtime_error("Failed to set baudrate");
  }
  RCLCPP_INFO(this->get_logger(), "Baudrate set to: %d", BAUDRATE);

  // Enable power and voltage control
  writeControlValue(ADDR_POWER_ENABLE, 0, "power enable");
  std::this_thread::sleep_for(std::chrono::seconds(1));
  writeControlValue(ADDR_POWER_ENABLE, 1, "power enable");
  writeControlValue(ADDR_VOLTAGE_CONTROL_ENABLE, 1, "voltage control enable");
}

void OmyHatBringup::writeControlValue(uint16_t address, uint8_t value, const char* control_name)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL_ID, address, value, &dxl_error);
  
  if (dxl_comm_result != COMM_SUCCESS) {
    std::string error_msg = "Failed to write " + std::string(control_name) + ": " +
      std::string(packetHandler->getTxRxResult(dxl_comm_result));
    throw std::runtime_error(error_msg);
  }
  
  if (dxl_error != 0) {
    std::string error_msg = "Error writing " + std::string(control_name) + ": " +
      std::string(packetHandler->getRxPacketError(dxl_error));
    throw std::runtime_error(error_msg);
  }
  
  RCLCPP_INFO(this->get_logger(), "%s written successfully", control_name);
}

}  // namespace dynamixel

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dynamixel::OmyHatBringup>();
  rclcpp::shutdown();
  return 0;
}
