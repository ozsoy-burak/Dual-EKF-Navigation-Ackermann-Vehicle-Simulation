#include "vehicle_simulator/vehicle_simulator.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vehicle_simulator::VehicleSimulator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
