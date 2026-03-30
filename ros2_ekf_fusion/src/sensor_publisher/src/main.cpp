#include "sensor_publisher/sensor_publisher.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sensor_publisher::SensorPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
