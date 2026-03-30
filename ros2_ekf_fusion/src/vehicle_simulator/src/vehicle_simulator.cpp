#include "vehicle_simulator/vehicle_simulator.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace vehicle_simulator
{

VehicleSimulator::VehicleSimulator()
: Node("vehicle_simulator")
{
  // Declare parameters
  declare_parameter("wheelbase", 2.7);           
  declare_parameter("max_speed", 15.0);          
  declare_parameter("max_steering_angle", 0.52); 
  declare_parameter("max_acceleration", 3.0);    
  declare_parameter("max_steering_rate", 1.0);  
  declare_parameter("update_rate", 50.0);        
  declare_parameter("initial_x", 0.0);
  declare_parameter("initial_y", 0.0);
  declare_parameter("initial_theta", 0.0);

  wheelbase_    = get_parameter("wheelbase").as_double();
  max_speed_    = get_parameter("max_speed").as_double();
  max_steering_ = get_parameter("max_steering_angle").as_double();
  max_accel_    = get_parameter("max_acceleration").as_double();
  max_steer_rate_ = get_parameter("max_steering_rate").as_double();
  update_rate_  = get_parameter("update_rate").as_double();
  dt_           = 1.0 / update_rate_;

  state_.x     = get_parameter("initial_x").as_double();
  state_.y     = get_parameter("initial_y").as_double();
  state_.theta = get_parameter("initial_theta").as_double();

  state_pub_  = create_publisher<vehicle_interfaces::msg::VehicleState>(
    "vehicle/true_state", rclcpp::QoS(10));
  path_pub_   = create_publisher<nav_msgs::msg::Path>(
    "vehicle/true_path", rclcpp::QoS(10));
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "vehicle/markers", rclcpp::QoS(10));
  distance_pub_ = create_publisher<std_msgs::msg::Float64>(
    "vehicle/total_distance", rclcpp::QoS(10));

  cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(10),
    std::bind(&VehicleSimulator::cmdCallback, this, std::placeholders::_1));

  timer_ = create_wall_timer(
    std::chrono::duration<double>(dt_),
    std::bind(&VehicleSimulator::simulationStep, this));

  true_path_.header.frame_id = "map";

  RCLCPP_INFO(get_logger(),
    "VehicleSimulator started. Wheelbase=%.2fm, MaxSpeed=%.1fm/s, MaxSteer=%.2frad",
    wheelbase_, max_speed_, max_steering_);
  RCLCPP_INFO(get_logger(),
    "Send commands to /cmd_vel: linear.x=target_speed(m/s), angular.z=steering_angle(rad)");
}

void VehicleSimulator::cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_speed_    = std::clamp(msg->linear.x,  -max_speed_,    max_speed_);
  cmd_steering_ = std::clamp(msg->angular.z, -max_steering_, max_steering_);
}

void VehicleSimulator::updateAckermann(double dt)
{
  double speed_error = cmd_speed_ - state_.speed;
  double max_delta_v = max_accel_ * dt;
  state_.speed += std::clamp(speed_error, -max_delta_v, max_delta_v);

  double steer_error = cmd_steering_ - state_.steering;
  double max_delta_s = max_steer_rate_ * dt;
  state_.steering += std::clamp(steer_error, -max_delta_s, max_delta_s);
  state_.steering = std::clamp(state_.steering, -max_steering_, max_steering_);

  double tan_delta = std::tan(state_.steering);
  state_.omega = (state_.speed * tan_delta) / wheelbase_;

  state_.theta += state_.omega * dt;

  while (state_.theta >  M_PI) state_.theta -= 2.0 * M_PI;
  while (state_.theta < -M_PI) state_.theta += 2.0 * M_PI;

  state_.vx = state_.speed * std::cos(state_.theta);
  state_.vy = state_.speed * std::sin(state_.theta);

  double prev_x = state_.x;
  double prev_y = state_.y;
  state_.x += state_.vx * dt;
  state_.y += state_.vy * dt;

  double dx = state_.x - prev_x;
  double dy = state_.y - prev_y;
  total_distance_ += std::sqrt(dx*dx + dy*dy);
}

void VehicleSimulator::simulationStep()
{
  updateAckermann(dt_);

  auto now = get_clock()->now();

  vehicle_interfaces::msg::VehicleState state_msg;
  state_msg.header.stamp    = now;
  state_msg.header.frame_id = "map";
  state_msg.x              = state_.x;
  state_msg.y              = state_.y;
  state_msg.theta          = state_.theta;
  state_msg.vx             = state_.vx;
  state_msg.vy             = state_.vy;
  state_msg.omega          = state_.omega;
  state_msg.speed          = state_.speed;
  state_msg.steering_angle = state_.steering;
  state_pub_->publish(state_msg);

  true_path_.header.stamp = now;
  geometry_msgs::msg::PoseStamped ps;
  ps.header = true_path_.header;
  ps.pose.position.x = state_.x;
  ps.pose.position.y = state_.y;
  ps.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, state_.theta);
  ps.pose.orientation = tf2::toMsg(q);
  true_path_.poses.push_back(ps);

  if (true_path_.poses.size() > 5000) {
    true_path_.poses.erase(true_path_.poses.begin());
  }
  path_pub_->publish(true_path_);

  std_msgs::msg::Float64 dist_msg;
  dist_msg.data = total_distance_;
  distance_pub_->publish(dist_msg);

  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.push_back(createVehicleMarker());
  marker_array.markers.push_back(createAxesMarker());
  marker_pub_->publish(marker_array);
}

visualization_msgs::msg::Marker VehicleSimulator::createVehicleMarker()
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = "map";
  m.header.stamp    = get_clock()->now();
  m.ns              = "vehicle";
  m.id              = 0;
  m.type            = visualization_msgs::msg::Marker::CUBE;
  m.action          = visualization_msgs::msg::Marker::ADD;
  m.pose.position.x = state_.x;
  m.pose.position.y = state_.y;
  m.pose.position.z = 0.4;
  tf2::Quaternion q;
  q.setRPY(0, 0, state_.theta);
  m.pose.orientation = tf2::toMsg(q);
  m.scale.x = wheelbase_ + 0.5;  // length
  m.scale.y = 1.8;               // width
  m.scale.z = 0.8;               // height
  m.color.r = 0.2f;
  m.color.g = 0.6f;
  m.color.b = 1.0f;
  m.color.a = 0.9f;
  return m;
}

visualization_msgs::msg::Marker VehicleSimulator::createAxesMarker()
{
  // Arrow showing heading direction
  visualization_msgs::msg::Marker m;
  m.header.frame_id = "map";
  m.header.stamp    = get_clock()->now();
  m.ns              = "vehicle_heading";
  m.id              = 1;
  m.type            = visualization_msgs::msg::Marker::ARROW;
  m.action          = visualization_msgs::msg::Marker::ADD;
  m.pose.position.x = state_.x;
  m.pose.position.y = state_.y;
  m.pose.position.z = 0.9;
  tf2::Quaternion q;
  q.setRPY(0, 0, state_.theta);
  m.pose.orientation = tf2::toMsg(q);
  m.scale.x = 3.0;  // arrow length
  m.scale.y = 0.3;  // arrow width
  m.scale.z = 0.3;
  m.color.r = 1.0f;
  m.color.g = 0.3f;
  m.color.b = 0.0f;
  m.color.a = 1.0f;
  return m;
}

}  