#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "vehicle_interfaces/msg/vehicle_state.hpp"

#include <cmath>
#include <string>

namespace vehicle_simulator
{

struct AckermannState
{
  double x = 0.0;        
  double y = 0.0;        
  double theta = 0.0;    
  double speed = 0.0;    
  double steering = 0.0; 
  double vx = 0.0;       
  double vy = 0.0;       
  double omega = 0.0;    
};

class VehicleSimulator : public rclcpp::Node
{
public:
  VehicleSimulator();

private:

  double wheelbase_;        
  double max_speed_;        
  double max_steering_;     
  double max_accel_;       
  double max_steer_rate_;   
  double dt_;               
  double update_rate_;      


  AckermannState state_;
  double cmd_speed_ = 0.0;
  double cmd_steering_ = 0.0;
  double total_distance_ = 0.0;

  nav_msgs::msg::Path true_path_;

  rclcpp::Publisher<vehicle_interfaces::msg::VehicleState>::SharedPtr state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void simulationStep();


  void updateAckermann(double dt);


  visualization_msgs::msg::Marker createVehicleMarker();
  visualization_msgs::msg::Marker createAxesMarker();
};

}  
