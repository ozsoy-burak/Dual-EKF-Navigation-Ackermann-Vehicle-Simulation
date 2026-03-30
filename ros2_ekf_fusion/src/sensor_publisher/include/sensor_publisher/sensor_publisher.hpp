#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "vehicle_interfaces/msg/vehicle_state.hpp"

#include <thread>
#include <mutex>
#include <atomic>
#include <random>
#include <functional>

namespace sensor_publisher
{

// ── Thread-safe latest vehicle state ─────────────────────────────────────────
struct SharedState
{
  std::mutex mtx;
  vehicle_interfaces::msg::VehicleState state;
  bool valid = false;
};

// ── Noise model for each sensor axis ─────────────────────────────────────────
struct NoiseModel
{
  double std_dev    = 0.0;   // Gaussian std deviation
  double bias       = 0.0;   // Constant bias
  double drift_rate = 0.0;   // Random walk drift (std/s)
  double bias_current = 0.0; // Runtime accumulated bias
};

class SensorPublisher : public rclcpp::Node
{
public:
  SensorPublisher();
  ~SensorPublisher();

private:
  // Shared ground truth
  std::shared_ptr<SharedState> shared_state_;

  // Subscriber
  rclcpp::Subscription<vehicle_interfaces::msg::VehicleState>::SharedPtr state_sub_;

  // Publishers (each sensor publishes to its own topic)
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr camera_odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr std_gps_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr std_gps_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr rtk_gps_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr rtk_gps_pose_pub_;
  // Sensor threads
  std::thread wheel_odom_thread_;
  std::thread imu_thread_;
  std::thread camera_odom_thread_;
  std::thread std_gps_thread_;
  std::thread rtk_gps_thread_;
  std::atomic<bool> running_{true};

  // Random number generation
  std::mt19937 rng_;

  // ── Sensor noise parameters ──────────────────────────────────────────────
  // Wheel Odometry
  double wheel_linear_x_std_;    // linear velocity noise std (m/s)
  double wheel_angular_z_std_;   // angular velocity noise std (rad/s)
  double wheel_linear_bias_;     // systematic bias on linear vel
  double wheel_drift_rate_;      // drift rate (m/s per sqrt-s)
  double wheel_rate_hz_;

  // IMU
  double imu_accel_std_;         // m/s^2
  double imu_gyro_std_;          // rad/s
  double imu_accel_bias_;        // m/s^2
  double imu_gyro_bias_;         // rad/s
  double imu_accel_drift_;       // random walk
  double imu_gyro_drift_;
  double imu_rate_hz_;

  // Camera Odometry
  double cam_linear_std_;        // m/s
  double cam_angular_std_;       // rad/s
  double cam_scale_error_;       // scale factor error (0.02 = 2%)
  double cam_rate_hz_;

  // GPS
  double gps_origin_lat_, gps_origin_lon_;
  // Std GPS Parametreleri
  double std_gps_rate_hz_, std_gps_pos_std_, std_gps_multipath_std_, std_gps_multipath_prob_;
  // RTK GPS Parametreleri
  double rtk_rate_hz_, rtk_fix_std_, rtk_float_std_, rtk_heading_std_, rtk_float_prob_;     // reference longitude

  // Runtime drift state
  double imu_accel_drift_current_ = 0.0;
  double imu_gyro_drift_current_  = 0.0;
  double wheel_drift_current_     = 0.0;

  // Helper: apply Gaussian noise
  double addNoise(double value, double std_dev);
  double sampleGaussian(double mean, double std_dev);
  double sampleGaussian(double std_dev) { return sampleGaussian(0.0, std_dev); }

  // Sensor loop functions (run in threads)
  void wheelOdomLoop();
  void imuLoop();
  void cameraOdomLoop();
  void stdGpsLoop();
  void rtkGpsLoop();
  // State callback
  void stateCallback(const vehicle_interfaces::msg::VehicleState::SharedPtr msg);

  // Convert local XY → GPS lat/lon (flat-earth approximation)
  std::pair<double,double> xyToLatLon(double x, double y);
};

}  // namespace sensor_publisher
