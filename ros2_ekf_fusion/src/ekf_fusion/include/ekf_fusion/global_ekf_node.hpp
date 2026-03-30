#ifndef GLOBAL_EKF_NODE_HPP_
#define GLOBAL_EKF_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <Eigen/Dense>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "ekf_fusion/ekf_fusion.hpp" 
#include "std_msgs/msg/float64.hpp" 

class GlobalEkfNode : public rclcpp::Node {
  public:
    GlobalEkfNode();

  private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;  
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr camera_odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr rtk_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr offset_sub_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_; // Global Path
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    
    nav_msgs::msg::Path path_msg_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time last_time_;
    bool is_initialized_ = false;

    double local_x_ = 0.0;
    double local_y_ = 0.0;
    double local_yaw_ = 0.0;
    double test_offset_x_ = 0.0;
    bool local_odom_ready_ = false;

    Eigen::VectorXd X_; 
    Eigen::MatrixXd P_; 
    Eigen::MatrixXd Q_; 
    Eigen::MatrixXd F_; 

    void wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void cameraOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void predict(double dt);
    void updateWheelOdom(const OdomMeasurement& meas);
    void updateCameraOdom(const OdomMeasurement& meas);
    void updateImu(const ImuMeasurement& meas);

    void offsetCallback(const std_msgs::msg::Float64::SharedPtr msg);
    
    void rtkCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void localOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void updateGps(const OdomMeasurement& meas);
    void publishGlobalOdom(); 
};
#endif // GLOBAL_EKF_NODE_HPP_
