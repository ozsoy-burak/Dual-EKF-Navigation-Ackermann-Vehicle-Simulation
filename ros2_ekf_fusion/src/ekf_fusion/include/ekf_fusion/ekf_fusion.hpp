#ifndef LOCAL_EKF_NODE_HPP_
#define LOCAL_EKF_NODE_HPP_

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

struct OdomMeasurement {
  double dt;          //zaman farki
  Eigen::VectorXd z;  //olcum vektoru
  Eigen::MatrixXd R;  //olcum gurultusu (kovaryans matrisi)
};

struct ImuMeasurement {
  double dt;          //zaman farki
  Eigen::VectorXd z;  //olcum vektoru
  Eigen::MatrixXd R;  //olcum gurultusu (kovaryans matrisi)
};

class LocalEkfNode : public rclcpp::Node{
  public:
    LocalEkfNode();

  private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;  
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr camera_odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;


    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;

    nav_msgs::msg::Path global_path_msg_;
    nav_msgs::msg::Path path_msg_; 
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time last_time_;
    double offset_x_ = 0.0;
    double offset_y_ = 0.0;
    double offset_yaw_ = 0.0;

    bool rtk_initialized_ = false;
    bool is_initialized_ = false;  //Ilk veri gelene kadarki zaman hesaplamak icin

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
    void publishLocalOdom();
    void rtkCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);};

#endif // LOCAL_EKF_NODE_HPP_
