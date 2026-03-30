#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <algorithm>

class PurePursuitNode : public rclcpp::Node {
public:
    const double WHEELBASE = 2.7;               
    const double MAX_STEERING = 0.52;           
    const double MAX_SPEED = 3.0;            
    const double MIN_SPEED = 1.0;               
    const double SLOW_DOWN_DIST = 3.0;         
    const double GOAL_TOLERANCE = 0.5;          
    const double LOOKAHEAD_MIN = 1.5;           

    PurePursuitNode() : Node("pure_pursuit_node") {
        goal_set_ = false;

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "ekf/local_odometry", 10,
            std::bind(&PurePursuitNode::odomCallback, this, std::placeholders::_1));

        target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10,
            std::bind(&PurePursuitNode::targetCallback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("pp_markers", 10);
    }

private:
    geometry_msgs::msg::PoseStamped goal_msg_;
    bool goal_set_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    void targetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        goal_msg_ = *msg;
        goal_set_ = true;
        RCLCPP_INFO(this->get_logger(), "Yeni Hedef ve Yaw Alındı: (%.2f, %.2f)", 
                    goal_msg_.pose.position.x, goal_msg_.pose.position.y);
        publishGoalMarker(goal_msg_);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!goal_set_) return;

        geometry_msgs::msg::PoseStamped goal_in_base;
        
        // TF zaman damgasini en guncelli kullanir
        goal_msg_.header.stamp = rclcpp::Time(0); 

        try {
            // Bu islem 'map -> odom_local -> base_link' zincirini birlestirir.
            goal_in_base = tf_buffer_->transform(goal_msg_, "base_link");
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "TF donusumu bekleniyor: %s", ex.what());
            return;
        }

        double tx = goal_in_base.pose.position.x;
        double ty = goal_in_base.pose.position.y;
        double actual_dist = std::hypot(tx, ty);

        // 1. Durus Islemleri (Ackermann in-place donemez, sadece XY mesafesine bakilir)
        if (actual_dist < GOAL_TOLERANCE) { 
            stopVehicle();
            goal_set_ = false;
            return;
        }

        double Ld = std::max(LOOKAHEAD_MIN, actual_dist);
        double steering_angle = std::atan2(2.0 * WHEELBASE * ty, Ld * Ld);
        steering_angle = std::clamp(steering_angle, -MAX_STEERING, MAX_STEERING);

        double speed_by_dist = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * std::clamp(actual_dist / SLOW_DOWN_DIST, 0.0, 1.0);
        double speed_by_steer = MAX_SPEED - (MAX_SPEED - MIN_SPEED) * std::clamp(std::abs(steering_angle) / MAX_STEERING, 0.0, 1.0);
        
        double target_v = std::min(speed_by_dist, speed_by_steer);

        if (tx < 0.0) {
            target_v = MIN_SPEED;
        }
        
        publishCommand(target_v, steering_angle);
        publishPathMarker(tx, ty);
    }

    void publishCommand(double v, double steering) {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = v;
        cmd.angular.z = steering;
        cmd_pub_->publish(cmd);
    }

    void stopVehicle() {
        publishCommand(0.0, 0.0);
        RCLCPP_INFO(this->get_logger(), "Hedef Noktaya Ulaşıldı!");
    }

    void publishGoalMarker(const geometry_msgs::msg::PoseStamped &goal) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "goal";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = goal.pose;
        marker.scale.x = 0.7; 
        marker.scale.y = 0.2; 
        marker.scale.z = 0.2;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker_pub_->publish(marker);
    }

    void publishPathMarker(double x, double y) {
        visualization_msgs::msg::Marker path_marker;
        // Marker base_link referansinda yayinlaniyor ki hedefin aractaki konumu belli olsun
        path_marker.header.frame_id = "base_link"; 
        path_marker.header.stamp = this->now();
        path_marker.ns = "pp_path";
        path_marker.id = 1;
        path_marker.type = visualization_msgs::msg::Marker::SPHERE;
        path_marker.action = visualization_msgs::msg::Marker::ADD;
        path_marker.pose.position.x = x;
        path_marker.pose.position.y = y;
        path_marker.pose.position.z = 0.0;
        path_marker.scale.x = 0.2;
        path_marker.scale.y = 0.2;
        path_marker.scale.z = 0.2;
        path_marker.color.r = 0.0;
        path_marker.color.g = 1.0;
        path_marker.color.b = 0.0;
        path_marker.color.a = 1.0;
        marker_pub_->publish(path_marker);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitNode>());
    rclcpp::shutdown();
    return 0;
}