#include "ekf_fusion/ekf_fusion.hpp"
#include "ekf_fusion/global_ekf_node.hpp" 
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Iki EKF dugumunu de bellekte olusturuyoruz
    auto local_node = std::make_shared<LocalEkfNode>();
    auto global_node = std::make_shared<GlobalEkfNode>();

    // ROS 2'nin coklu is parcacigi yoneticisi (Multi-Threaded Executor)
    // Bu sayede iki EKF birbirini beklemeden paralel cekirdeklerde calisir
    rclcpp::executors::MultiThreadedExecutor executor;
    
    // Dugumleri yoneticiye ekliyoruz
    executor.add_node(local_node);
    executor.add_node(global_node);

    // Her iki dugum de ayni anda donmeye (spin) basliyor
    executor.spin();

    rclcpp::shutdown();
    return 0;
}