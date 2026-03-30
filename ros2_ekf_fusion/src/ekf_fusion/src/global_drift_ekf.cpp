#include "ekf_fusion/global_ekf_node.hpp"

GlobalEkfNode::GlobalEkfNode() : Node("global_ekf_node"){
   X_ = Eigen::VectorXd::Zero(8); 
   P_ = Eigen::MatrixXd::Identity(8, 8) * 1e-3;

   Q_ = Eigen::MatrixXd::Identity(8, 8);
   Q_(0,0) = 0.01; Q_(1,1) = 0.01; Q_(2,2) = 0.01; Q_(3,3) = 0.1; 
   Q_(4,4) = 1e-9; Q_(5,5) = 0.02; Q_(6,6) = 0.01; Q_(7,7) = 1e-9; 

   wheel_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "sensors/wheel_odometry", 10, std::bind(&GlobalEkfNode::wheelOdomCallback, this, std::placeholders::_1));

   camera_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "sensors/camera_odometry", 10, std::bind(&GlobalEkfNode::cameraOdomCallback, this, std::placeholders::_1));

   imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "sensors/imu", 10, std::bind(&GlobalEkfNode::imuCallback, this, std::placeholders::_1));

   rtk_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/sensors/gps_rtk/pose", 10, std::bind(&GlobalEkfNode::rtkCallback, this, std::placeholders::_1));

   local_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "ekf/local_odometry", 10, std::bind(&GlobalEkfNode::localOdomCallback, this, std::placeholders::_1));

   offset_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/debug/gps_offset", 10, std::bind(&GlobalEkfNode::offsetCallback, this, std::placeholders::_1));

   odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("ekf/global_odometry", 10);
   path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/ekf/global_path", 10);
   tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
   
   timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&GlobalEkfNode::publishGlobalOdom, this));

   RCLCPP_INFO(this->get_logger(), "Global EKF Node basariyla baslatildi. RTK dinleniyor...");
}

void GlobalEkfNode::publishGlobalOdom() {
    if (!is_initialized_ || !local_odom_ready_) return;

    rclcpp::Time current_time = this->get_clock()->now();

    // 1. global EKF'nin buldugu kesin ve mutlak Konum
    double global_x = X_(0);
    double global_y = X_(1);
    double global_yaw = X_(2);

    // 2. aradaki drifti hesaplama
    double offset_yaw = global_yaw - local_yaw_;
    while (offset_yaw > M_PI)  offset_yaw -= 2.0 * M_PI;
    while (offset_yaw < -M_PI) offset_yaw += 2.0 * M_PI;

    double cos_y = std::cos(offset_yaw);
    double sin_y = std::sin(offset_yaw);

    double offset_x = global_x - (local_x_ * cos_y - local_y_ * sin_y);
    double offset_y = global_y - (local_x_ * sin_y + local_y_ * cos_y);

    // 3. MAP -> ODOM_LOCAL TF AGACINI YAYINLA
    geometry_msgs::msg::TransformStamped t_map;
    t_map.header.stamp = current_time;
    t_map.header.frame_id = "map";
    t_map.child_frame_id = "odom_local";

    t_map.transform.translation.x = offset_x;
    t_map.transform.translation.y = offset_y;
    t_map.transform.translation.z = 0.0;

    tf2::Quaternion q_offset;
    q_offset.setRPY(0.0, 0.0, offset_yaw);
    t_map.transform.rotation.x = q_offset.x();
    t_map.transform.rotation.y = q_offset.y();
    t_map.transform.rotation.z = q_offset.z();
    t_map.transform.rotation.w = q_offset.w();

    tf_broadcaster_->sendTransform(t_map);

    path_msg_.header.stamp = current_time;
    path_msg_.header.frame_id = "map";

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = current_time;
    pose_stamped.header.frame_id = "map";
    pose_stamped.pose.position.x = global_x;
    pose_stamped.pose.position.y = global_y;
    
    tf2::Quaternion q_global;
    q_global.setRPY(0.0, 0.0, global_yaw);
    pose_stamped.pose.orientation = tf2::toMsg(q_global);

    path_msg_.poses.push_back(pose_stamped);

    nav_msgs::msg::Odometry global_odom_msg;
    global_odom_msg.header.stamp = current_time;
    global_odom_msg.header.frame_id = "map";
    global_odom_msg.child_frame_id = "map";

    global_odom_msg.pose.pose.position.x = global_x;
    global_odom_msg.pose.pose.position.y = global_y;
    global_odom_msg.pose.pose.position.z = 0.0;
    global_odom_msg.pose.pose.orientation = tf2::toMsg(q_global);

    global_odom_msg.twist.twist.linear.x = X_(3);
    global_odom_msg.twist.twist.linear.y = X_(4);
    global_odom_msg.twist.twist.angular.z = X_(5);

    odom_pub_->publish(global_odom_msg);
    
    if (path_msg_.poses.size() > 5000) {
        path_msg_.poses.erase(path_msg_.poses.begin());
    }

    path_pub_->publish(path_msg_);
}

void GlobalEkfNode::predict(double dt) {

    if (std::abs(X_(5)) < 0.008) {
        X_(5) = 0.0;
    }

    double x     = X_(0);
    double y     = X_(1);
    double theta = X_(2);
    double vx    = X_(3);
    double vy    = X_(4);
    double omega = X_(5);
    double ax    = X_(6);
    double ay    = X_(7);

    double cos_t = std::cos(theta);
    double sin_t = std::sin(theta);

    //Durum tahmiini
    X_(0) = x + (vx * cos_t - vy * sin_t) * dt; //position.x
    X_(1) = y + (vx * sin_t + vy * cos_t) * dt; //position.y
    X_(2) = theta + omega * dt; //yaw
    X_(3) = vx + ax * dt;  //vx
    X_(4) = vy + ay * dt;  //vy


    // Aciyi -Pi ile +Pi arasinda tutmak icin normalizasyon
    while (X_(2) > M_PI)  X_(2) -= 2.0 * M_PI;
    while (X_(2) < -M_PI) X_(2) += 2.0 * M_PI;

    //Jacobian Matrisi (F)
    F_ = Eigen::MatrixXd::Identity(8,8);

    //kismi turevler
    //Stateteki kucuk hata, diger stateleri nasil etkiliyor bulmak icin?
    //Ornek: theta degisirse, x ne kadar degisir gibi. 

    // x'in turevleri:
    F_(0, 2) = (-vx * sin_t - vy * cos_t) * dt; // dx/d(theta)
    F_(0, 3) = cos_t * dt;                      // dx/d(vx)
    F_(0, 4) = -sin_t * dt;                     // dx/d(vy)

    // y'nin turevleri:
    F_(1, 2) = (vx * cos_t - vy * sin_t) * dt;  // dy/d(theta)
    F_(1, 3) = sin_t * dt;                      // dy/d(vx)
    F_(1, 4) = cos_t * dt;                      // dy/d(vy)

    // theta, vx, vy turevleri:
    F_(2, 5) = dt;                              // d(theta)/d(omega)
    F_(3, 6) = dt;                              // d(vx)/d(ax)
    F_(4, 7) = dt;                              // d(vy)/d(ay)

    //Q matrisi modele ne kadar guvenilmedigini belirtir.
    //Aracin fiziksel modeli muk olmadigi icin Q belirsizligi artirilir.
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void GlobalEkfNode::updateWheelOdom(const OdomMeasurement& meas){
    /*
    1. Sensor geldi (z)
    2. Tahminle farki bul (y)
    3. Bu fark guvenilir mi? (S)
    4. Ne kadar duzelteyim? (K)
    5. State i duzelt (X)
    6. Belirsizligi azalt (P)
    */

    //Gozlem Matrisi(H) - 2 veri var toplam 8 verilik alan var.
    //tekerden vx ve wz kullainilacak yani 2 satir-olcum sayisi ve 8 sutun(toplam state sayisi)
    // X = [x, y, theta, vx, vy, wz, ax, ay] toplam state
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2,8);

    H(0,3) = 1.0; //Vx kismi burasi, 1 ile aktif yapilir
    H(1,5) = 1.0;  //Wz kismi burasida, aktif gerisi sifir olmadigi icin

    //Yenilik vektoru
    //Olculen deger(z) ile EKF'nin tahmini (H*X) arasindaki fark 
    Eigen::VectorXd y = meas.z - (H * X_);  //olculen - ekf tahmini

    //Fark ne kadar guvenilir onu olcer
    //H * P* H^T ekfnin kendi hatasi, R ise sensorun hatasi
    //Birlesince toplam berlirsizlik bulunur.
    Eigen::MatrixXd S = H * P_ * H.transpose() + meas.R;

    //EKF kime guvenecegine karar verdigi an, Tahmine mi sensore mi? 
    //k buyukse sensore, kucukse kendine guven
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    //Durumu guncelleme
    //X_ = tahmin edilen durum, y hata miktari, K ne kadar duzeltilecek
    //X=1, y=0.2 ve k=0.5 ise X_ = 1 + (0.5*0.2) = 1.1 olur yeni tahmin
    X_ = X_ + (K * y);

    while (X_(2) > M_PI)  X_(2) -= 2.0 * M_PI;
    while (X_(2) < -M_PI) X_(2) += 2.0 * M_PI;

    //Artik daha az belirsizim, kovaryansi guncelleriz
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(8, 8);
    P_ = (I - K * H) * P_;

}

void GlobalEkfNode::updateCameraOdom(const OdomMeasurement& meas){

    //vx, vy, wz alinacak 3x8 matris

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 8);
    H(0, 3) = 1.0; //  vx
    H(1, 5) = 1.0; //  wz

    Eigen::VectorXd y = meas.z - (H * X_);
    Eigen::MatrixXd S = H * P_ * H.transpose() + meas.R;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
    X_ = X_ + (K * y);

    while (X_(2) > M_PI)  X_(2) -= 2.0 * M_PI;
    while (X_(2) < -M_PI) X_(2) += 2.0 * M_PI;

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(8, 8);
    P_ = (I - K * H) * P_;
}

void GlobalEkfNode::updateImu(const ImuMeasurement& meas){

    //wz, ax, ay alinacak 3x8 matris

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 8);
    H(0, 6) = 1.0; //  ax
    H(1, 5) = 1.0; //  wz

    Eigen::VectorXd y = meas.z - (H * X_);
    Eigen::MatrixXd S = H * P_ * H.transpose() + meas.R;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
    X_ = X_ + (K * y);

    while (X_(2) > M_PI)  X_(2) -= 2.0 * M_PI;
    while (X_(2) < -M_PI) X_(2) += 2.0 * M_PI;

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(8, 8);
    P_ = (I - K * H) * P_;
}

void GlobalEkfNode::updateGps(const OdomMeasurement& meas) {
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 8);
    H(0, 0) = 1.0; // X
    H(1, 1) = 1.0; // Y
    H(2, 2) = 1.0; // Yaw (Theta)

    Eigen::VectorXd y = meas.z - (H * X_);

    while (y(2) > M_PI)  y(2) -= 2.0 * M_PI;
    while (y(2) < -M_PI) y(2) += 2.0 * M_PI;

    // 1. HIZA DUYARLI ADAPTİF R (Velocity-Scheduled Covariance)
    double speed = std::abs(X_(3));
    double speed_penalty = 1.0;

    if (speed < 0.5) {
        speed_penalty = 15.0;  
    } else if (speed < 1.5) {
        speed_penalty = 5.0;   
    } else {
        speed_penalty = 1.0;   
    }

    Eigen::MatrixXd adaptive_R = meas.R;
    adaptive_R(0, 0) = std::max(meas.R(0, 0), 0.5) * speed_penalty;
    adaptive_R(1, 1) = std::max(meas.R(1, 1), 0.5) * speed_penalty;
    adaptive_R(2, 2) = std::max(meas.R(2, 2), 0.1) * speed_penalty;

    Eigen::MatrixXd S = H * P_ * H.transpose() + adaptive_R;

    double mahalanobis_dist = y.transpose() * S.inverse() * y;
    if (mahalanobis_dist > 3.0) {
        RCLCPP_WARN(this->get_logger(), "GPS Outlier: %.2f | Hiz: %.2f", mahalanobis_dist, speed);
        return; // Felaketleri engelle
    }

    double position_jump = std::sqrt(y(0)*y(0) + y(1)*y(1));
    
    double jump_penalty = 1.0 + (position_jump * position_jump * 25.0);

    adaptive_R = adaptive_R * jump_penalty;
    
    S = H * P_ * H.transpose() + adaptive_R;

    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
    
    X_ = X_ + (K * y);

    while (X_(2) > M_PI)  X_(2) -= 2.0 * M_PI;
    while (X_(2) < -M_PI) X_(2) += 2.0 * M_PI;

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(8, 8);
    P_ = (I - K * H) * P_;
}

void GlobalEkfNode::wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){

  rclcpp::Time current_time = msg->header.stamp;
  
  //Sistem ilk acildiginda zamani kaydeder. 
  if (!is_initialized_){
    last_time_ = current_time;
    is_initialized_ = true;
    return ;
  }

  //zaman farkini hesapliyoruz
  double dt = (current_time - last_time_).seconds();

  //Veri geriden akiyorsa veya fark asiri buyukse ihmal ederiz.
  if (dt <= 0.0 || dt > 1.0){
    return;
  }

  OdomMeasurement meas;
  meas.dt = dt;

  //2 boyutlu vektor actik, cunku tekerden vx ve wz degerleri kullanilacak, 2 deger.
  meas.z = Eigen::VectorXd(2);
  meas.z(0) = msg->twist.twist.linear.x;

  if(std::abs(msg->twist.twist.angular.z) < 0.008){
    meas.z(1) =  0.0;
  } else {
    meas.z(1) = msg->twist.twist.angular.z;
  }

  //Kovaryans yani olcum gurulusu matrisini dolduracagiz.
  meas.R = Eigen::MatrixXd(2,2); //diger iki deger birbiriyle iliskisi var mi onu olcer
  meas.R(0,0) = msg->twist.covariance[0]; //vx gurultusu
  meas.R(0,1) = 0.0; //cov(vx, omega), vx hatasi artarsa omega da artar
  meas.R(1,0) = 0.0;  //cov(omega, vx), omega hatasi artarsa vx de artar
  meas.R(1,1) = msg->twist.covariance[35]; //wz hatasi 
 
  last_time_ = current_time;

  predict(meas.dt);
  updateWheelOdom(meas);

  RCLCPP_INFO(this->get_logger(), "Odom Paketi Hazir | dt: %.4f | Vx: %.2f", 
              meas.dt, meas.z(0));
  //double x = msg->pose.pose.position.x;
  //R7CLCPP_INFO(this->get_logger(), "Current data: %.2f ", x);
}

void GlobalEkfNode::cameraOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  rclcpp::Time current_time = msg->header.stamp;
  
  if (!is_initialized_){
    last_time_ = current_time;
    is_initialized_ = true;
    return ;
  }

  double dt = (current_time - last_time_).seconds();

  if (dt <= 0.0 || dt > 1.0){
    return;
  }
  
  OdomMeasurement meas_cam;

  meas_cam.dt = dt; 
  
  //vx, vy ve wz degerleri alinacaktir.
  meas_cam.z = Eigen::VectorXd(2);
  meas_cam.z(0) = msg->twist.twist.linear.x;

  if(std::abs(msg->twist.twist.angular.z) < 0.008){
    meas_cam.z(1) =  0.0;
  } else {
    meas_cam.z(1) = msg->twist.twist.angular.z;
  }

  meas_cam.R = Eigen::MatrixXd(2,2);
  meas_cam.R.setZero();

  meas_cam.R(0,0) = msg->twist.covariance[0];
  meas_cam.R(1,1) = msg->twist.covariance[35];

  last_time_ = current_time;

  predict(meas_cam.dt);
  updateCameraOdom(meas_cam);

  RCLCPP_INFO(this->get_logger(), "Kamera Odom Hazir | dt: %.4f | Vx: %.2f | Vy: %.2f", 
              meas_cam.dt, meas_cam.z(0), meas_cam.z(1));

}

void GlobalEkfNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  rclcpp::Time current_time = msg->header.stamp;
  
  if (!is_initialized_){
    last_time_ = current_time;
    is_initialized_ = true;
    return ;
  }

  double dt = (current_time - last_time_).seconds();

  if (dt <= 0.0 || dt > 1.0){
    return;
  }

  ImuMeasurement imu_meas;

  imu_meas.dt = dt;

  //ax, wz, 
  imu_meas.z = Eigen::VectorXd(2);
  imu_meas.z(0) = msg->linear_acceleration.x;

  if(std::abs(msg->angular_velocity.z) < 0.005){
    imu_meas.z(1) =  0.0;
  } else {
    imu_meas.z(1) =  msg->angular_velocity.z;
  }


  imu_meas.R = Eigen::MatrixXd(2,2);
  imu_meas.R.setZero();

  imu_meas.R(0,0) = msg->linear_acceleration_covariance[0];
  imu_meas.R(1,1) = msg->angular_velocity_covariance[8];

  last_time_ = current_time;

  predict(imu_meas.dt);
  updateImu(imu_meas);

  RCLCPP_INFO(this->get_logger(), "IMU Hazir | dt: %.4f | ax: %.2f | ay: %.2f", 
            imu_meas.dt, imu_meas.z(0), imu_meas.z(1));
}

void GlobalEkfNode::localOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    local_x_ = msg->pose.pose.position.x;
    local_y_ = msg->pose.pose.position.y;
    
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, local_yaw_);
    
    local_odom_ready_ = true;
}

void GlobalEkfNode::rtkCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    if (!is_initialized_) return;

    rclcpp::Time current_time = msg->header.stamp;
    double dt = (current_time - last_time_).seconds();
    if (dt <= 0.0 || dt > 1.0) return;

    OdomMeasurement gps_meas;
    gps_meas.dt = dt;
    
    tf2::Quaternion q_gps;
    tf2::fromMsg(msg->pose.pose.orientation, q_gps);
    tf2::Matrix3x3 m_gps(q_gps);
    double roll_gps, pitch_gps, yaw_gps;
    m_gps.getRPY(roll_gps, pitch_gps, yaw_gps);
    
    gps_meas.z = Eigen::VectorXd(3);
    gps_meas.z(0) = msg->pose.pose.position.x + test_offset_x_;
    gps_meas.z(1) = msg->pose.pose.position.y;
    gps_meas.z(2) = yaw_gps; 

    gps_meas.R = Eigen::MatrixXd(3, 3);
    gps_meas.R.setZero();
    
    gps_meas.R(0, 0) = std::max(msg->pose.covariance[0], 1.0);  
    gps_meas.R(1, 1) = std::max(msg->pose.covariance[7], 1.0);  
    gps_meas.R(2, 2) = std::max(msg->pose.covariance[35], 1.0);

    last_time_ = current_time;
    
    predict(gps_meas.dt); 
    updateGps(gps_meas); 
}

void GlobalEkfNode::offsetCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    test_offset_x_ = msg->data;
    RCLCPP_WARN(this->get_logger(), "Yapay GPS Kaymasi Uygulandi: %.2f metre", test_offset_x_);
}

