#include "ekf_fusion/ekf_fusion.hpp"

LocalEkfNode::LocalEkfNode() : Node("local_ekf_node"){
   // 8 boyutlu Durum Vektoru (Baalangicta her sey sifir: Konum 0, Hiz 0)
   X_ = Eigen::VectorXd::Zero(8); 

   //Baslangivta 0,0 da, belirsialzik kucuk
   P_ = Eigen::MatrixXd::Identity(8, 8) * 1e-3;

    // 8x8 surec gurultu Matrisi (Q)
    Q_ = Eigen::MatrixXd::Identity(8, 8);
    Q_(0,0) = 0.05;  // X konumu 
    Q_(1,1) = 0.05;  // Y konumu 
    Q_(2,2) = 0.03;  // Theta (Yaw) 
    Q_(3,3) = 0.1;  // Vx 
    Q_(4,4) = 1e-9;  // Vy 
    Q_(5,5) = 0.02;  // Omega 
    Q_(6,6) = 0.01;  // Ax 
    Q_(7,7) = 1e-9;  // Ay 

   wheel_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry> (
    "sensors/wheel_odometry", 10, 
    std::bind(&LocalEkfNode::wheelOdomCallback, this, std::placeholders::_1));

   camera_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry> (
      "sensors/camera_odometry", 10, 
      std::bind(&LocalEkfNode::cameraOdomCallback, this, std::placeholders::_1));

   imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "sensors/imu", 10,
      std::bind(&LocalEkfNode::imuCallback, this, std::placeholders::_1));

   odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("ekf/local_odometry", 10);
   tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

   path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/ekf/path", 10);

   timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&LocalEkfNode::publishLocalOdom, this));

   RCLCPP_INFO(this->get_logger(), "Local EKF Node basariyla baslatildi. Sensorler dinleniyor...");
}
void LocalEkfNode::publishLocalOdom(){

    if(!is_initialized_){
      return;
    }

    rclcpp::Time current_time = this->get_clock()->now();

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom_local";
    odom_msg.child_frame_id = "base_link";


    //Pose veriler
    odom_msg.pose.pose.position.x = X_(0);
    odom_msg.pose.pose.position.y = X_(1);
    odom_msg.pose.pose.position.z = 0.0;

    //Euler to Quaternion donusumu
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, X_(2));
    odom_msg.pose.pose.orientation = tf2::toMsg(q);

    //hiz bilgileri
    odom_msg.twist.twist.linear.x = X_(3);
    odom_msg.twist.twist.linear.y = X_(4);
    odom_msg.twist.twist.angular.z = X_(5);

    // P_ matrisinin kosegenleri: [0]=x, [1]=y, [2]=theta, [3]=vx, [4]=vy, [5]=omega
    odom_msg.pose.covariance.fill(0.0);
    odom_msg.pose.covariance[0] = P_(0, 0);   // X varyansi
    odom_msg.pose.covariance[7] = P_(1, 1);   // Y varyansi
    odom_msg.pose.covariance[35] = P_(2, 2);  // Theta varyansi

    odom_msg.twist.covariance.fill(0.0);
    odom_msg.twist.covariance[0] = P_(3, 3);  // Vx varyansi
    odom_msg.twist.covariance[7] = P_(4, 4);  // Vy varyansi
    odom_msg.twist.covariance[35] = P_(5, 5); // Wz varyansi

    odom_pub_->publish(odom_msg);

    // TF (TRANSFORM) AGACINI YAYINLA
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = current_time;
    t.header.frame_id = "odom_local";
    t.child_frame_id = "base_link";

    // Translation 
    t.transform.translation.x = X_(0);
    t.transform.translation.y = X_(1);
    t.transform.translation.z = 0.0;

    // Rotation
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);

    path_msg_.header.stamp = current_time;
    path_msg_.header.frame_id = "map";

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = current_time;
    pose_stamped.header.frame_id = "map";
    
    pose_stamped.pose.position.x = X_(0);
    pose_stamped.pose.position.y = X_(1);
    pose_stamped.pose.position.z = 0.0;
    
    pose_stamped.pose.orientation = tf2::toMsg(q);

    path_msg_.poses.push_back(pose_stamped);

  
    if (path_msg_.poses.size() > 5000) {
        path_msg_.poses.erase(path_msg_.poses.begin());
    }

    path_pub_->publish(path_msg_);
  
}

void LocalEkfNode::predict(double dt) {

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

void LocalEkfNode::updateWheelOdom(const OdomMeasurement& meas){
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

void LocalEkfNode::updateCameraOdom(const OdomMeasurement& meas){

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

void LocalEkfNode::updateImu(const ImuMeasurement& meas){

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


void LocalEkfNode::wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){

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

void LocalEkfNode::cameraOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
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

void LocalEkfNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
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

