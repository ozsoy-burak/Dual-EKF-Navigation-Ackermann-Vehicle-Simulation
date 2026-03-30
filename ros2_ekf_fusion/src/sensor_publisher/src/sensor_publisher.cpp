#include "sensor_publisher/sensor_publisher.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>

namespace sensor_publisher
{

using namespace std::chrono_literals;

SensorPublisher::SensorPublisher()
: Node("sensor_publisher"),
  rng_(std::random_device{}())
{
  // ── Declare parameters (Diğer sensörler aynı) ───────────────────────────
  declare_parameter("wheel_odom.rate_hz",          50.0);
  declare_parameter("wheel_odom.linear_x_std",     0.03);   
  declare_parameter("wheel_odom.angular_z_std",    0.08);   
  declare_parameter("wheel_odom.linear_bias",      0.005);  
  declare_parameter("wheel_odom.drift_rate",       0.001);  

  declare_parameter("imu.rate_hz",                 200.0);
  declare_parameter("imu.accel_std",               0.05);   
  declare_parameter("imu.gyro_std",                0.003);  
  declare_parameter("imu.accel_bias",              0.02);   
  declare_parameter("imu.gyro_bias",               0.001);  
  declare_parameter("imu.accel_drift_rate",        0.0001);
  declare_parameter("imu.gyro_drift_rate",         0.00005);

  declare_parameter("camera_odom.rate_hz",         30.0);
  declare_parameter("camera_odom.linear_std",      0.05);   
  declare_parameter("camera_odom.angular_std",     0.02);   
  declare_parameter("camera_odom.scale_error",     0.02);   

  // ── YENİ GPS PARAMETRELERİ ──────────────────────────────────────────────
  declare_parameter("gps.origin_lat",              39.9334); 
  declare_parameter("gps.origin_lon",              32.8597); 

  declare_parameter("gps.standard.rate_hz",        5.0);
  declare_parameter("gps.standard.position_std",   1.5);    
  declare_parameter("gps.standard.multipath_std",  5.0);    
  declare_parameter("gps.standard.multipath_prob", 0.02);   

  declare_parameter("gps.rtk.rate_hz",             10.0);
  declare_parameter("gps.rtk.fix_std",             0.02);   
  declare_parameter("gps.rtk.float_std",           0.40);   
  declare_parameter("gps.rtk.heading_std",         0.01);   
  declare_parameter("gps.rtk.float_prob",          0.05);   

  // Read parameters
  wheel_rate_hz_       = get_parameter("wheel_odom.rate_hz").as_double();
  wheel_linear_x_std_  = get_parameter("wheel_odom.linear_x_std").as_double();
  wheel_angular_z_std_ = get_parameter("wheel_odom.angular_z_std").as_double();
  wheel_linear_bias_   = get_parameter("wheel_odom.linear_bias").as_double();
  wheel_drift_rate_    = get_parameter("wheel_odom.drift_rate").as_double();

  imu_rate_hz_         = get_parameter("imu.rate_hz").as_double();
  imu_accel_std_       = get_parameter("imu.accel_std").as_double();
  imu_gyro_std_        = get_parameter("imu.gyro_std").as_double();
  imu_accel_bias_      = get_parameter("imu.accel_bias").as_double();
  imu_gyro_bias_       = get_parameter("imu.gyro_bias").as_double();
  imu_accel_drift_     = get_parameter("imu.accel_drift_rate").as_double();
  imu_gyro_drift_      = get_parameter("imu.gyro_drift_rate").as_double();

  cam_rate_hz_         = get_parameter("camera_odom.rate_hz").as_double();
  cam_linear_std_      = get_parameter("camera_odom.linear_std").as_double();
  cam_angular_std_     = get_parameter("camera_odom.angular_std").as_double();
  cam_scale_error_     = get_parameter("camera_odom.scale_error").as_double();

  gps_origin_lat_      = get_parameter("gps.origin_lat").as_double();
  gps_origin_lon_      = get_parameter("gps.origin_lon").as_double();

  std_gps_rate_hz_         = get_parameter("gps.standard.rate_hz").as_double();
  std_gps_pos_std_         = get_parameter("gps.standard.position_std").as_double();
  std_gps_multipath_std_   = get_parameter("gps.standard.multipath_std").as_double();
  std_gps_multipath_prob_  = get_parameter("gps.standard.multipath_prob").as_double();

  rtk_rate_hz_             = get_parameter("gps.rtk.rate_hz").as_double();
  rtk_fix_std_             = get_parameter("gps.rtk.fix_std").as_double();
  rtk_float_std_           = get_parameter("gps.rtk.float_std").as_double();
  rtk_heading_std_         = get_parameter("gps.rtk.heading_std").as_double();
  rtk_float_prob_          = get_parameter("gps.rtk.float_prob").as_double();

  // ── Shared state ─────────────────────────────────────────────────────────
  shared_state_ = std::make_shared<SharedState>();

  // ── Publishers ───────────────────────────────────────────────────────────
  wheel_odom_pub_  = create_publisher<nav_msgs::msg::Odometry>("sensors/wheel_odometry", 10);
  imu_pub_         = create_publisher<sensor_msgs::msg::Imu>("sensors/imu", 10);
  camera_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("sensors/camera_odometry", 10);
  
  // YENİ: Ayrı GPS Publisher'ları
  std_gps_pub_       = create_publisher<sensor_msgs::msg::NavSatFix>("sensors/gps_standard/fix", 10);
  std_gps_pose_pub_  = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("sensors/gps_standard/pose", 10);
  
  rtk_gps_pub_       = create_publisher<sensor_msgs::msg::NavSatFix>("sensors/gps_rtk/fix", 10);
  rtk_gps_pose_pub_  = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("sensors/gps_rtk/pose", 10);

  // ── Subscriber ───────────────────────────────────────────────────────────
  state_sub_ = create_subscription<vehicle_interfaces::msg::VehicleState>(
    "vehicle/true_state", 10,
    std::bind(&SensorPublisher::stateCallback, this, std::placeholders::_1));

  // ── Launch sensor threads ─────────────────────────────────────────────────
  wheel_odom_thread_  = std::thread(&SensorPublisher::wheelOdomLoop,  this);
  imu_thread_         = std::thread(&SensorPublisher::imuLoop,         this);
  camera_odom_thread_ = std::thread(&SensorPublisher::cameraOdomLoop, this);
  std_gps_thread_     = std::thread(&SensorPublisher::stdGpsLoop,      this);
  rtk_gps_thread_     = std::thread(&SensorPublisher::rtkGpsLoop,      this);

  RCLCPP_INFO(get_logger(), "SensorPublisher started.");
}

SensorPublisher::~SensorPublisher()
{
  running_ = false;
  if (wheel_odom_thread_.joinable())  wheel_odom_thread_.join();
  if (imu_thread_.joinable())         imu_thread_.join();
  if (camera_odom_thread_.joinable()) camera_odom_thread_.join();
  if (std_gps_thread_.joinable())     std_gps_thread_.join();
  if (rtk_gps_thread_.joinable())     rtk_gps_thread_.join();
}

void SensorPublisher::stateCallback(const vehicle_interfaces::msg::VehicleState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(shared_state_->mtx);
  shared_state_->state = *msg;
  shared_state_->valid = true;
}

double SensorPublisher::sampleGaussian(double mean, double std_dev)
{
  std::normal_distribution<double> dist(mean, std_dev);
  return dist(rng_);
}

// ── Thread 1: Wheel Odometry ──────────────────────────────────────────────────
//
// Gerçekçi model:
//
//  ÖLÇÜM PRENSİBİ:
//    - Her tekerlekte Hall-effect encoder var → tekerlek dönüşünden Vx ölçülür
//    - Direksiyon kolonu encoder'ı δ (steering angle) ölçer
//    - omega = Vx_enc * tan(δ_enc) / L   ← hesaplanır, doğrudan ölçülmez
//
//  GÜRÜLTÜ MODELİ:
//    - Durukta: encoder pulse yok → ölçüm 0, gürültü 0 (sadece quantization)
//    - Hareket halinde: gürültü hıza orantılı (pulse resolution etkisi)
//    - Yüksek ivmede: tekerlek slip → Vx_enc > Vx_gerçek
//    - Frende: reverse slip → Vx_enc < Vx_gerçek
//    - Drift: uzun süreli çalışmada ısınma/yağlama değişimi → çok yavaş
//    - Y ekseni: NON-HOLONOMİK → her zaman 0, gürültü de yok
//
void SensorPublisher::wheelOdomLoop()
{
  const double period_us  = 1e6 / wheel_rate_hz_;
  const double dt         = 1.0 / wheel_rate_hz_;
  const double wheelbase  = 2.7;   // metre - Ackermann L

  // Entegrasyon değişkenleri (dead-reckoning)
  double int_x     = 0.0;
  double int_y     = 0.0;
  double int_theta = 0.0;

  // Encoder quantization: tipik 4096 pulse/tur, tekerlek çevresi ~2m
  // → çözünürlük ~0.49 mm, hız çözünürlüğü @50Hz = 0.024 m/s
  const double encoder_resolution = (2.0 * M_PI * 0.35) / 4096.0; // ~0.537mm/pulse
  const double vel_resolution     = encoder_resolution * wheel_rate_hz_; // @50Hz

  // Direksiyon encoder çözünürlüğü: tipik 12-bit → 4096 adım, ±0.52 rad range
  const double steer_resolution = (2.0 * 0.52) / 4096.0; // ~0.254 mrad/step

  // Slip durum değişkenleri
  double slip_error    = 0.0;   // birikimli slip offset (yavaşça sıfırlanır)
  double prev_speed    = 0.0;   // önceki hız (ivme hesabı için)

  // Uzun vadeli drift (ısınma etkisi) - çok yavaş rastgele yürüyüş
  double thermal_drift = 0.0;

  while (running_) {
    auto t_start = std::chrono::steady_clock::now();

    vehicle_interfaces::msg::VehicleState state;
    bool valid;
    {
      std::lock_guard<std::mutex> lk(shared_state_->mtx);
      state = shared_state_->state;
      valid = shared_state_->valid;
    }

    if (valid) {
      const double true_speed   = state.speed;
      const double true_steering = state.steering_angle;  // δ (rad)
      const double true_omega   = state.omega;

      // ── 1. DURUŞ KONTROLÜ ─────────────────────────────────────────────────
      // Araç duruyorsa encoder pulse gelmiyor → her şey 0
      // Eşik: ~1 encoder pulse/dt = vel_resolution
      const bool   vehicle_moving = (std::abs(true_speed) > vel_resolution * 0.5);

      // ── 2. TEKERLEK SLIP MODELİ ───────────────────────────────────────────
      // Boylamsal ivme
      const double accel = (true_speed - prev_speed) / dt;
      prev_speed = true_speed;

      // Slip oranı: ivme ne kadar büyükse tekerlek o kadar kayar
      // Drive slip (pozitif ivme): Vx_enc > Vx_gerçek
      // Brake slip (negatif ivme): Vx_enc < Vx_gerçek
      // Formül: slip_ratio = accel / (g * mu)  — mu ~0.8 for asphalt
      const double g    = 9.81;
      const double mu   = 0.85;  // asphalt friction coefficient
      double slip_ratio = accel / (g * mu);
      // Sınırla: maksimum %15 slip
      slip_ratio = std::clamp(slip_ratio, -0.15, 0.15);

      // Slip hız etkisi: delta_v = slip_ratio * true_speed
      // Yavaşça bir önceki değere yaklaşarak birikir, durukta sıfırlanır
      if (vehicle_moving) {
        double target_slip = slip_ratio * std::abs(true_speed);
        slip_error += (target_slip - slip_error) * (1.0 - std::exp(-dt / 0.3));
      } else {
        // Durukta slip 0'a git (exponential decay, 0.1s time constant)
        slip_error *= std::exp(-dt / 0.1);
      }

      // ── 3. ENCODER ÖLÇÜM SİMÜLASYONU ─────────────────────────────────────
      // Tekerlek encoder hız ölçümü
      double enc_vx = 0.0;
      double enc_delta = 0.0;  // encoder'dan okunan direksiyon açısı

      if (vehicle_moving) {
        // Tekerlek encoder: gerçek hız + slip + quantization + hıza-orantılı gürültü
        // Gürültü hıza orantılı: hızlanınca pulse timing error artar
        const double speed_dependent_noise = wheel_linear_x_std_
          * (std::abs(true_speed) / 10.0 + 0.1);  // min %10, hızla artar

        // Thermal drift: araç çalıştıkça çok yavaş değişir
        thermal_drift += sampleGaussian(0.0,
          wheel_drift_rate_ * std::sqrt(dt) * std::abs(true_speed));

        enc_vx = true_speed
          + slip_error
          + thermal_drift * true_speed  // oransal etki
          + sampleGaussian(0.0, speed_dependent_noise);

        // Quantization: en yakın encoder adımına yuvarla
        enc_vx = std::round(enc_vx / vel_resolution) * vel_resolution;

        // Direksiyon encoder: açısal quantization + küçük mekanik oynak
        const double steer_noise = steer_resolution * 250.0;  // 2 LSB belirsizlik
        enc_delta = true_steering
          + std::round(sampleGaussian(0.0, steer_noise) / steer_resolution)
            * steer_resolution;

      } else {
        // Araç duruyorsa: sıfır (quantization sınırının altında)
        // Sadece çok nadir elektriksel gürültü (0.1% ihtimal)
        std::uniform_real_distribution<double> u(0.0, 1.0);
        if (u(rng_) < 0.001) {
          // Tek bir yanlış pulse (çok nadir)
          enc_vx    = vel_resolution * (u(rng_) > 0.5 ? 1.0 : -1.0);
          enc_delta = 0.0;
        } else {
          enc_vx    = 0.0;
          enc_delta = true_steering;  // direksiyon statik, encoder sabit okur
        }
      }

      // ── 4. OMEGA HESAPLA (Ackermann kinematiği) ───────────────────────────
      // omega = Vx * tan(δ) / L
      // Gürültü: hem Vx hem δ encoder gürültüsünden gelir
      // Doğrudan gürültülü omega değil, FİZİKSEL OLARAK HESAPLANIR
      double enc_omega = 0.0;
      if (vehicle_moving || std::abs(enc_delta) > steer_resolution) {
        enc_omega = (enc_vx * std::tan(enc_delta)) / wheelbase;
      }
      // Not: omega gürültüsü δ ve Vx encoder hatalarından otomatik oluşur
      // Ek gürültü YOK — gerçekçi model bu

      // ── 5. ENTEGRASYON ────────────────────────────────────────────────────
      int_theta += enc_omega * dt;
      while (int_theta >  M_PI) int_theta -= 2.0 * M_PI;
      while (int_theta < -M_PI) int_theta += 2.0 * M_PI;

      // Non-holonomic: sadece heading yönünde hareket
      int_x += enc_vx * std::cos(int_theta) * dt;
      int_y += enc_vx * std::sin(int_theta) * dt;

      // ── 6. MESAJ OLUŞTUR ───────────────────────────────────────────────────
      nav_msgs::msg::Odometry msg;
      msg.header.stamp    = rclcpp::Clock().now();
      msg.header.frame_id = "odom";
      msg.child_frame_id  = "base_link";

      msg.pose.pose.position.x = int_x;
      msg.pose.pose.position.y = int_y;
      msg.pose.pose.position.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0, 0, int_theta);
      msg.pose.pose.orientation = tf2::toMsg(q);

      // TWIST — body frame
      msg.twist.twist.linear.x  = enc_vx;
      msg.twist.twist.linear.y  = 0.0;   // NON-HOLONOMİK: kesinlikle 0, gürültü de yok
      msg.twist.twist.linear.z  = 0.0;
      msg.twist.twist.angular.z = enc_omega;

      // ── 7. KOVARYANS ──────────────────────────────────────────────────────
      // Durukta çok küçük, hareket halinde hıza orantılı
      msg.twist.covariance.fill(0.0);
      if (vehicle_moving) {
        // Vx varyansı: hıza orantılı (daha hızlı → daha çok timing error)
        double vx_var = std::pow(wheel_linear_x_std_
          * (std::abs(enc_vx) / 5.0 + 0.2), 2.0);
        // Omega varyansı: δ ve Vx hatalarından propagasyon
        // sigma_omega^2 ≈ (tan(δ)/L)^2 * sigma_vx^2 + (Vx/L/cos^2(δ))^2 * sigma_delta^2
        double tan_d  = std::tan(enc_delta);
        double cos2_d = std::pow(std::cos(enc_delta), 2.0);
        double d_omega_dvx    = tan_d / wheelbase;
        double d_omega_ddelta = enc_vx / (wheelbase * cos2_d);
        double omega_var = d_omega_dvx * d_omega_dvx * vx_var
          + d_omega_ddelta * d_omega_ddelta * steer_resolution * steer_resolution;

        msg.twist.covariance[0]  = 0.001;
        msg.twist.covariance[7]  = 1e-10;  // Vy: 0, belirsizlik çok küçük
        msg.twist.covariance[35] = (enc_vx * 1.25) * 0.01;
      } else {
        // Durukta: sadece quantization seviyesinde
        msg.twist.covariance[0]  = vel_resolution * vel_resolution;
        msg.twist.covariance[7]  = 1e-12;
        msg.twist.covariance[35] = 0.02;
      }

      // Pose kovaryansı: dead-reckoning birikim
      msg.pose.covariance.fill(0.0);
      double drift_pos_std = 0.01 + std::abs(int_x) * 0.002 + std::abs(int_y) * 0.002;
      msg.pose.covariance[0]  = drift_pos_std * drift_pos_std;
      msg.pose.covariance[7]  = drift_pos_std * drift_pos_std;
      msg.pose.covariance[35] = 1e-4 + std::abs(int_theta) * 5e-5;

      wheel_odom_pub_->publish(msg);
    }

    auto elapsed = std::chrono::steady_clock::now() - t_start;
    auto sleep_us = static_cast<long>(period_us)
      - std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
    if (sleep_us > 0)
      std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
  }
}


// ── Thread 2: IMU ─────────────────────────────────────────────────────────────
void SensorPublisher::imuLoop()
{
  double period_us = 1e6 / imu_rate_hz_;
  double dt = 1.0 / imu_rate_hz_;

  // Previous speed for acceleration estimation
  double prev_speed = 0.0;

  while (running_) {
    auto t_start = std::chrono::steady_clock::now();

    vehicle_interfaces::msg::VehicleState state;
    bool valid;
    {
      std::lock_guard<std::mutex> lk(shared_state_->mtx);
      state = shared_state_->state;
      valid = shared_state_->valid;
    }

    if (valid) {
      // Update drift (random walk)
      imu_accel_drift_current_ += sampleGaussian(0.0, imu_accel_drift_ * std::sqrt(dt));
      imu_gyro_drift_current_  += sampleGaussian(0.0, imu_gyro_drift_  * std::sqrt(dt));

      // True longitudinal acceleration (finite difference of speed)
      double true_accel_x = (state.speed - prev_speed) / dt;
      prev_speed = state.speed;

      // Centripetal acceleration in body frame (ay = v * omega)
      double true_accel_y = state.speed * state.omega;

      // Noisy IMU measurements in body frame
      double noisy_ax = true_accel_x
        + imu_accel_bias_ + imu_accel_drift_current_
        + sampleGaussian(0.0, imu_accel_std_);
      double noisy_ay = true_accel_y
        + sampleGaussian(0.0, imu_accel_std_);
      double noisy_gz = state.omega
        + imu_gyro_bias_ + imu_gyro_drift_current_
        + sampleGaussian(0.0, imu_gyro_std_);

      sensor_msgs::msg::Imu msg;
      msg.header.stamp    = rclcpp::Clock().now();
      msg.header.frame_id = "base_link";

      // Orientation (from true heading - IMU with magnetometer)
      tf2::Quaternion q;
      q.setRPY(0, 0, state.theta + sampleGaussian(0.0, 0.01));
      msg.orientation = tf2::toMsg(q);
      msg.orientation_covariance[8] = 0.0001; // yaw variance

      // Angular velocity
      msg.angular_velocity.x = sampleGaussian(0.0, imu_gyro_std_);
      msg.angular_velocity.y = sampleGaussian(0.0, imu_gyro_std_);
      msg.angular_velocity.z = noisy_gz;
      double gv = imu_gyro_std_ * imu_gyro_std_;
      msg.angular_velocity_covariance[0] = gv;
      msg.angular_velocity_covariance[4] = gv;
      msg.angular_velocity_covariance[8] = gv;

      // Linear acceleration (body frame)
      msg.linear_acceleration.x = noisy_ax;
      msg.linear_acceleration.y = noisy_ay;
      msg.linear_acceleration.z = 9.81 + sampleGaussian(0.0, imu_accel_std_); // gravity
      double av = imu_accel_std_ * imu_accel_std_;
      msg.linear_acceleration_covariance[0] = 0.05;
      msg.linear_acceleration_covariance[4] = 0.1;
      msg.linear_acceleration_covariance[8] = 0.01;

      imu_pub_->publish(msg);
    }

    auto elapsed = std::chrono::steady_clock::now() - t_start;
    auto sleep_us = static_cast<long>(period_us) -
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
    if (sleep_us > 0) {
      std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
    }
  }
}

// ── Thread 3: Camera Odometry (Visual Odometry) ───────────────────────────────
void SensorPublisher::cameraOdomLoop()
{
  double period_us = 1e6 / cam_rate_hz_;
  double dt = 1.0 / cam_rate_hz_;
  double integrated_x = 0.0;
  double integrated_y = 0.0;
  double integrated_theta = 0.0;
  int frame_count = 0;

  while (running_) {
    auto t_start = std::chrono::steady_clock::now();

    vehicle_interfaces::msg::VehicleState state;
    bool valid;
    {
      std::lock_guard<std::mutex> lk(shared_state_->mtx);
      state = shared_state_->state;
      valid = shared_state_->valid;
    }

    if (valid) {
      frame_count++;

      // Camera loses tracking at high angular rates (simulate tracking failure)
      bool tracking_ok = std::abs(state.omega) < 0.8;

      double noisy_linear  = 0.0;
      double noisy_angular = 0.0;

      if (tracking_ok) {
        // Scale factor error + Gaussian noise (proportional to speed)
        double scale_noise = 1.0 + cam_scale_error_ * sampleGaussian(0.0, 0.5);
        noisy_linear  = state.speed * scale_noise
          + sampleGaussian(0.0, cam_linear_std_ * (1.0 + state.speed * 0.02));
        noisy_angular = state.omega
          + sampleGaussian(0.0, cam_angular_std_);
      } else {
        // Tracking failure: large uncertainty, nearly zero output
        noisy_linear  = sampleGaussian(0.0, cam_linear_std_ * 5.0);
        noisy_angular = sampleGaussian(0.0, cam_angular_std_ * 5.0);
      }

      integrated_theta += noisy_angular * dt;
      integrated_x     += noisy_linear * std::cos(integrated_theta) * dt;
      integrated_y     += noisy_linear * std::sin(integrated_theta) * dt;

      nav_msgs::msg::Odometry msg;
      msg.header.stamp    = rclcpp::Clock().now();
      msg.header.frame_id = "camera_odom";
      msg.child_frame_id  = "base_link";

      msg.pose.pose.position.x = integrated_x;
      msg.pose.pose.position.y = integrated_y;
      tf2::Quaternion q;
      q.setRPY(0, 0, integrated_theta);
      msg.pose.pose.orientation = tf2::toMsg(q);

      msg.twist.twist.linear.x  = noisy_linear;
      msg.twist.twist.angular.z = noisy_angular;

      double cov_scale = tracking_ok ? 1.0 : 25.0;
      double lv_var = cam_linear_std_ * cam_linear_std_ * cov_scale;
      double av_var = cam_angular_std_ * cam_angular_std_ * cov_scale;
      msg.twist.covariance[0]  = 0.015;
      if (noisy_linear > 3.0){
          msg.twist.covariance[35] = 1000.0;
      } else {
        msg.twist.covariance[35] = (noisy_linear) * 0.03;
      }

      camera_odom_pub_->publish(msg);
    }

    auto elapsed = std::chrono::steady_clock::now() - t_start;
    auto sleep_us = static_cast<long>(period_us) -
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
    if (sleep_us > 0) {
      std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
    }
  }
}
void SensorPublisher::stdGpsLoop()
{
  double period_us = 1e6 / std_gps_rate_hz_;
  std::uniform_real_distribution<double> uniform(0.0, 1.0);

  while (running_) {
    auto t_start = std::chrono::steady_clock::now();

    vehicle_interfaces::msg::VehicleState state;
    bool valid;
    {
      std::lock_guard<std::mutex> lk(shared_state_->mtx);
      state = shared_state_->state;
      valid = shared_state_->valid;
    }

    if (valid) {
      // Multipath simülasyonu (ağaç altı vs sinyal yansıması)
      bool multipath = (uniform(rng_) < std_gps_multipath_prob_);
      double noise_std = multipath ? std_gps_multipath_std_ : std_gps_pos_std_;

      double noisy_x = state.x + sampleGaussian(0.0, noise_std);
      double noisy_y = state.y + sampleGaussian(0.0, noise_std);

      auto [lat, lon] = xyToLatLon(noisy_x, noisy_y);

      // NavSatFix mesajı (Global)
      sensor_msgs::msg::NavSatFix gps_msg;
      gps_msg.header.stamp    = rclcpp::Clock().now();
      gps_msg.header.frame_id = "gps_standard_link";
      gps_msg.status.status   = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      gps_msg.status.service  = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
      gps_msg.latitude        = lat;
      gps_msg.longitude       = lon;
      gps_msg.altitude        = 900.0 + sampleGaussian(0.0, 3.0); 
      
      double pos_var = noise_std * noise_std;
      gps_msg.position_covariance = {
        pos_var, 0, 0,
        0, pos_var, 0,
        0, 0, pos_var * 4.0
      };
      gps_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      std_gps_pub_->publish(gps_msg);

      // EKF için Local Pose (X, Y) - Yönelim (Heading) YOK.
      geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
      pose_msg.header.stamp    = gps_msg.header.stamp;
      pose_msg.header.frame_id = "map";
      pose_msg.pose.pose.position.x = noisy_x;
      pose_msg.pose.pose.position.y = noisy_y;
      
      // Covariance array'i 36 elemanlıdır, önce hepsini sıfırlamak güvenlidir.
      pose_msg.pose.covariance.fill(0.0);
      pose_msg.pose.covariance[0] = pos_var; // X varyansı
      pose_msg.pose.covariance[7] = pos_var; // Y varyansı
      // Heading (yaw) ölçümü güvenilmez olduğu için EKF'ye verilmez.
      
      std_gps_pose_pub_->publish(pose_msg);
    }

    auto elapsed = std::chrono::steady_clock::now() - t_start;
    auto sleep_us = static_cast<long>(period_us) -
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
    if (sleep_us > 0) std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
  }
}

// ── Thread 5: RTK GPS (Dual Antenna, Santimetre Hassasiyeti, Güvenilir Heading) ─────────
void SensorPublisher::rtkGpsLoop()
{
  double period_us = 1e6 / rtk_rate_hz_;
  std::uniform_real_distribution<double> uniform(0.0, 1.0);

  while (running_) {
    auto t_start = std::chrono::steady_clock::now();

    vehicle_interfaces::msg::VehicleState state;
    bool valid;
    {
      std::lock_guard<std::mutex> lk(shared_state_->mtx);
      state = shared_state_->state;
      valid = shared_state_->valid;
    }

    if (valid) {
      // Durum Makinesi: RTK Fix mi Float mu?
      bool is_float = (uniform(rng_) < rtk_float_prob_);
      
      double pos_std_dev = is_float ? rtk_float_std_ : rtk_fix_std_;
      double hdg_std_dev = is_float ? (rtk_heading_std_ * 3.0) : rtk_heading_std_; // Float'ta heading de bozulur

      double noisy_x = state.x + sampleGaussian(0.0, pos_std_dev);
      double noisy_y = state.y + sampleGaussian(0.0, pos_std_dev);
      
      // Dual anten heading ölçümü simülasyonu
      double noisy_yaw = state.theta + sampleGaussian(0.0, hdg_std_dev);

      auto [lat, lon] = xyToLatLon(noisy_x, noisy_y);

      // NavSatFix mesajı
      sensor_msgs::msg::NavSatFix gps_msg;
      gps_msg.header.stamp    = rclcpp::Clock().now();
      gps_msg.header.frame_id = "gps_rtk_link";
      // Gerçekte GBAS_FIX genelde RTK_FIX olarak yorumlanır
      gps_msg.status.status   = is_float 
        ? sensor_msgs::msg::NavSatStatus::STATUS_FIX        // Float
        : sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;  // RTK Fix
      gps_msg.status.service  = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
      gps_msg.latitude        = lat;
      gps_msg.longitude       = lon;
      gps_msg.altitude        = 900.0 + sampleGaussian(0.0, 0.5); // RTK yükseklikte de iyidir
      
      double pos_var = pos_std_dev * pos_std_dev;
      gps_msg.position_covariance = {
        pos_var, 0, 0,
        0, pos_var, 0,
        0, 0, pos_var * 2.0
      };
      gps_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      rtk_gps_pub_->publish(gps_msg);

      // EKF için Local Pose (X, Y ve YAW) - Dual Antenna RTK
      geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
      pose_msg.header.stamp    = gps_msg.header.stamp;
      pose_msg.header.frame_id = "map";
      pose_msg.pose.pose.position.x = noisy_x;
      pose_msg.pose.pose.position.y = noisy_y;
      
      tf2::Quaternion q;
      q.setRPY(0, 0, noisy_yaw);
      pose_msg.pose.pose.orientation = tf2::toMsg(q);

      pose_msg.pose.covariance.fill(0.0);
      pose_msg.pose.covariance[0] = pos_var;                 // X
      pose_msg.pose.covariance[7] = pos_var;                 // Y
      pose_msg.pose.covariance[35] = hdg_std_dev * hdg_std_dev; // YAW
      
      rtk_gps_pose_pub_->publish(pose_msg);
    }

    auto elapsed = std::chrono::steady_clock::now() - t_start;
    auto sleep_us = static_cast<long>(period_us) -
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
    if (sleep_us > 0) std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
  }
}

std::pair<double,double> SensorPublisher::xyToLatLon(double x, double y)
{
  constexpr double DEG_PER_M_LAT = 1.0 / 111320.0;
  double deg_per_m_lon = 1.0 / (111320.0 * std::cos(gps_origin_lat_ * M_PI / 180.0));
  double lat = gps_origin_lat_ + y * DEG_PER_M_LAT;
  double lon = gps_origin_lon_ + x * deg_per_m_lon;
  return {lat, lon};
}

}  // namespace sensor_publisher