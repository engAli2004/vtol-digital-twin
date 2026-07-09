// ESKF: 9-dim error state [dp, dv, dtheta]. IMU predicts, GPS corrects.
// Ground truth is used ONLY to synthesize noisy sensors + grade the filter.
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <random>
#include <cmath>

using Eigen::Vector3d; using Eigen::Matrix3d; using Eigen::Quaterniond;
using Matrix9d = Eigen::Matrix<double, 9, 9>;
using Vector9d = Eigen::Matrix<double, 9, 1>;

class EskfNode : public rclcpp::Node {
public:
  EskfNode() : Node("eskf_node"), rng_(42), gauss01_(0.0, 1.0) {
    P_ = Matrix9d::Identity();
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      "/vtol_uav/imu", rclcpp::SensorDataQoS(),
      std::bind(&EskfNode::imuCallback, this, std::placeholders::_1));
    sub_truth_ = create_subscription<nav_msgs::msg::Odometry>(
      "/vtol_uav/ground_truth", 10,
      std::bind(&EskfNode::truthCallback, this, std::placeholders::_1));
    pub_est_ = create_publisher<nav_msgs::msg::Odometry>("/vtol_uav/odom_est", 10);
    RCLCPP_INFO(get_logger(),
      "ESKF v1: noisy IMU prediction (100 Hz) + noisy GPS updates (10 Hz, sigma 0.5 m)");
  }

private:
  double gauss(double sigma) { return sigma * gauss01_(rng_); }

  static Matrix3d skew(const Vector3d& v) {
    Matrix3d m;
    m <<     0, -v.z(),  v.y(),
         v.z(),      0, -v.x(),
        -v.y(),  v.x(),      0;
    return m;
  }

  // ===== SENSOR SIM: ground truth -> 10 Hz noisy GPS =====
  void truthCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    p_true_ = Vector3d(msg->pose.pose.position.x,
                       msg->pose.pose.position.y,
                       msg->pose.pose.position.z);
    if (++truth_count_ % 5 != 0) return;   // 50 Hz -> 10 Hz
    Vector3d gps = p_true_ + Vector3d(gauss(0.5), gauss(0.5), gauss(0.5));
    gpsUpdate(gps);
  }

  // ===== PREDICT: noisy IMU at 100 Hz =====
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    rclcpp::Time now(msg->header.stamp);
    if (!initialized_) {
      // Until the first GPS fix, keep grabbing attitude so we start aligned.
      q_ = Quaterniond(msg->orientation.w, msg->orientation.x,
                       msg->orientation.y, msg->orientation.z);
      last_imu_time_ = now;
      return;
    }
    double dt = (now - last_imu_time_).seconds();
    last_imu_time_ = now;
    if (dt <= 0.0 || dt > 0.05) return;

    // Noisy sensors (gyro sigma 0.01 rad/s, accel sigma 0.1 m/s^2)
    Vector3d w(msg->angular_velocity.x + gauss(0.01),
               msg->angular_velocity.y + gauss(0.01),
               msg->angular_velocity.z + gauss(0.01));
    Vector3d a(msg->linear_acceleration.x + gauss(0.1),
               msg->linear_acceleration.y + gauss(0.1),
               msg->linear_acceleration.z + gauss(0.1));

    // ----- Nominal state propagation -----
    Matrix3d R = q_.toRotationMatrix();
    Vector3d g(0.0, 0.0, -9.81);
    Vector3d acc = R * a + g;               // world-frame acceleration
    p_ += v_ * dt + 0.5 * acc * dt * dt;
    v_ += acc * dt;
    Vector3d dth = w * dt;
    if (dth.norm() > 1e-9) {
      Quaterniond dq(Eigen::AngleAxisd(dth.norm(), dth.normalized()));
      q_ = (q_ * dq).normalized();
    }

    // ----- Covariance propagation (Jacobian F of the error state) -----
    Matrix9d F = Matrix9d::Identity();
    F.block<3,3>(0,3) =  Matrix3d::Identity() * dt;   // dp <- dv
    F.block<3,3>(3,6) = -R * skew(a) * dt;            // dv <- dtheta
    F.block<3,3>(6,6) =  Matrix3d::Identity() - skew(w) * dt;
    Matrix9d Q = Matrix9d::Zero();
    Q.block<3,3>(3,3) = Matrix3d::Identity() * (0.1*0.1) * dt*dt;
    Q.block<3,3>(6,6) = Matrix3d::Identity() * (0.01*0.01) * dt*dt;
    P_ = F * P_ * F.transpose() + Q;

    publishEstimate(msg->header.stamp);
  }

  // ===== CORRECT: GPS position, H = [I 0 0] =====
  void gpsUpdate(const Vector3d& z) {
    if (!initialized_) {
      p_ = z; v_.setZero(); initialized_ = true;
      RCLCPP_INFO(get_logger(), "ESKF initialized at first GPS fix.");
      return;
    }
    Eigen::Matrix<double,3,9> H = Eigen::Matrix<double,3,9>::Zero();
    H.block<3,3>(0,0) = Matrix3d::Identity();
    Matrix3d Rn = Matrix3d::Identity() * (0.5*0.5);
    Matrix3d S = H * P_ * H.transpose() + Rn;
    Eigen::Matrix<double,9,3> K = P_ * H.transpose() * S.inverse();
    Vector9d dx = K * (z - p_);

    // Inject error state into nominal state
    p_ += dx.segment<3>(0);
    v_ += dx.segment<3>(3);
    Vector3d dth = dx.segment<3>(6);
    if (dth.norm() > 1e-9) {
      Quaterniond dq(Eigen::AngleAxisd(dth.norm(), dth.normalized()));
      q_ = (q_ * dq).normalized();
    }
    P_ = (Matrix9d::Identity() - K * H) * P_;

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "ESKF: pos error %.2f m (raw GPS sigma: 0.50 m)", (p_ - p_true_).norm());
  }

  void publishEstimate(const rclcpp::Time& stamp) {
    nav_msgs::msg::Odometry est;
    est.header.stamp = stamp;
    est.header.frame_id = "world";
    est.child_frame_id = "dummy_root";
    est.pose.pose.position.x = p_.x();
    est.pose.pose.position.y = p_.y();
    est.pose.pose.position.z = p_.z();
    est.pose.pose.orientation.x = q_.x();
    est.pose.pose.orientation.y = q_.y();
    est.pose.pose.orientation.z = q_.z();
    est.pose.pose.orientation.w = q_.w();
    est.twist.twist.linear.x = v_.x();
    est.twist.twist.linear.y = v_.y();
    est.twist.twist.linear.z = v_.z();
    pub_est_->publish(est);
  }

  // Nominal state
  Vector3d p_{0,0,0}, v_{0,0,0};
  Quaterniond q_{1,0,0,0};
  Matrix9d P_;
  bool initialized_ = false;
  // Bookkeeping
  Vector3d p_true_{0,0,0};
  int truth_count_ = 0;
  rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
  std::mt19937 rng_;
  std::normal_distribution<double> gauss01_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_truth_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_est_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EskfNode>());
  rclcpp::shutdown();
  return 0;
}
