#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <cmath>
#include <algorithm>
#include <vector>

class FlightController : public rclcpp::Node {
public:
  FlightController() : Node("flight_controller") {
    pub_rpm_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/rotor_velocity_controller/commands", 10);
    pub_debug_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/pid_debug", 10);
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/vtol_uav/imu", rclcpp::SensorDataQoS(),
      std::bind(&FlightController::imuCallback, this, std::placeholders::_1));
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/vtol_uav/ground_truth", 10,
      std::bind(&FlightController::odomCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(),
      "Flight Controller v8: waypoint mission. %zu waypoints loaded.",
      mission_.size());
  }

private:
  // ===== OUTER LOOP (50 Hz): waypoints + position + altitude =====
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double x  = msg->pose.pose.position.x;
    double y  = msg->pose.pose.position.y;
    double z  = msg->pose.pose.position.z;
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    double vz = msg->twist.twist.linear.z;

    // ----- Waypoint sequencer -----
    // Arrived = within 0.5 m of target AND slow (< 0.3 m/s).
    // The speed check forces a real arrival, not a fly-by at full speed.
    double dist = std::sqrt(std::pow(target_x_-x,2) + std::pow(target_y_-y,2)
                          + std::pow(target_z_-z,2));
    double speed = std::sqrt(vx*vx + vy*vy + vz*vz);
    if (dist < 0.8 && speed < 0.5 && wp_index_ + 1 < mission_.size()) {
      wp_index_++;
      target_x_ = mission_[wp_index_].x;
      target_y_ = mission_[wp_index_].y;
      target_z_ = mission_[wp_index_].z;
      RCLCPP_INFO(this->get_logger(),
        "Waypoint %zu captured -> heading to WP%zu (%.1f, %.1f, %.1f)",
        wp_index_-1, wp_index_, target_x_, target_y_, target_z_);
    }

    double qx = msg->pose.pose.orientation.x, qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z, qw = msg->pose.pose.orientation.w;

    // Yaw for telemetry + yaw controller (NOT used to rotate position errors).
    yaw_rad_ = std::atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz));
    yaw_deg_ = yaw_rad_ * 57.3;

    // World-frame PD position control.
    // kp pulls toward target; kd is artificial drag (Gazebo has none).
    double tilt_max = 0.087;  // 5 degrees
    setpoint_pitch_ =  std::clamp( kp_pos_*(target_x_-x) - kd_pos_*vx,
                                   -tilt_max, tilt_max);
    setpoint_roll_  =  std::clamp(-kp_pos_*(target_y_-y) + kd_pos_*vy,
                                   -tilt_max, tilt_max);

    // Altitude PD -> base motor speed.
    double thrust = mass_*9.8 + kp_z_*(target_z_-z) - kd_z_*vz;
    thrust = std::clamp(thrust, 10.0, 45.0);
    base_omega_ = std::sqrt(thrust / (4.0 * ct_));

    x_ = x; y_ = y; z_ = z;
  }

  // ===== INNER LOOP (100 Hz): attitude PID + yaw PD =====
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    double qx = msg->orientation.x, qy = msg->orientation.y;
    double qz = msg->orientation.z, qw = msg->orientation.w;
    double roll  = std::atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy));
    double sinp  = 2*(qw*qy - qz*qx);
    double pitch = (std::abs(sinp) >= 1) ? std::copysign(M_PI/2, sinp)
                                         : std::asin(sinp);

    rclcpp::Time now(msg->header.stamp);
    if (first_msg_) { last_time_ = now; first_msg_ = false; return; }
    double dt = (now - last_time_).seconds();
    last_time_ = now;
    if (dt <= 0.0 || dt > 0.05) return;

    // Crash guard.
    if (std::abs(roll) > 1.0 || std::abs(pitch) > 1.0) {
      std_msgs::msg::Float64MultiArray stop;
      stop.data.assign(5, 0.0);
      pub_rpm_->publish(stop);
      RCLCPP_WARN(this->get_logger(), "CRASH GUARD: tilt > 57 deg. Motors cut.");
      return;
    }

    // ----- Roll / Pitch PID -----
    double error_roll  = setpoint_roll_  - roll;
    double error_pitch = setpoint_pitch_ - pitch;

    // Conditional integration: accumulate trim only when near setpoint.
    if (std::abs(error_pitch) < 0.10) integral_pitch_ += error_pitch * dt;
    if (std::abs(error_roll)  < 0.10) integral_roll_  += error_roll  * dt;
    double max_integral = 3.5 / ki_;
    integral_roll_  = std::clamp(integral_roll_,  -max_integral, max_integral);
    integral_pitch_ = std::clamp(integral_pitch_, -max_integral, max_integral);

    double output_roll  = kp_*error_roll  + ki_*integral_roll_
                        + kd_*(0.0 - msg->angular_velocity.x);
    double output_pitch = kp_*error_pitch + ki_*integral_pitch_
                        + kd_*(0.0 - msg->angular_velocity.y);
    output_roll  = std::clamp(output_roll,  -30.0, 30.0);
    output_pitch = std::clamp(output_pitch, -30.0, 30.0);

    // ----- Yaw PD: hold heading at 0 -----
    double yaw_err = target_yaw_ - yaw_rad_;
    while (yaw_err >  M_PI) yaw_err -= 2*M_PI;   // wrap to [-pi, pi]
    while (yaw_err < -M_PI) yaw_err += 2*M_PI;
    double output_yaw = kp_yaw_*yaw_err - kd_yaw_*msg->angular_velocity.z;
    output_yaw = std::clamp(output_yaw, -20.0, 20.0);

    // ----- Mixer -----
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.resize(5);
    cmd.data[0] =  (base_omega_ - output_pitch + output_roll - output_yaw);  // FL CCW
    cmd.data[1] = -(base_omega_ - output_pitch - output_roll + output_yaw);  // FR CW
    cmd.data[2] = -(base_omega_ + output_pitch + output_roll + output_yaw);  // RL CW
    cmd.data[3] =  (base_omega_ + output_pitch - output_roll - output_yaw);  // RR CCW
    cmd.data[4] = 0.0;                                                        // pusher off
    pub_rpm_->publish(cmd);

    // Debug: [z, x, y, pitch_sp°, pitch°, wp_index, yaw°]
    std_msgs::msg::Float64MultiArray dbg;
    dbg.data = { z_, x_, y_, setpoint_pitch_*57.3, pitch*57.3,
                 static_cast<double>(wp_index_), yaw_deg_ };
    pub_debug_->publish(dbg);
  }

  // ---------- Mission ----------
  struct Waypoint { double x, y, z; };
  std::vector<Waypoint> mission_ = {
    {0.0, 0.0, 2.0},   // WP0: climb straight up to 2 m
    {5.0, 0.0, 2.0},   // WP1: 5 m forward
    {5.0, 5.0, 2.0},   // WP2: corner — 5 m left
    {0.0, 5.0, 2.0},   // WP3: back edge
    {0.0, 0.0, 2.0},   // WP4: return home
  };
  size_t wp_index_ = 0;
  double target_x_ = 0.0, target_y_ = 0.0, target_z_ = 2.0;
  double target_yaw_ = 0.0;

  // ---------- Physical constants ----------
  double mass_ = 3.05, ct_ = 0.00025;

  // ---------- Outer-loop gains ----------
  double kp_pos_ = 0.08, kd_pos_ = 0.35;   // position PD (kd = synthetic drag)
  double kp_z_   = 6.0,  kd_z_   = 8.0;    // altitude PD

  // ---------- Yaw gains ----------
  double kp_yaw_ = 8.0, kd_yaw_ = 3.0;

  // ---------- Inner-loop (attitude) gains ----------
  double kp_ = 2.5, ki_ = 0.5, kd_ = 3.0;

  // ---------- State ----------
  double setpoint_roll_ = 0.0, setpoint_pitch_ = 0.0;
  double base_omega_ = 173.0;
  double integral_roll_  = 0.0;
  double integral_pitch_ = 2.4;   // preloaded measured trim
  double x_ = 0.0, y_ = 0.0, z_ = 0.0;
  double yaw_rad_ = 0.0, yaw_deg_ = 0.0;
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
  bool first_msg_ = true;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_rpm_, pub_debug_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlightController>());
  rclcpp::shutdown();
  return 0;
}
