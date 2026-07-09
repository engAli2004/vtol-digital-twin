#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <cmath>
#include <algorithm>

class FlightController : public rclcpp::Node {
public:
  FlightController() : Node("flight_controller") {
    pub_rpm_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/rotor_velocity_controller/commands", 10);
    pub_debug_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/pid_debug", 10);
    pub_target_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/mpc_target", 10);
    sub_mpc_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/mpc_cmd", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr m) {
        if (m->data.size() >= 3) {
          mpc_pitch_ = m->data[0]; mpc_roll_ = m->data[1];
          mpc_thrust_ = m->data[2]; mpc_age_ = 0;
          RCLCPP_INFO_ONCE(this->get_logger(), ">>> MPC ENGAGED (PD on standby)");
        }
      });
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/vtol_uav/imu", rclcpp::SensorDataQoS(),
      std::bind(&FlightController::imuCallback, this, std::placeholders::_1));
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/vtol_uav/odom_est", 10,
      std::bind(&FlightController::odomCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(),
      "Flight Controller v13 CAPSTONE: hover -> min-snap square -> transition -> cruise -> brake -> land.");
  }

private:
  enum class Mode { HOVER, WAYPOINTS, ACCEL, CRUISE, BRAKE, LAND };
  // Debug mode numbers: 0=HOVER 1=WAYPOINTS 2=ACCEL 3=CRUISE 4=BRAKE 5=LAND

  // ===== OUTER LOOP (50 Hz) =====
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double x  = msg->pose.pose.position.x;
    double y  = msg->pose.pose.position.y;
    double z  = msg->pose.pose.position.z;
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    double vz = msg->twist.twist.linear.z;
    double speed = std::sqrt(vx*vx + vy*vy + vz*vz);
    double dist = std::sqrt(std::pow(target_x_-x,2) + std::pow(target_y_-y,2)
                          + std::pow(target_z_-z,2));

    // ----- Mode logic -----
    if (mode_ == Mode::HOVER) {
      if (dist < 0.8 && speed < 0.5) hover_stable_ticks_++;
      else hover_stable_ticks_ = 0;
      if (hover_stable_ticks_ >= 500) {
        mode_ = Mode::WAYPOINTS;
        RCLCPP_INFO(this->get_logger(), ">>> WAYPOINTS: min-snap square patrol.");
      }

    } else if (mode_ == Mode::WAYPOINTS) {
      // Min-snap step polynomial: zero vel/acc/jerk at both segment ends.
      seg_t_ += 0.02;
      double tau = std::min(seg_t_ / seg_T_, 1.0);
      double s = 35*std::pow(tau,4) - 84*std::pow(tau,5)
               + 70*std::pow(tau,6) - 20*std::pow(tau,7);
      target_x_ = seg_ax_ + s * (seg_bx_ - seg_ax_);
      target_y_ = seg_ay_ + s * (seg_by_ - seg_ay_);
      target_z_ = 3.0;
      if (seg_t_ >= seg_T_ + 2.0) {          // segment done + 2 s settle
        seg_++;
        if (seg_ >= 4) {
          mode_ = Mode::ACCEL;
          RCLCPP_INFO(this->get_logger(), ">>> Square complete. ACCEL: pusher ramping.");
        } else {
          seg_ax_ = seg_bx_; seg_ay_ = seg_by_;
          seg_bx_ = wx_[seg_]; seg_by_ = wy_[seg_];
          seg_t_ = 0.0;
          RCLCPP_INFO(this->get_logger(), ">>> Segment %d -> (%.0f, %.0f)",
                      seg_+1, seg_bx_, seg_by_);
        }
      }

    } else if (mode_ == Mode::ACCEL) {
      if (speed >= 11.5) cruise_ticks_++;
      else cruise_ticks_ = 0;
      if (cruise_ticks_ >= 100) {
        mode_ = Mode::CRUISE;
        RCLCPP_INFO(this->get_logger(),
          ">>> CRUISE at V=%.1f m/s: handing weight to the wings.", speed);
      }

    } else if (mode_ == Mode::CRUISE) {
      thrust_floor_ = std::max(8.0, thrust_floor_ - 4.0 * 0.02);
      if (speed < 8.0) {
        mode_ = Mode::ACCEL;
        cruise_ticks_ = 0;
        thrust_floor_ = 10.0;
        RCLCPP_WARN(this->get_logger(),
          ">>> CRUISE ABORT: V=%.1f too slow. Back to ACCEL.", speed);
      }
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "CRUISE: V=%.1f, vz=%+.1f, rotor thrust=%.1f N (%.0f%% of weight on wings)",
        speed, vz, commanded_thrust_, 100.0 * (1.0 - commanded_thrust_ / 29.9));

    } else if (mode_ == Mode::BRAKE) {
      if (dist < 0.8 && speed < 0.5) brake_stable_ticks_++;
      else brake_stable_ticks_ = 0;
      if (brake_stable_ticks_ >= 250) {      // 5 s stable hover downrange
        mode_ = Mode::LAND;
        RCLCPP_INFO(this->get_logger(), ">>> LAND: descending at 0.3 m/s.");
      }

    } else if (mode_ == Mode::LAND) {
      target_z_ = std::max(0.0, target_z_ - 0.3 * 0.02);
      if (z < 0.35 && !landed_) {
        landed_ = true;
        RCLCPP_INFO(this->get_logger(), ">>> TOUCHDOWN. MISSION COMPLETE.");
      }
    }

    // Transition end: brake once downrange
    if ((mode_ == Mode::ACCEL || mode_ == Mode::CRUISE) && x > 250.0) {
      mode_ = Mode::BRAKE;
      thrust_floor_ = 10.0;
      target_x_ = x + (speed*speed) / 11.0;
      target_y_ = y;
      target_z_ = 3.0;
      RCLCPP_INFO(this->get_logger(),
        ">>> BRAKE at x=%.1f, V=%.1f. Hover target: x=%.1f", x, speed, target_x_);
    }

    // ----- Yaw -----
    double qx = msg->pose.pose.orientation.x, qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z, qw = msg->pose.pose.orientation.w;
    yaw_rad_ = std::atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz));

    // ----- Publish target for MPC -----
    mpc_age_++;
    std_msgs::msg::Float64MultiArray tgt;
    tgt.data = { target_x_, target_y_, target_z_,
                 static_cast<double>(static_cast<int>(mode_)) };
    pub_target_->publish(tgt);

    // ----- Attitude setpoints -----
    bool mpc_fresh = (mpc_age_ < 15);
    double tilt_max = 0.087;
    if (mode_ == Mode::ACCEL || mode_ == Mode::CRUISE) {
      setpoint_pitch_ = std::clamp(0.087 - 0.025*vz, -0.09, 0.17);
      setpoint_roll_ = std::clamp(-kp_pos_*(target_y_-y) + kd_pos_*vy,
                                   -tilt_max, tilt_max);
    } else if (mpc_fresh) {
      setpoint_pitch_ = mpc_pitch_;
      setpoint_roll_  = mpc_roll_;
    } else {
      setpoint_pitch_ = std::clamp( kp_pos_*(target_x_-x) - kd_pos_*vx,
                                    -tilt_max, tilt_max);
      setpoint_roll_ = std::clamp(-kp_pos_*(target_y_-y) + kd_pos_*vy,
                                   -tilt_max, tilt_max);
    }

    // ----- Altitude -----
    double thrust = mass_*9.8 + kp_z_*(target_z_-z) - kd_z_*vz;
    if (mpc_fresh && mode_ != Mode::ACCEL && mode_ != Mode::CRUISE)
      thrust = mpc_thrust_;
    thrust = std::clamp(thrust, thrust_floor_, 45.0);
    commanded_thrust_ = thrust;
    base_omega_ = std::sqrt(thrust / (4.0 * ct_));

    x_ = x; z_ = z; speed_ = speed;
  }

  // ===== INNER LOOP (100 Hz) =====
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (landed_) {
      std_msgs::msg::Float64MultiArray stop;
      stop.data.assign(5, 0.0);
      pub_rpm_->publish(stop);
      return;
    }

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

    if (std::abs(roll) > 1.0 || std::abs(pitch) > 1.0) {
      std_msgs::msg::Float64MultiArray stop;
      stop.data.assign(5, 0.0);
      pub_rpm_->publish(stop);
      RCLCPP_WARN(this->get_logger(), "CRASH GUARD: tilt > 57 deg. Motors cut.");
      return;
    }

    if (mode_ == Mode::ACCEL || mode_ == Mode::CRUISE) {
      pusher_omega_ = std::min(pusher_omega_ + 60.0*dt, 300.0);
    } else if (mode_ == Mode::BRAKE || mode_ == Mode::LAND) {
      pusher_omega_ = std::max(pusher_omega_ - 60.0*dt, 0.0);
    }

    double error_roll  = setpoint_roll_  - roll;
    double error_pitch = setpoint_pitch_ - pitch;

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

    double yaw_err = target_yaw_ - yaw_rad_;
    while (yaw_err >  M_PI) yaw_err -= 2*M_PI;
    while (yaw_err < -M_PI) yaw_err += 2*M_PI;
    double output_yaw = kp_yaw_*yaw_err - kd_yaw_*msg->angular_velocity.z;
    output_yaw = std::clamp(output_yaw, -20.0, 20.0);

    double m_fl = std::max(0.0, base_omega_ - output_pitch + output_roll - output_yaw);
    double m_fr = std::max(0.0, base_omega_ - output_pitch - output_roll + output_yaw);
    double m_rl = std::max(0.0, base_omega_ + output_pitch + output_roll + output_yaw);
    double m_rr = std::max(0.0, base_omega_ + output_pitch - output_roll - output_yaw);

    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.resize(5);
    cmd.data[0] =  m_fl;   // FL CCW
    cmd.data[1] = -m_fr;   // FR CW
    cmd.data[2] = -m_rl;   // RL CW
    cmd.data[3] =  m_rr;   // RR CCW
    cmd.data[4] = pusher_omega_;
    pub_rpm_->publish(cmd);

    // Debug: [z, x, V, pitch deg, mode, pusher omega, thrust N, yaw deg]
    std_msgs::msg::Float64MultiArray dbg;
    dbg.data = { z_, x_, speed_, pitch*57.3,
                 static_cast<double>(static_cast<int>(mode_)),
                 pusher_omega_, commanded_thrust_, yaw_rad_*57.3 };
    pub_debug_->publish(dbg);
  }

  // ---------- Mission ----------
  Mode mode_ = Mode::HOVER;
  int hover_stable_ticks_ = 0;
  int cruise_ticks_ = 0;
  int brake_stable_ticks_ = 0;
  bool landed_ = false;
  double target_x_ = 0.0, target_y_ = 0.0, target_z_ = 3.0;
  double target_yaw_ = 0.0;
  double pusher_omega_ = 0.0;
  double thrust_floor_ = 10.0;
  double commanded_thrust_ = 29.9;

  // ---------- Min-snap square ----------
  double wx_[4] = {5.0, 5.0, 0.0, 0.0};
  double wy_[4] = {0.0, 5.0, 5.0, 0.0};
  int seg_ = 0;
  double seg_t_ = 0.0, seg_T_ = 6.0;
  double seg_ax_ = 0.0, seg_ay_ = 0.0, seg_bx_ = 5.0, seg_by_ = 0.0;

  // ---------- Physical constants ----------
  double mass_ = 3.05, ct_ = 0.00025;

  // ---------- Gains ----------
  double kp_pos_ = 0.08, kd_pos_ = 0.35;
  double kp_z_   = 6.0,  kd_z_   = 8.0;
  double kp_yaw_ = 8.0,  kd_yaw_ = 3.0;
  double kp_ = 2.5, ki_ = 0.5, kd_ = 3.0;

  // ---------- State ----------
  double setpoint_roll_ = 0.0, setpoint_pitch_ = 0.0;
  double base_omega_ = 173.0;
  double integral_roll_ = 0.0, integral_pitch_ = 2.4;
  double x_ = 0.0, z_ = 0.0, speed_ = 0.0;
  double yaw_rad_ = 0.0;
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
  bool first_msg_ = true;

  // ---------- MPC interface ----------
  double mpc_pitch_ = 0.0, mpc_roll_ = 0.0, mpc_thrust_ = 29.9;
  int mpc_age_ = 999;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_target_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_mpc_;

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
