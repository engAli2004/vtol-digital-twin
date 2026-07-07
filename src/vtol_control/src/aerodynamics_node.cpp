#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include <string>
#include <cmath>

class AerodynamicsNode : public rclcpp::Node {
public:
  AerodynamicsNode() : Node("aerodynamics_node") {
    // 1. Publishers to Gazebo force plugins (rotor thrust, link frame)
    pub_fl_ = this->create_publisher<geometry_msgs::msg::Wrench>("/vtol_uav/force_fl", 10);
    pub_fr_ = this->create_publisher<geometry_msgs::msg::Wrench>("/vtol_uav/force_fr", 10);
    pub_rl_ = this->create_publisher<geometry_msgs::msg::Wrench>("/vtol_uav/force_rl", 10);
    pub_rr_ = this->create_publisher<geometry_msgs::msg::Wrench>("/vtol_uav/force_rr", 10);

    // 2. Publisher for body aerodynamic forces (drag, world frame)
    pub_body_ = this->create_publisher<geometry_msgs::msg::Wrench>("/vtol_uav/force_body", 10);

    // 3. Subscribe to motor speeds (thrust computation)
    sub_joints_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&AerodynamicsNode::jointCallback, this, std::placeholders::_1));

    // 4. Subscribe to velocity (drag computation)
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/vtol_uav/ground_truth", 10,
      std::bind(&AerodynamicsNode::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "Aerodynamics Node v2: thrust + body drag. CT=%f, k_quad=%.3f",
      thrust_coeff_, k_quad_);
  }

private:
  // ===== ROTOR THRUST: T = C_T * omega^2 (unchanged) =====
  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      std::string joint_name = msg->name[i];

      if (joint_name.find("rotor") != std::string::npos &&
          joint_name != "joint_rotor_pusher") {

        double omega = msg->velocity[i];
        double thrust = thrust_coeff_ * (omega * omega);

        geometry_msgs::msg::Wrench wrench_msg;
        wrench_msg.force.z = thrust;

        if      (joint_name == "joint_rotor_fl") pub_fl_->publish(wrench_msg);
        else if (joint_name == "joint_rotor_fr") pub_fr_->publish(wrench_msg);
        else if (joint_name == "joint_rotor_rl") pub_rl_->publish(wrench_msg);
        else if (joint_name == "joint_rotor_rr") pub_rr_->publish(wrench_msg);
      }
    }
  }

  // ===== BODY DRAG: F = -(k_lin*v + k_quad*|v|*v), world frame =====
  // Physics: F_drag = 0.5 * rho * v^2 * C_D * A, opposing velocity.
  // k_quad = 0.5 * 1.225 * 1.0 * 0.15 = 0.092 N per (m/s)^2
  // k_lin adds gentle low-speed damping (viscous effects, rotor downwash).
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    double vz = msg->twist.twist.linear.z;

    double speed = std::sqrt(vx*vx + vy*vy + vz*vz);

    geometry_msgs::msg::Wrench drag;
    drag.force.x = -(k_lin_ * vx + k_quad_ * speed * vx);
    drag.force.y = -(k_lin_ * vy + k_quad_ * speed * vy);
    drag.force.z = -(k_lin_ * vz + k_quad_ * speed * vz);
    pub_body_->publish(drag);
  }

  // ---------- Aerodynamic constants ----------
  double thrust_coeff_ = 0.00025;  // C_T, experimentally determined
  double k_lin_  = 0.05;           // linear drag [N per m/s]
  double k_quad_ = 0.092;          // quadratic drag [N per (m/s)^2]

  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr
    pub_fl_, pub_fr_, pub_rl_, pub_rr_, pub_body_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joints_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AerodynamicsNode>());
  rclcpp::shutdown();
  return 0;
}
