#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include <string>
#include <cmath>

class AerodynamicsNode : public rclcpp::Node {
public:
  AerodynamicsNode() : Node("aerodynamics_node") {
    pub_fl_ = this->create_publisher<geometry_msgs::msg::Wrench>("/vtol_uav/force_fl", 10);
    pub_fr_ = this->create_publisher<geometry_msgs::msg::Wrench>("/vtol_uav/force_fr", 10);
    pub_rl_ = this->create_publisher<geometry_msgs::msg::Wrench>("/vtol_uav/force_rl", 10);
    pub_rr_ = this->create_publisher<geometry_msgs::msg::Wrench>("/vtol_uav/force_rr", 10);
    pub_pusher_ = this->create_publisher<geometry_msgs::msg::Wrench>("/vtol_uav/force_pusher", 10);
    pub_body_ = this->create_publisher<geometry_msgs::msg::Wrench>("/vtol_uav/force_body", 10);

    sub_joints_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&AerodynamicsNode::jointCallback, this, std::placeholders::_1));
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/vtol_uav/ground_truth", 10,
      std::bind(&AerodynamicsNode::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "Aerodynamics v4: thrust + pusher + body drag + wing polars (stall at %.0f deg)",
      alpha_stall_ * 57.3);
  }

private:
  // ===== ROTOR THRUST: T = C_T * omega^2 =====
  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      std::string joint_name = msg->name[i];
      if (joint_name.find("rotor") == std::string::npos) continue;

      double omega = msg->velocity[i];
      double thrust = thrust_coeff_ * (omega * omega);
      geometry_msgs::msg::Wrench wrench_msg;

      if (joint_name == "joint_rotor_pusher") {
        // Pusher: thrust along the aircraft's forward (x) axis
        wrench_msg.force.z = thrust;  // link z = spin axis = body forward
        pub_pusher_->publish(wrench_msg);
      } else {
        // Lift rotors: thrust along local up (z)
        wrench_msg.force.z = thrust;
        if      (joint_name == "joint_rotor_fl") pub_fl_->publish(wrench_msg);
        else if (joint_name == "joint_rotor_fr") pub_fr_->publish(wrench_msg);
        else if (joint_name == "joint_rotor_rl") pub_rl_->publish(wrench_msg);
        else if (joint_name == "joint_rotor_rr") pub_rr_->publish(wrench_msg);
      }
    }
  }

  // ===== BODY DRAG + WING LIFT/DRAG (unchanged from v3) =====
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    double vz = msg->twist.twist.linear.z;
    double speed = std::sqrt(vx*vx + vy*vy + vz*vz);

    geometry_msgs::msg::Wrench total;
    total.force.x = -(k_lin_ * vx + k_quad_ * speed * vx);
    total.force.y = -(k_lin_ * vy + k_quad_ * speed * vy);
    total.force.z = -(k_lin_ * vz + k_quad_ * speed * vz);

    double qx = msg->pose.pose.orientation.x, qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z, qw = msg->pose.pose.orientation.w;

    double vxb = (1 - 2*(qy*qy + qz*qz))*vx + (2*(qx*qy + qw*qz))*vy + (2*(qx*qz - qw*qy))*vz;
    double vzb = (2*(qx*qz + qw*qy))*vx + (2*(qy*qz - qw*qx))*vy + (1 - 2*(qx*qx + qy*qy))*vz;

    if (vxb > 1.0) {
      double alpha = std::atan2(-vzb, vxb);

      double cl;
      if (std::abs(alpha) < alpha_stall_) {
        cl = cl0_ + cla_ * alpha;
      } else {
        double cl_at_stall = cl0_ + cla_ * alpha_stall_ * (alpha > 0 ? 1.0 : -1.0);
        double fade = 1.0 - (std::abs(alpha) - alpha_stall_) / 0.2;
        cl = cl_at_stall * std::max(0.0, fade);
      }

      double cd = cd0_ + k_induced_ * cl * cl;
      double q_dyn = 0.5 * rho_ * speed * speed;
      double lift = q_dyn * wing_area_ * cl;
      double drag = q_dyn * wing_area_ * cd;

      total.force.z += lift;
      if (speed > 0.1) {
        total.force.x += -drag * vx / speed;
        total.force.y += -drag * vy / speed;
        total.force.z += -drag * vz / speed;
      }

      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "WINGS ACTIVE: V=%.1f m/s, alpha=%.1f deg, CL=%.2f, lift=%.1f N%s",
        speed, alpha*57.3, cl, lift,
        (std::abs(alpha) >= alpha_stall_) ? "  ** STALL **" : "");
    }

    pub_body_->publish(total);
  }

  // ---------- Aerodynamic constants ----------
  double thrust_coeff_ = 0.00025;
  double rho_    = 1.225;
  double k_lin_  = 0.05;
  double k_quad_ = 0.092;
  double wing_area_   = 0.4;
  double cl0_         = 0.2;
  double cla_         = 5.0;
  double alpha_stall_ = 0.26;
  double cd0_         = 0.03;
  double k_induced_   = 0.05;

  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr
    pub_fl_, pub_fr_, pub_rl_, pub_rr_, pub_pusher_, pub_body_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joints_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AerodynamicsNode>());
  rclcpp::shutdown();
  return 0;
}
