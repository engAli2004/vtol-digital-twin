#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include <string>
#include <cmath>

class AerodynamicsNode : public rclcpp::Node {
public:
    AerodynamicsNode() : Node("aerodynamics_node") {
        // 1. Create Publishers to Gazebo Force Plugins
        pub_fl_ = this->create_publisher<geometry_msgs::msg::Wrench>("/vtol_uav/force_fl", 10);
        pub_fr_ = this->create_publisher<geometry_msgs::msg::Wrench>("/vtol_uav/force_fr", 10);
        pub_rl_ = this->create_publisher<geometry_msgs::msg::Wrench>("/vtol_uav/force_rl", 10);
        pub_rr_ = this->create_publisher<geometry_msgs::msg::Wrench>("/vtol_uav/force_rr", 10);

        // 2. Subscribe to Motor Speeds (from ros2_control)
        sub_joints_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&AerodynamicsNode::jointCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Aerodynamics Atmosphere Node Online. CT = %f", thrust_coeff_);
    }

private:
    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Iterate through all broadcasting joints
        for (size_t i = 0; i < msg->name.size(); ++i) {
            std::string joint_name = msg->name[i];
            
            // We only care about the hover rotors
            if (joint_name.find("rotor") != std::string::npos && joint_name != "joint_rotor_pusher") {
                
                double omega = msg->velocity[i];
                
                // The Fundamental Equation of Multirotor Flight: T = C_T * (omega^2)
                double thrust = thrust_coeff_ * (omega * omega);
                
                // Create the Force Vector (Pure Z-axis lift)
                geometry_msgs::msg::Wrench wrench_msg;
                wrench_msg.force.z = thrust;

                // Route the calculated Newtons to the correct physical motor
                if (joint_name == "joint_rotor_fl") pub_fl_->publish(wrench_msg);
                else if (joint_name == "joint_rotor_fr") pub_fr_->publish(wrench_msg);
                else if (joint_name == "joint_rotor_rl") pub_rl_->publish(wrench_msg);
                else if (joint_name == "joint_rotor_rr") pub_rr_->publish(wrench_msg);
            }
        }
    }

    // Aerodynamic Thrust Coefficient (Experimentally determined for this URDF)
    double thrust_coeff_ = 0.00025; 

    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_fl_, pub_fr_, pub_rl_, pub_rr_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joints_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AerodynamicsNode>());
    rclcpp::shutdown();
    return 0;
}
