/*
 * gripper_joint_publisher.cpp
 * Combined joint state publisher for RViz visualization
 *
 * Combines robot arm joints (from dynamic_joint_states) and gripper joints
 * (from gripper/stroke topic) into a single joint_states message at 50Hz.
 */

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class GripperJointPublisher : public rclcpp::Node {
public:
    GripperJointPublisher()
        : Node("gripper_joint_publisher"),
          stroke_(0.0), target_stroke_(0.0), stroke_speed_(50.0),
          stroke_to_rad_(1.0 / 700.0)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        timer_ = this->create_wall_timer(20ms, std::bind(&GripperJointPublisher::publish_joint_states, this));

        // Initialize arm joint positions
        for (const auto& name : arm_joint_names_) {
            arm_positions_[name] = 0.0;
            arm_velocities_[name] = 0.0;
        }

        // Subscribe to dynamic_joint_states (arm joints)
        dynamic_sub_ = this->create_subscription<control_msgs::msg::DynamicJointState>(
            "dynamic_joint_states", 10,
            std::bind(&GripperJointPublisher::dynamic_joint_state_callback, this, std::placeholders::_1));

        // Subscribe to gripper/stroke
        stroke_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "gripper/stroke", 10,
            std::bind(&GripperJointPublisher::stroke_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Combined Joint Publisher Ready!");
        RCLCPP_INFO(this->get_logger(), "----------------------------------------");
        RCLCPP_INFO(this->get_logger(), "Subscribing to:");
        RCLCPP_INFO(this->get_logger(), "  dynamic_joint_states - Arm joints");
        RCLCPP_INFO(this->get_logger(), "  gripper/stroke - Int32 (0~700)");
        RCLCPP_INFO(this->get_logger(), "Publishing to:");
        RCLCPP_INFO(this->get_logger(), "  joint_states - All joint angles");
        RCLCPP_INFO(this->get_logger(), "========================================");
    }

private:
    void dynamic_joint_state_callback(const control_msgs::msg::DynamicJointState::SharedPtr msg) {
        for (size_t i = 0; i < msg->joint_names.size(); ++i) {
            const auto& name = msg->joint_names[i];
            if (arm_positions_.count(name) && i < msg->interface_values.size()) {
                const auto& iface = msg->interface_values[i];
                if (!iface.values.empty()) {
                    arm_positions_[name] = iface.values[0];
                }
                if (iface.values.size() > 1) {
                    arm_velocities_[name] = iface.values[1];
                }
            }
        }
    }

    void stroke_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        target_stroke_ = std::clamp(static_cast<double>(msg->data), 0.0, 700.0);
    }

    void publish_joint_states() {
        // Smooth gripper movement
        if (std::abs(stroke_ - target_stroke_) > 1.0) {
            if (stroke_ < target_stroke_) {
                stroke_ = std::min(stroke_ + stroke_speed_, target_stroke_);
            } else {
                stroke_ = std::max(stroke_ - stroke_speed_, target_stroke_);
            }
        } else {
            stroke_ = target_stroke_;
        }

        double gripper_angle = stroke_ * stroke_to_rad_;

        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->get_clock()->now();

        // Build joint names
        msg.name.insert(msg.name.end(), arm_joint_names_.begin(), arm_joint_names_.end());
        msg.name.insert(msg.name.end(), gripper_joint_names_.begin(), gripper_joint_names_.end());

        // Arm positions
        for (const auto& name : arm_joint_names_) {
            msg.position.push_back(arm_positions_[name]);
            msg.velocity.push_back(arm_velocities_[name]);
        }

        // Gripper positions (all 4 joints same angle)
        for (size_t i = 0; i < gripper_joint_names_.size(); ++i) {
            msg.position.push_back(gripper_angle);
            msg.velocity.push_back(0.0);
        }

        msg.effort.resize(msg.name.size(), 0.0);
        publisher_->publish(msg);
    }

    const std::vector<std::string> arm_joint_names_ = {
        "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"
    };
    const std::vector<std::string> gripper_joint_names_ = {
        "gripper_rh_r1", "gripper_rh_r2", "gripper_rh_l1", "gripper_rh_l2"
    };

    std::map<std::string, double> arm_positions_;
    std::map<std::string, double> arm_velocities_;
    double stroke_;
    double target_stroke_;
    double stroke_speed_;
    double stroke_to_rad_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr dynamic_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr stroke_sub_;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperJointPublisher>();
    rclcpp::spin(node);
    node.reset();
    rclcpp::shutdown();
    return 0;
}
