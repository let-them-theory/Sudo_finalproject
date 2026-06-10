/*
 * gazebo_bridge.cpp
 * Real robot → Gazebo synchronization bridge
 *
 * Subscribes to real robot joint_states and publishes to Gazebo controllers
 * for digital twin visualization at 50Hz.
 */

#include <map>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class GazeboBridge : public rclcpp::Node {
public:
    GazeboBridge(const std::string& real_ns = "dsr01", const std::string& gazebo_ns = "gz")
        : Node("gazebo_bridge"), real_ns_(real_ns), gazebo_ns_(gazebo_ns), count_(0)
    {
        // Subscribe to real robot joint_states
        std::string real_topic = "/" + real_ns + "/joint_states";
        real_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            real_topic, 10,
            std::bind(&GazeboBridge::real_joint_state_callback, this, std::placeholders::_1));

        // Publish to Gazebo arm trajectory controller
        std::string arm_topic = "/" + gazebo_ns + "/joint_trajectory_controller/joint_trajectory";
        arm_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(arm_topic, 10);

        // Publish to Gazebo gripper controller
        std::string gripper_topic = "/" + gazebo_ns + "/gripper_controller/commands";
        gripper_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(gripper_topic, 10);

        // 50Hz publish timer
        timer_ = this->create_wall_timer(20ms, std::bind(&GazeboBridge::publish_to_gazebo, this));

        RCLCPP_INFO(this->get_logger(), "============================================================");
        RCLCPP_INFO(this->get_logger(), "  Gazebo Digital Twin Bridge Started");
        RCLCPP_INFO(this->get_logger(), "============================================================");
        RCLCPP_INFO(this->get_logger(), "  Real robot namespace: %s", real_ns.c_str());
        RCLCPP_INFO(this->get_logger(), "  Gazebo namespace: %s", gazebo_ns.c_str());
        RCLCPP_INFO(this->get_logger(), "  Subscribed to: %s", real_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Publishing to: %s", arm_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Publishing to: %s", gripper_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "============================================================");
    }

private:
    void real_joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
            joint_positions_[msg->name[i]] = msg->position[i];
        }
    }

    void publish_to_gazebo() {
        if (joint_positions_.empty()) return;

        // Arm trajectory
        std::vector<double> arm_positions;
        for (const auto& joint : ARM_JOINTS) {
            auto it = joint_positions_.find(joint);
            arm_positions.push_back(it != joint_positions_.end() ? it->second : 0.0);
        }

        if (arm_positions.size() == ARM_JOINTS.size()) {
            auto traj_msg = trajectory_msgs::msg::JointTrajectory();
            traj_msg.joint_names = ARM_JOINTS;

            auto point = trajectory_msgs::msg::JointTrajectoryPoint();
            point.positions = arm_positions;
            point.time_from_start.sec = 0;
            point.time_from_start.nanosec = 50000000;  // 50ms

            traj_msg.points.push_back(point);
            arm_pub_->publish(traj_msg);
        }

        // Gripper positions
        std::vector<double> gripper_positions;
        for (const auto& joint : GRIPPER_JOINTS) {
            auto it = joint_positions_.find(joint);
            gripper_positions.push_back(it != joint_positions_.end() ? it->second : 0.0);
        }

        if (!gripper_positions.empty()) {
            auto gripper_msg = std_msgs::msg::Float64MultiArray();
            gripper_msg.data = gripper_positions;
            gripper_pub_->publish(gripper_msg);
        }

        ++count_;
        if (count_ % 500 == 0) {
            RCLCPP_INFO(this->get_logger(), "Published %d updates to Gazebo", count_);
        }
    }

    const std::vector<std::string> ARM_JOINTS = {
        "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"
    };
    const std::vector<std::string> GRIPPER_JOINTS = {
        "gripper_rh_r1", "gripper_rh_r2", "gripper_rh_l1", "gripper_rh_l2"
    };

    std::string real_ns_;
    std::string gazebo_ns_;
    int count_;
    std::map<std::string, double> joint_positions_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr real_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    std::string real_ns = "dsr01";
    std::string gazebo_ns = "gz";

    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg == "--real-ns" && i + 1 < argc) {
            real_ns = argv[++i];
        } else if (arg == "--gazebo-ns" && i + 1 < argc) {
            gazebo_ns = argv[++i];
        }
    }

    auto node = std::make_shared<GazeboBridge>(real_ns, gazebo_ns);

    rclcpp::spin(node);
    node.reset();
    rclcpp::shutdown();
    return 0;
}
