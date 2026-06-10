/*
 * gripper_service_node.cpp
 * RH-P12-RN-A gripper ROS2 service node (C++ implementation)
 * Controls gripper via Modbus RTU over Doosan Tool Flange Serial
 *
 * Interfaces:
 *   Services:
 *     /dsr01/gripper/open   - Open gripper (Trigger)
 *     /dsr01/gripper/close  - Close gripper (Trigger)
 *   Topics:
 *     /dsr01/gripper/position_cmd - Position command (Int32, 0~700)
 *     /dsr01/gripper/stroke       - Current stroke (Int32, for RViz)
 */

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/int32.hpp"
#include "dsr_msgs2/srv/flange_serial_open.hpp"
#include "dsr_msgs2/srv/flange_serial_close.hpp"
#include "dsr_msgs2/srv/flange_serial_write.hpp"
#include "dsr_msgs2/srv/flange_serial_read.hpp"

#include "e0509_gripper_description/modbus_rtu.hpp"

using namespace std::chrono_literals;

class GripperWorker {
public:
    GripperWorker(const std::string& ns, bool real_mode, rclcpp::Logger logger)
        : namespace_(ns), port_(1), real_mode_(real_mode), logger_(logger) {}

    bool init_clients() {
        if (!real_mode_) {
            RCLCPP_INFO(logger_, "Virtual mode - gripper hardware control disabled");
            return false;
        }

        worker_node_ = rclcpp::Node::make_shared("gripper_worker_node");

        std::string prefix = "/" + namespace_ + "/gripper";
        cli_open_ = worker_node_->create_client<dsr_msgs2::srv::FlangeSerialOpen>(prefix + "/flange_serial_open");
        cli_close_ = worker_node_->create_client<dsr_msgs2::srv::FlangeSerialClose>(prefix + "/flange_serial_close");
        cli_write_ = worker_node_->create_client<dsr_msgs2::srv::FlangeSerialWrite>(prefix + "/flange_serial_write");
        cli_read_ = worker_node_->create_client<dsr_msgs2::srv::FlangeSerialRead>(prefix + "/flange_serial_read");

        RCLCPP_INFO(logger_, "Waiting for Doosan flange serial services...");

        bool all_ready = true;
        for (const auto& [cli, name] : std::vector<std::pair<rclcpp::ClientBase::SharedPtr, std::string>>{
                {cli_open_, "flange_serial_open"},
                {cli_close_, "flange_serial_close"},
                {cli_write_, "flange_serial_write"},
                {cli_read_, "flange_serial_read"}}) {
            if (!cli->wait_for_service(10s)) {
                RCLCPP_ERROR(logger_, "Service %s connection failed!", name.c_str());
                all_ready = false;
            }
        }

        if (!all_ready) {
            real_mode_ = false;
            return false;
        }

        RCLCPP_INFO(logger_, "Doosan flange serial services connected - Real mode ready");
        return true;
    }

    std::pair<bool, std::string> execute_command(int position) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!real_mode_) {
            std::string msg = "Virtual mode: position=" + std::to_string(position);
            RCLCPP_INFO(logger_, "%s", msg.c_str());
            return {true, msg};
        }

        RCLCPP_INFO(logger_, "Real mode: gripper control start (position=%d)", position);

        if (!serial_open()) {
            return {false, "Serial port open failed"};
        }

        std::this_thread::sleep_for(100ms);
        if (!enable_torque()) {
            serial_close();
            return {false, "Torque enable failed"};
        }

        std::this_thread::sleep_for(200ms);
        RCLCPP_INFO(logger_, "Setting position: %d", position);
        if (!serial_write(modbus_rtu::fc16_position(position))) {
            serial_close();
            return {false, "Position set failed"};
        }

        std::this_thread::sleep_for(1000ms);
        serial_close();
        return {true, "Gripper position set: " + std::to_string(position)};
    }

    void destroy() {
        if (worker_node_) {
            worker_node_.reset();
        }
    }

private:
    bool serial_open(int baudrate = 57600, int max_retries = 3) {
        auto req = std::make_shared<dsr_msgs2::srv::FlangeSerialOpen::Request>();
        req->port = port_;
        req->baudrate = baudrate;
        req->bytesize = 8;
        req->parity = 0;
        req->stopbits = 1;

        for (int attempt = 0; attempt < max_retries; ++attempt) {
            auto future = cli_open_->async_send_request(req);
            if (rclcpp::spin_until_future_complete(worker_node_, future, 10s) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                auto result = future.get();
                if (result && result->success) {
                    RCLCPP_INFO(logger_, "Serial port opened");
                    return true;
                }
            }
            if (attempt < max_retries - 1) {
                RCLCPP_WARN(logger_, "Serial open failed, force close and retry (%d/%d)",
                           attempt + 1, max_retries);
                serial_close();
                std::this_thread::sleep_for(300ms);
            }
        }
        RCLCPP_ERROR(logger_, "Serial port open failed (retries exhausted)");
        return false;
    }

    void serial_close() {
        auto req = std::make_shared<dsr_msgs2::srv::FlangeSerialClose::Request>();
        req->port = port_;
        auto future = cli_close_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(worker_node_, future, 10s) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = future.get();
            if (result && result->success) {
                RCLCPP_INFO(logger_, "Serial port closed");
            }
        }
    }

    bool serial_write(const std::vector<uint8_t>& data) {
        auto req = std::make_shared<dsr_msgs2::srv::FlangeSerialWrite::Request>();
        req->port = port_;
        req->data.assign(data.begin(), data.end());

        auto future = cli_write_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(worker_node_, future, 10s) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = future.get();
            if (result && result->success) {
                RCLCPP_INFO(logger_, "Write success (%zu bytes)", data.size());
                return true;
            }
        }
        RCLCPP_ERROR(logger_, "Write failed");
        return false;
    }

    bool enable_torque() {
        RCLCPP_INFO(logger_, "Enabling torque...");
        return serial_write(modbus_rtu::fc06_torque_enable());
    }

public:
    std::vector<uint8_t> serial_read(float timeout_sec = 1.0) {
        auto req = std::make_shared<dsr_msgs2::srv::FlangeSerialRead::Request>();
        req->port = port_;
        req->timeout = timeout_sec;

        auto future = cli_read_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(worker_node_, future, 10s) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = future.get();
            if (result && result->success && result->size > 0) {
                RCLCPP_INFO(logger_, "Read success (%d bytes)", result->size);
                return std::vector<uint8_t>(result->data.begin(), result->data.end());
            }
        }
        RCLCPP_WARN(logger_, "Read failed or empty");
        return {};
    }

    int read_present_position() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!real_mode_) return -1;

        if (!serial_open()) return -1;

        std::this_thread::sleep_for(200ms);

        // Send FC03 read request
        auto frame = modbus_rtu::fc03_read_present_position();
        if (!serial_write(frame)) {
            serial_close();
            return -1;
        }

        // Wait for gripper to process and respond
        std::this_thread::sleep_for(500ms);

        // Read response with longer timeout
        auto response = serial_read(3.0);
        serial_close();

        if (response.empty()) {
            RCLCPP_WARN(logger_, "No response from gripper for position read");
            return -1;
        }

        // Log raw response for debugging
        std::string hex_str;
        for (auto b : response) {
            char buf[4];
            snprintf(buf, sizeof(buf), "%02X ", b);
            hex_str += buf;
        }
        RCLCPP_INFO(logger_, "Gripper response (%zu bytes): %s", response.size(), hex_str.c_str());

        int position = modbus_rtu::parse_present_position(response);
        if (position >= 0) {
            RCLCPP_INFO(logger_, "Present position: %d", position);
        } else {
            RCLCPP_WARN(logger_, "Failed to parse position from response");
        }
        return position;
    }

    std::string namespace_;
    int port_;
    bool real_mode_;
    rclcpp::Logger logger_;
    std::mutex mutex_;
    rclcpp::Node::SharedPtr worker_node_;
    rclcpp::Client<dsr_msgs2::srv::FlangeSerialOpen>::SharedPtr cli_open_;
    rclcpp::Client<dsr_msgs2::srv::FlangeSerialClose>::SharedPtr cli_close_;
    rclcpp::Client<dsr_msgs2::srv::FlangeSerialWrite>::SharedPtr cli_write_;
    rclcpp::Client<dsr_msgs2::srv::FlangeSerialRead>::SharedPtr cli_read_;
};


class GripperServiceNode : public rclcpp::Node {
public:
    GripperServiceNode(const std::string& ns = "dsr01")
        : Node("gripper_service_node"), namespace_(ns), current_position_(0)
    {
        this->declare_parameter<std::string>("mode", "virtual");
        std::string mode = this->get_parameter("mode").as_string();
        bool real_mode = (mode == "real");

        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Mode: %s", mode.c_str());
        RCLCPP_INFO(this->get_logger(), "========================================");

        worker_ = std::make_unique<GripperWorker>(ns, real_mode, this->get_logger());

        std::string prefix = "/" + ns + "/gripper";
        stroke_pub_ = this->create_publisher<std_msgs::msg::Int32>(prefix + "/stroke", 10);

        srv_open_ = this->create_service<std_srvs::srv::Trigger>(
            prefix + "/open",
            std::bind(&GripperServiceNode::handle_open, this,
                     std::placeholders::_1, std::placeholders::_2));

        srv_close_ = this->create_service<std_srvs::srv::Trigger>(
            prefix + "/close",
            std::bind(&GripperServiceNode::handle_close, this,
                     std::placeholders::_1, std::placeholders::_2));

        position_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            prefix + "/position_cmd", 10,
            std::bind(&GripperServiceNode::handle_position_cmd, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Gripper service node started (namespace: %s)", ns.c_str());
        RCLCPP_INFO(this->get_logger(), "  Services: %s/open, %s/close", prefix.c_str(), prefix.c_str());
        RCLCPP_INFO(this->get_logger(), "  Topics: %s/position_cmd (sub), %s/stroke (pub)",
                    prefix.c_str(), prefix.c_str());
    }

    void init_worker() {
        worker_->init_clients();

        // Initialize gripper to open position on startup
        RCLCPP_INFO(this->get_logger(), "Initializing gripper to open position...");
        auto [success, message] = worker_->execute_command(0);
        if (success) {
            RCLCPP_INFO(this->get_logger(), "Gripper initialized to open position");
            publish_stroke(0);
        } else {
            RCLCPP_WARN(this->get_logger(), "Gripper init failed: %s, defaulting to 0 (open)", message.c_str());
        }
    }

    ~GripperServiceNode() override {
        if (worker_) {
            worker_->destroy();
        }
    }

private:
    void publish_stroke(int position) {
        auto msg = std_msgs::msg::Int32();
        msg.data = position;
        stroke_pub_->publish(msg);
        current_position_ = position;
    }

    std::pair<bool, std::string> execute_in_thread(int position) {
        publish_stroke(position);

        auto worker_task = std::async(std::launch::async, [this, position]() {
            return worker_->execute_command(position);
        });

        if (worker_task.wait_for(15s) == std::future_status::timeout) {
            return {false, "Gripper command timeout"};
        }

        return worker_task.get();
    }

    void handle_open(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Gripper open request received");
        auto [success, message] = execute_in_thread(0);
        response->success = success;
        response->message = message;
        RCLCPP_INFO(this->get_logger(), "Gripper open result: %s", message.c_str());
    }

    void handle_close(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Gripper close request received");
        auto [success, message] = execute_in_thread(700);
        response->success = success;
        response->message = message;
        RCLCPP_INFO(this->get_logger(), "Gripper close result: %s", message.c_str());
    }

    void handle_position_cmd(const std_msgs::msg::Int32::SharedPtr msg) {
        int position = msg->data;
        RCLCPP_INFO(this->get_logger(), "Position command received: %d", position);
        auto [success, message] = execute_in_thread(position);
        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
        }
    }

    std::string namespace_;
    int current_position_;
    std::unique_ptr<GripperWorker> worker_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr stroke_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_open_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_close_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr position_sub_;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    std::string ns = "dsr01";
    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg == "--namespace" && i + 1 < argc) {
            ns = argv[++i];
        } else if (arg.rfind("--namespace=", 0) == 0) {
            ns = arg.substr(12);
        }
    }

    auto node = std::make_shared<GripperServiceNode>(ns);
    node->init_worker();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
