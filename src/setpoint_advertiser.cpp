//
// Created by hien on 02.06.21.
//

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "bridge_msgs/msg/setpoint.hpp"

using namespace std::chrono_literals;


class SetpointAdvertiser : public rclcpp::Node {
public:
    SetpointAdvertiser() : Node("setpoint_advertiser") {

        parse_parameters();
        auto qos = rclcpp::QoS(
                rclcpp::QoSInitialization(
                        history_policy_,
                        depth_
                ));
        qos.reliability(reliability_policy_);

        publisher_ = this->create_publisher<bridge_msgs::msg::Setpoint>("Setpoint_PubSubTopic", qos);
        auto timer_callback =
                [this]() -> void {
                    auto setpoint = bridge_msgs::msg::Setpoint();
                    setpoint.header.stamp = this->now();
                    count_++;
                    setpoint.header.frame_id = std::to_string(count_);
                    setpoint.x = count_;
                    setpoint.y = 0.0;
                    setpoint.z = -3.0;
                    RCLCPP_INFO(this->get_logger(), "Publishing setpoint %d: second: %llu nanosecond: %llu x: %f y: %f z: %f ",
                                count_, setpoint.header.stamp.sec, setpoint.header.stamp.nanosec, setpoint.x, setpoint.y, setpoint.z);
                    this->publisher_->publish(setpoint);
                };
        timer_ = this->create_wall_timer(100ms, timer_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<bridge_msgs::msg::Setpoint>::SharedPtr publisher_;
    size_t count_;

    size_t depth_;
    rmw_qos_reliability_policy_t reliability_policy_;
    rmw_qos_history_policy_t history_policy_;
    std::map<std::string, rmw_qos_reliability_policy_t> name_to_reliability_policy_map = {
            {"reliable", RMW_QOS_POLICY_RELIABILITY_RELIABLE},
            {"best_effort", RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT}
    };

    std::map<std::string, rmw_qos_history_policy_t> name_to_history_policy_map = {
            {"keep_last", RMW_QOS_POLICY_HISTORY_KEEP_LAST},
            {"keep_all", RMW_QOS_POLICY_HISTORY_KEEP_ALL}
    };

    void parse_parameters() {
        // Parse 'reliability' parameter
        rcl_interfaces::msg::ParameterDescriptor reliability_desc;
        reliability_desc.description = "Reliability QoS setting for the listener";
        reliability_desc.additional_constraints = "Must be one of: ";
        for (auto entry : name_to_reliability_policy_map) {
            reliability_desc.additional_constraints += entry.first + " ";
        }
        const std::string reliability_param = this->declare_parameter(
                "reliability", "reliable", reliability_desc);
        auto reliability = name_to_reliability_policy_map.find(reliability_param);
        if (reliability == name_to_reliability_policy_map.end()) {
            std::ostringstream oss;
            oss << "Invalid QoS reliability setting '" << reliability_param << "'";
            throw std::runtime_error(oss.str());
        }
        reliability_policy_ = reliability->second;

        // Parse 'history' parameter
        rcl_interfaces::msg::ParameterDescriptor history_desc;
        history_desc.description = "History QoS setting for the listener";
        history_desc.additional_constraints = "Must be one of: ";
        for (auto entry : name_to_history_policy_map) {
            history_desc.additional_constraints += entry.first + " ";
        }
        const std::string history_param = this->declare_parameter(
                "history", name_to_history_policy_map.begin()->first, history_desc);
        auto history = name_to_history_policy_map.find(history_param);
        if (history == name_to_history_policy_map.end()) {
            std::ostringstream oss;
            oss << "Invalid QoS history setting '" << history_param << "'";
            throw std::runtime_error(oss.str());
        }
        history_policy_ = history->second;

        // Declare and get remaining parameters
        depth_ = this->declare_parameter("depth", 10);
    }



};

int main(int argc, char *argv[]) {
    std::cout << "Starting TrajectorySetpoint advertiser node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SetpointAdvertiser>());
    rclcpp::shutdown();
    return 0;
}