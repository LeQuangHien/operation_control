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
    SetpointAdvertiser() : Node("trajectory_advertiser") {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
        auto qos = rclcpp::QoS(
                rclcpp::QoSInitialization(
                        qos_profile.history,
                        qos_profile.depth
                ),
                qos_profile);
        qos.best_effort();

        publisher_ = this->create_publisher<bridge_msgs::msg::Setpoint>("Setpoint_PubSubTopic", qos);
        auto timer_callback =
                [this]() -> void {
                    auto setpoint = bridge_msgs::msg::Setpoint();
                    setpoint.header.stamp = this->now();
                    setpoint.x = count_++;
                    setpoint.y = 0.0;
                    setpoint.z = -3.0;
                    RCLCPP_INFO(this->get_logger(), "Publishing setpoint %d: time: %llu x: %f y: %f z: %f ",
                                count_ - 1, setpoint.header.stamp, setpoint.x, setpoint.y, setpoint.z);
                    this->publisher_->publish(setpoint);
                };
        timer_ = this->create_wall_timer(100ms, timer_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<bridge_msgs::msg::Setpoint>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char *argv[]) {
    std::cout << "Starting TrajectorySetpoint advertiser node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SetpointAdvertiser>());
    rclcpp::shutdown();
    return 0;
}