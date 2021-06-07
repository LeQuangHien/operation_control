//
// Created by hien on 04.06.21.
//

#include <rclcpp/rclcpp.hpp>
#include <bridge_msgs/msg/sensor.hpp>


/**
 * @brief Sensor Combined uORB topic data callback
 */
class SensorBridgeListener : public rclcpp::Node {
public:
    explicit SensorBridgeListener() : Node("sensor_bridge_listener") {

        parse_parameters();
        auto qos = rclcpp::QoS(
                rclcpp::QoSInitialization(
                        history_policy_,
                        depth_
                ));
        qos.reliability(reliability_policy_);

        // manually enable topic statistics via options
        auto options = rclcpp::SubscriptionOptions();
        options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;

        // configure the topic name (default '/statistics')
        options.topic_stats_options.publish_topic = "/statistics_sensor";

        subscription_ = this->create_subscription<bridge_msgs::msg::Sensor>(
                "SensorBridge_PubSubTopic", qos,

                [this](const bridge_msgs::msg::Sensor::UniquePtr msg) {
                    std::cout << "\n";
                    std::cout << "RECEIVED SENSOR COMBINED DATA" << std::endl;
                    std::cout << "=============================" << std::endl;
                    std::cout << "seconds: " << msg->header.stamp.sec << std::endl;
                    std::cout << "nanoseconds: " << msg->header.stamp.nanosec << std::endl;
                    std::cout << "gyro_rad[0]: " << msg->gyro_rad[0] << std::endl;
                    std::cout << "gyro_rad[1]: " << msg->gyro_rad[1] << std::endl;
                    std::cout << "gyro_rad[2]: " << msg->gyro_rad[2] << std::endl;
                    std::cout << "gyro_integral_dt: " << msg->gyro_integral_dt << std::endl;
                    std::cout << "accelerometer_timestamp_relative: " << msg->accelerometer_timestamp_relative
                              << std::endl;
                    std::cout << "accelerometer_m_s2[0]: " << msg->accelerometer_m_s2[0] << std::endl;
                    std::cout << "accelerometer_m_s2[1]: " << msg->accelerometer_m_s2[1] << std::endl;
                    std::cout << "accelerometer_m_s2[2]: " << msg->accelerometer_m_s2[2] << std::endl;
                    std::cout << "accelerometer_integral_dt: " << msg->accelerometer_integral_dt << std::endl;

                }, options);
    }

private:
    rclcpp::Subscription<bridge_msgs::msg::Sensor>::SharedPtr subscription_;
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
    std::cout << "Starting sensor_bridge listener node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorBridgeListener>());

    rclcpp::shutdown();
    return 0;
}



