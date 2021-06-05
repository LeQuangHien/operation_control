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

        rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
        auto qos = rclcpp::QoS(
                rclcpp::QoSInitialization(
                        qos_profile.history,
                        qos_profile.depth
                ),
                qos_profile);

        //qos.best_effort();

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
};


int main(int argc, char *argv[]) {
    std::cout << "Starting sensor_bridge listener node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorBridgeListener>());

    rclcpp::shutdown();
    return 0;
}



