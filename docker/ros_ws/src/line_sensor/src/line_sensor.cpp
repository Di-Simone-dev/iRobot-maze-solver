#include "rclcpp/rclcpp.hpp"
#include "message_interfaces/msg/line_sensor_msg.hpp"

using namespace std::chrono_literals;

class LineSensor : public rclcpp::Node {
    public:
        LineSensor() : Node("line_sensor") {
            // Creating publisher
            publisher_ = this->create_publisher<message_interfaces::msg::LineSensorMsg>("line_sensor_status", 2);

            // Creating timer
            timer_ = this->create_wall_timer(0.5s, std::bind(&LineSensor::publish_message, this));
        }
    
    private:
        rclcpp::Publisher<message_interfaces::msg::LineSensorMsg>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        void publish_message() {
            auto message = message_interfaces::msg::LineSensorMsg();
            message.status = "Prova";
            RCLCPP_INFO(this->get_logger(), "Pubblicato: '%s'", message.status.c_str());
            publisher_->publish(message);
        }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineSensor>());
    rclcpp::shutdown();
    return 0;
}