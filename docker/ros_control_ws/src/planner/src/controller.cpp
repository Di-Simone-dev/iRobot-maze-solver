#include "rclcpp/rclcpp.hpp"
#include "irobot_create_msgs/msg/dock_status.hpp"
#include "custom_msg/msg/command.hpp"


class Controller : public rclcpp::Node {
    public:
        Controller(): Node("planner") {
            
            // Dock subscription
            rclcpp::QoS dock_qos = rclcpp::QoS(1).best_effort().durability_volatile();
            dockSubscription = this->create_subscription<irobot_create_msgs::msg::DockStatus>(
                "dock_status",
                dock_qos,
                [this](irobot_create_msgs::msg::DockStatus::SharedPtr msg) {
                    this->dockSubCallback(msg);
                }
            );

            // Command subscription
            rclcpp::QoS command_qos = rclcpp::QoS(5).reliable().durability_volatile();
            commandSubscription = this->create_subscription<custom_msg::msg::Command>(
                "command",
                command_qos,
                [this](custom_msg::msg::Command::SharedPtr msg) {
                    this->commandSubCallback(msg);
                }
            );

            // Timer for decisional loop
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100), std::bind(&Controller::main_loop, this)
            );
        }
    private:
        // Status variables
        bool is_docked = true;
        std::string command = "";

        // Timer
        rclcpp::TimerBase::SharedPtr timer_;

        // Dock Subsciption
        rclcpp::Subscription<irobot_create_msgs::msg::DockStatus>::SharedPtr dockSubscription;
        void dockSubCallback(irobot_create_msgs::msg::DockStatus::SharedPtr msg) {
            is_docked = msg->is_docked;
        }

        // Command Subsciption
        rclcpp::Subscription<custom_msg::msg::Command>::SharedPtr commandSubscription;
        void commandSubCallback(custom_msg::msg::Command::SharedPtr msg) {
            command = msg->command;
        }


        // Main Loop
        void main_loop() {
            if(command != "") {
                if(command == "DOCK") {

                } else if(command == "UNDOCK") {

                } else if(command == "MODE A") {
                    
                } else if(command == "MODE B") {
                    
                } else if(command == "SOLVE") {

                } else if(command == "FORWARD") {
                    
                } else if(command == "BACKWARD") {
                    
                } else if(command == "ROTATE LEFT") {
                    
                } else if(command == "ROTATE RIGHT") {
                    
                } else if(command == "STOP") {
                    
                } else {
                    
                }
                
                command = "";
            }
        }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}