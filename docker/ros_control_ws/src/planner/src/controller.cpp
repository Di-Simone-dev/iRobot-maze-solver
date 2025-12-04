#include "rclcpp/rclcpp.hpp"
#include "irobot_create_msgs/msg/dock_status.hpp"
#include "custom_msg/action/actuator_dock.hpp"
#include "custom_msg/action/actuator_move.hpp"
#include "custom_msg/msg/command.hpp"
#include "custom_msg/srv/stop.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


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

            // Dock Actuator Client
            dockClient = rclcpp_action::create_client<custom_msg::action::ActuatorDock>(
                this,
                "actuator_dock"
            );

            // Dock Actuator Client
            movementClient = rclcpp_action::create_client<custom_msg::action::ActuatorMove>(
                this,
                "actuator_movement"
            );

            // Stop Actuator Client
            StopClient = this->create_client<custom_msg::srv::Stop>("actuator_power");

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
        int mode = 0;

        // Timer
        rclcpp::TimerBase::SharedPtr timer_;

        // Dock Subsciption
        rclcpp::Subscription<irobot_create_msgs::msg::DockStatus>::SharedPtr dockSubscription;
        void dockSubCallback(irobot_create_msgs::msg::DockStatus::SharedPtr msg) {
            is_docked = msg->is_docked;
        }

        // Dock Actuator client
        rclcpp_action::Client<custom_msg::action::ActuatorDock>::SharedPtr dockClient;

        // Movement Actuator Client
        rclcpp_action::Client<custom_msg::action::ActuatorMove>::SharedPtr movementClient;

        // Stop Actuator Client
        rclcpp::Client<custom_msg::srv::Stop>::SharedPtr StopClient;

        // Command Subsciption
        rclcpp::Subscription<custom_msg::msg::Command>::SharedPtr commandSubscription;
        void commandSubCallback(custom_msg::msg::Command::SharedPtr msg) {
            command = msg->command;
        }

        // Dock callback
        void send_actuator_dock_goal(const std::string &type)
        {
            using namespace std::placeholders;

            // Controllo input
            if (type != "DOCK" && type != "UNDOCK") {
                RCLCPP_ERROR(this->get_logger(), "Invalid type sent to actuator_dock: %s", type.c_str());
                return;
            }

            // Attesa server
            if (!dockClient->wait_for_action_server()) {
                RCLCPP_ERROR(this->get_logger(), "ActuatorDock server not available");
                return;
            }

            // Creazione messaggio goal
            custom_msg::action::ActuatorDock::Goal goal_msg;
            goal_msg.type = type;

            // Opzioni callback
            rclcpp_action::Client<custom_msg::action::ActuatorDock>::SendGoalOptions options;

            // GOAL response
            options.goal_response_callback =
                [this, type](auto handle)
                {
                    if (!handle) {
                        RCLCPP_ERROR(this->get_logger(), "[%s] Goal rejected", type.c_str());
                    } else {
                        RCLCPP_INFO(this->get_logger(), "[%s] Goal accepted", type.c_str());
                    }
                };

            // FEEDBACK 
            options.feedback_callback =
                [](auto, auto) {
                    
                };

            // RISULTATO
            options.result_callback =
                [this, type](const rclcpp_action::ClientGoalHandle<custom_msg::action::ActuatorDock>::WrappedResult &result)
                {
                    switch (result.code) {

                        case rclcpp_action::ResultCode::SUCCEEDED:
                            if (result.result->status)
                                RCLCPP_INFO(this->get_logger(), "[%s] COMPLETED SUCCESSFULLY", type.c_str());
                            else
                                RCLCPP_WARN(this->get_logger(), "[%s] Completed but status=false", type.c_str());
                            return;

                        case rclcpp_action::ResultCode::ABORTED:
                            RCLCPP_ERROR(this->get_logger(), "[%s] ABORTED", type.c_str());
                            return;

                        case rclcpp_action::ResultCode::CANCELED:
                            RCLCPP_ERROR(this->get_logger(), "[%s] CANCELED", type.c_str());
                            return;

                        default:
                            RCLCPP_ERROR(this->get_logger(), "[%s] UNKNOWN result code", type.c_str());
                            return;
                    }
                };

            // Invio del goal
            dockClient->async_send_goal(goal_msg, options);
        }

        // Movement callback
        void send_actuator_movement_goal(const std::string &type, float distance, float max_vel)
        {
            using namespace std::placeholders;

            // Controllo input
            if (type != "DISTANCE" && type != "ANGLE") {
                RCLCPP_ERROR(this->get_logger(), "Invalid type sent to actuator_dock: %s", type.c_str());
                return;
            }

            // Attesa server
            if (!movementClient->wait_for_action_server()) {
                RCLCPP_ERROR(this->get_logger(), "ActuatorMovement server not available");
                return;
            }

            // Creazione messaggio goal
            custom_msg::action::ActuatorMove::Goal goal_msg;
            goal_msg.type = type;
            goal_msg.distance = distance;
            goal_msg.max_speed = max_vel;

            // Opzioni callback
            rclcpp_action::Client<custom_msg::action::ActuatorMove>::SendGoalOptions options;

            // GOAL response
            options.goal_response_callback =
                [this, type](auto handle)
                {
                    if (!handle) {
                        RCLCPP_ERROR(this->get_logger(), "[%s] Goal rejected", type.c_str());
                    } else {
                        RCLCPP_INFO(this->get_logger(), "[%s] Goal accepted", type.c_str());
                    }
                };

            // FEEDBACK
            options.feedback_callback =
                [this, type](auto, const std::shared_ptr<const custom_msg::action::ActuatorMove::Feedback> feedback)
                {
                    // Aggiorna log o interfaccia con percentuale completamento
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 500,
                                        "[%s] Progress: %.2f%%", type.c_str(), feedback->percentage);
                };

            // RISULTATO
            options.result_callback =
                [this, type](const rclcpp_action::ClientGoalHandle<custom_msg::action::ActuatorMove>::WrappedResult &result)
                {
                    switch (result.code) {

                        case rclcpp_action::ResultCode::SUCCEEDED:
                            if (result.result->status)
                                RCLCPP_INFO(this->get_logger(), "[%s] COMPLETED SUCCESSFULLY", type.c_str());
                            else
                                RCLCPP_WARN(this->get_logger(), "[%s] Completed but status=false", type.c_str());
                            return;

                        case rclcpp_action::ResultCode::ABORTED:
                            RCLCPP_ERROR(this->get_logger(), "[%s] ABORTED", type.c_str());
                            return;

                        case rclcpp_action::ResultCode::CANCELED:
                            RCLCPP_ERROR(this->get_logger(), "[%s] CANCELED", type.c_str());
                            return;

                        default:
                            RCLCPP_ERROR(this->get_logger(), "[%s] UNKNOWN result code", type.c_str());
                            return;
                    }
                };

            // Invio del goal
            movementClient->async_send_goal(goal_msg, options);
        }

        // =======================
        // Send Stop / Start Command
        // =======================
        void send_actuator_stop(bool stop)
        {
            if (!StopClient->wait_for_service(std::chrono::seconds(2))) {
                RCLCPP_ERROR(this->get_logger(), "Stop service not available");
                return;
            }

            auto request = std::make_shared<custom_msg::srv::Stop::Request>();
            request->stop = stop;

            auto future_result = StopClient->async_send_request(request,
                [this, stop](rclcpp::Client<custom_msg::srv::Stop>::SharedFuture future) {
                    try {
                        auto response = future.get();
                        if (response->status) {
                            RCLCPP_INFO(this->get_logger(), "Actuator %s successfully", stop ? "stopped" : "started");
                        } else {
                            RCLCPP_WARN(this->get_logger(), "Actuator %s failed", stop ? "stop" : "start");
                        }
                    } catch (const std::exception &e) {
                        RCLCPP_ERROR(this->get_logger(), "Failed to call stop service: %s", e.what());
                    }
                }
            );
        }

        // Main Loop
        void main_loop() {
            if(command != "") {
                if(command == "DOCK") {
                    send_actuator_dock_goal("DOCK");
                } else if(command == "UNDOCK") {
                    send_actuator_dock_goal("UNDOCK");
                } else if(command == "MODE A") {
                    this->mode = 0;
                } else if(command == "MODE B") {
                    this->mode = 1;
                } else if(command == "SOLVE") {

                } else if(command == "FORWARD") {
                    send_actuator_movement_goal("DISTANCE", 1, 0.5);
                } else if(command == "BACKWARD") {
                    send_actuator_movement_goal("DISTANCE", -1, 0.5);
                } else if(command == "ROTATE LEFT") {
                    send_actuator_movement_goal("ANGLE", -1.57, 1);
                } else if(command == "ROTATE RIGHT") {
                    send_actuator_movement_goal("ANGLE", 1.57, 1);
                }else if(command == "START") {
                    send_actuator_stop(false);
                } else if(command == "STOP") {
                    send_actuator_stop(true);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Command unrecognized!");
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