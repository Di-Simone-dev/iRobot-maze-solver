#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "irobot_create_msgs/msg/dock_status.hpp"
#include "irobot_create_msgs/action/drive_distance.hpp"
#include "irobot_create_msgs/action/rotate_angle.hpp"
#include "irobot_create_msgs/srv/e_stop.hpp"
#include "custom_msg/msg/command.hpp"


class Actuator : public rclcpp::Node {
    public:
        Actuator(): Node("actuator") {

            // Drive distance client
            driveDistanceClient = rclcpp_action::create_client<irobot_create_msgs::action::DriveDistance>(
                this,
                "drive_distance"
            );

            // Rotate angle client
            rotateAngleClient = rclcpp_action::create_client<irobot_create_msgs::action::RotateAngle>(
                this,
                "rotate_angle"
            );

            // Estop client
            eStopClient = this->create_client<irobot_create_msgs::srv::EStop>("e_stop");
        }

        // Callbacks
        // Drive distance goal
        void send_drive_distance_goal(float distance, float max_speed) {
            if (!driveDistanceClient->wait_for_action_server()) {
                RCLCPP_ERROR(this->get_logger(), "Drive distance action server not available");
                return;
            }

            // Creating goal message
            auto goal_msg = irobot_create_msgs::action::DriveDistance::Goal();
            goal_msg.distance = distance;
            goal_msg.max_translation_speed = max_speed;

            // Opzioni per callback
            auto options = rclcpp_action::Client<irobot_create_msgs::action::DriveDistance>::SendGoalOptions();

            options.goal_response_callback =
            [this](rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::DriveDistance>::SharedPtr goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Drive distance goal refused");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Drive distance goal accepted");
                }
            };

            options.feedback_callback =
            [this](rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::DriveDistance>::SharedPtr, const std::shared_ptr<const irobot_create_msgs::action::DriveDistance::Feedback> feedback) {
                RCLCPP_INFO(this->get_logger(), "Feedback: remaining distance %.2f m", feedback->remaining_travel_distance);
            };

            options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::DriveDistance>::WrappedResult & result) {
                switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "Goal completed");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Goal aborted");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(this->get_logger(), "Goal cancelled");
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown result");
                    break;
                }
            };

            driveDistanceClient->async_send_goal(goal_msg, options);

        }

        // Rotate angle goal
        void send_rotate_angle_goal(float angle, float max_speed) {
            
            if (!driveDistanceClient->wait_for_action_server()) {
                RCLCPP_ERROR(this->get_logger(), "Rotate angle action server not available");
                return;
            }

            // Creating goal message
            auto goal_msg = irobot_create_msgs::action::RotateAngle::Goal();
            goal_msg.angle = angle;
            goal_msg.max_rotation_speed = max_speed;


            // Opzioni per callback
            auto options = rclcpp_action::Client<irobot_create_msgs::action::RotateAngle>::SendGoalOptions();

            options.goal_response_callback =
            [this](rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::RotateAngle>::SharedPtr goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Rotate angle goal refused");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Rotate angle goal accepted");
                }
            };

            options.feedback_callback =
            [this](rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::RotateAngle>::SharedPtr, const std::shared_ptr<const irobot_create_msgs::action::RotateAngle::Feedback> feedback) {
                RCLCPP_INFO(this->get_logger(), "Feedback: remaining angle %.2f m", feedback->remaining_angle_travel);
            };

            options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::RotateAngle>::WrappedResult & result) {
                switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "Goal completed");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Goal aborted");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(this->get_logger(), "Goal cancelled");
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown result");
                    break;
                }
            };

            rotateAngleClient->async_send_goal(goal_msg, options);
        }

        // EStop request
        void send_request(bool stop) {
            // Attende che il server sia disponibile
            while (!eStopClient->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted");
                return;
            }
                RCLCPP_INFO(this->get_logger(), "Waiting e_stop service...");
            }

            // Crea la richiesta
            auto request = std::make_shared<irobot_create_msgs::srv::EStop::Request>();
            request->e_stop_on = stop;

            // Invia la richiesta asincrona
            auto future = eStopClient->async_send_request(request);

            // Attende la risposta
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "Response: success=%s", response->success ? "true" : "false");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Error during EStop calling");
            }
        };
        
    private:
        // Subscription
        rclcpp_action::Client<irobot_create_msgs::action::DriveDistance>::SharedPtr driveDistanceClient;
        rclcpp_action::Client<irobot_create_msgs::action::RotateAngle>::SharedPtr rotateAngleClient;
        rclcpp::Client<irobot_create_msgs::srv::EStop>::SharedPtr eStopClient;
    };

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Actuator>());
  rclcpp::shutdown();
  return 0;
}