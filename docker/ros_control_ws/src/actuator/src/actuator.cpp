#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "irobot_create_msgs/action/drive_distance.hpp"
#include "irobot_create_msgs/action/rotate_angle.hpp"
#include "irobot_create_msgs/srv/e_stop.hpp"

#include "custom_msg/action/actuator_move.hpp"
#include "custom_msg/srv/stop.hpp"


class Actuator : public rclcpp::Node {
public:
    Actuator() : Node("actuator")
    {
        using namespace std::placeholders;

        driveDistanceClient = rclcpp_action::create_client<irobot_create_msgs::action::DriveDistance>(
            this, "drive_distance");

        rotateAngleClient = rclcpp_action::create_client<irobot_create_msgs::action::RotateAngle>(
            this, "rotate_angle");

        eStopClient = this->create_client<irobot_create_msgs::srv::EStop>("e_stop");

        movementServer = rclcpp_action::create_server<custom_msg::action::ActuatorMove>(
            this,
            "actuator_movement",
            std::bind(&Actuator::handle_solve_goal, this, _1, _2),
            std::bind(&Actuator::handle_solve_cancel, this, _1),
            std::bind(&Actuator::handle_solve_accepted, this, _1)
        );

        stopServer = this->create_service<custom_msg::srv::Stop>(
            "actuator_power",
            std::bind(&Actuator::handle_stop_request, this, std::placeholders::_1, std::placeholders::_2)
        );
    }

private:
    // --------------------------
    // Client & Server handles
    // --------------------------
    rclcpp_action::Client<irobot_create_msgs::action::DriveDistance>::SharedPtr driveDistanceClient;
    rclcpp_action::Client<irobot_create_msgs::action::RotateAngle>::SharedPtr rotateAngleClient;
    rclcpp::Client<irobot_create_msgs::srv::EStop>::SharedPtr eStopClient;

    rclcpp_action::Server<custom_msg::action::ActuatorMove>::SharedPtr movementServer;
    rclcpp::Service<custom_msg::srv::Stop>::SharedPtr stopServer;

    // ======================
    //      Handle stop
    // ======================
    void handle_stop_request(
        const std::shared_ptr<custom_msg::srv::Stop::Request> request,
        std::shared_ptr<custom_msg::srv::Stop::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Stop request received: %s", request->stop ? "true" : "false");

        // Send stop request
        send_request(request->stop);

        // Risposta al client
        response->status = true;
    }

    //EStop request
    void send_request(bool stop) { 
        // Waiting client
        while (!eStopClient->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting e_stop service...");
        }
        
        // Creating request
        auto request = std::make_shared<irobot_create_msgs::srv::EStop::Request>();
        request->e_stop_on = stop;
        
        // Sending request
        auto future = eStopClient->async_send_request(request);
        
        // waiting response
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Response: success=%s", response->success ? "true" : "false");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Error during EStop calling");
        }
    };


    // Runtime state
    float last_drive_remaining = std::numeric_limits<float>::infinity();
    float last_angle_remaining = std::numeric_limits<float>::infinity();
    int drive_status = -1;
    int angle_status = -1;

    // --------------------------
    // Action Server Callbacks
    // --------------------------
    rclcpp_action::GoalResponse handle_solve_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const custom_msg::action::ActuatorMove::Goal> goal)
    {
        if (goal->type != "DISTANCE" && goal->type != "ANGLE") {
            RCLCPP_ERROR(get_logger(), "Invalid goal type: %s", goal->type.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_solve_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msg::action::ActuatorMove>>)
    {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_solve_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msg::action::ActuatorMove>> goal_handle)
    {
        std::thread(&Actuator::execute_solve, this, goal_handle).detach();
    }

    // --------------------------
    // EXECUTION LOGIC
    // --------------------------
    void execute_solve(const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msg::action::ActuatorMove>> goal_handle)
    {
        const auto goal = goal_handle->get_goal();

        auto feedback = std::make_shared<custom_msg::action::ActuatorMove::Feedback>();
        auto result   = std::make_shared<custom_msg::action::ActuatorMove::Result>();

        reset_state();
        publish_initial_feedback(goal_handle, feedback);

        // ============================
        //        DISTANCE MODE
        // ============================
        if (goal->type == "DISTANCE")
        {
            if (!create3_begin_drive(goal->distance, goal->max_speed, result, goal_handle))
                return;

            monitor_progress_distance(goal_handle, goal, feedback, result);
            return;
        }

        // ============================
        //         ANGLE MODE
        // ============================
        if (goal->type == "ANGLE")
        {
            if (!create3_begin_angle(goal->distance, goal->max_speed, result, goal_handle))
                return;

            monitor_progress_angle(goal_handle, goal, feedback, result);
            return;
        }
    }

    // --------------------------
    // TOTEM: Reset State
    // --------------------------
    void reset_state()
    {
        drive_status = -1;
        angle_status = -1;
        last_drive_remaining = std::numeric_limits<float>::infinity();
        last_angle_remaining = std::numeric_limits<float>::infinity();
    }

    // --------------------------
    // Ensure feedback starts at 0%
    // --------------------------
    void publish_initial_feedback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msg::action::ActuatorMove>> &goal_handle,
        std::shared_ptr<custom_msg::action::ActuatorMove::Feedback> &feedback)
    {
        feedback->percentage = 0.0f;
        goal_handle->publish_feedback(feedback);
    }

    // ---------------------------------------------
    // SEND DRIVE DISTANCE GOAL
    // ---------------------------------------------
    bool create3_begin_drive(float distance, float speed,
                     std::shared_ptr<custom_msg::action::ActuatorMove::Result> &result,
                     const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msg::action::ActuatorMove>> &goal_handle)
    {
        if (distance == 0.0f) {
            result->status = true;
            goal_handle->succeed(result);
            return false;
        }

        if (!driveDistanceClient->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(get_logger(), "DriveDistance server unreachable");
            result->status = false;
            goal_handle->abort(result);
            return false;
        }

        irobot_create_msgs::action::DriveDistance::Goal goal_msg;
        goal_msg.distance = distance;
        goal_msg.max_translation_speed = speed;

        auto options = rclcpp_action::Client<irobot_create_msgs::action::DriveDistance>::SendGoalOptions();

        options.feedback_callback =
            [this](auto, const std::shared_ptr<const irobot_create_msgs::action::DriveDistance::Feedback> fb)
        {
            last_drive_remaining = fb->remaining_travel_distance;
        };

        options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::DriveDistance>::WrappedResult & res)
        {
            drive_status = static_cast<int>(res.code);
        };

        driveDistanceClient->async_send_goal(goal_msg, options);
        return true;
    }

    // ---------------------------------------------
    // SEND ROTATE ANGLE GOAL
    // ---------------------------------------------
    bool create3_begin_angle(float angle, float speed,
                     std::shared_ptr<custom_msg::action::ActuatorMove::Result> &result,
                     const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msg::action::ActuatorMove>> &goal_handle)
    {
        if (angle == 0.0f) {
            result->status = true;
            goal_handle->succeed(result);
            return false;
        }

        if (!rotateAngleClient->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(get_logger(), "RotateAngle server unreachable");
            result->status = false;
            goal_handle->abort(result);
            return false;
        }

        irobot_create_msgs::action::RotateAngle::Goal goal_msg;
        goal_msg.angle = angle;
        goal_msg.max_rotation_speed = speed;

        auto options = rclcpp_action::Client<irobot_create_msgs::action::RotateAngle>::SendGoalOptions();

        options.feedback_callback =
            [this](auto, const std::shared_ptr<const irobot_create_msgs::action::RotateAngle::Feedback> fb)
        {
            last_angle_remaining = fb->remaining_angle_travel;
        };

        options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::RotateAngle>::WrappedResult & res)
        {
            angle_status = static_cast<int>(res.code);
        };

        rotateAngleClient->async_send_goal(goal_msg, options);
        return true;
    }

    // ---------------------------------------------
    // DRIVE PROGRESS MONITOR
    // ---------------------------------------------
    void monitor_progress_distance(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msg::action::ActuatorMove>> &goal_handle,
        const std::shared_ptr<const custom_msg::action::ActuatorMove::Goal> &goal,
        std::shared_ptr<custom_msg::action::ActuatorMove::Feedback> &feedback,
        std::shared_ptr<custom_msg::action::ActuatorMove::Result> &result)
    {
        rclcpp::Rate rate(20);
        auto start = now();

        while (rclcpp::ok())
        {
            if (goal_handle->is_canceling()) {
                result->status = false;
                goal_handle->canceled(result);
                return;
            }

            if (!std::isinf(last_drive_remaining)) {
                float perc = (goal->distance - last_drive_remaining) * 100.0f / goal->distance;
                feedback->percentage = std::clamp(perc, 0.0f, 100.0f);
                goal_handle->publish_feedback(feedback);
            }

            if (drive_status != -1)
                break;

            if ((now() - start).seconds() > 20.0) {
                RCLCPP_ERROR(get_logger(), "Drive distance TIMEOUT");
                result->status = false;
                goal_handle->abort(result);
                return;
            }

            rate.sleep();
        }

        result->status = (drive_status == 0);
        if (result->status) goal_handle->succeed(result);
        else goal_handle->abort(result);
    }

    // ---------------------------------------------
    // ANGLE PROGRESS MONITOR
    // ---------------------------------------------
    void monitor_progress_angle(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msg::action::ActuatorMove>> &goal_handle,
        const std::shared_ptr<const custom_msg::action::ActuatorMove::Goal> &goal,
        std::shared_ptr<custom_msg::action::ActuatorMove::Feedback> &feedback,
        std::shared_ptr<custom_msg::action::ActuatorMove::Result> &result)
    {
        rclcpp::Rate rate(20);
        auto start = now();

        while (rclcpp::ok())
        {
            if (goal_handle->is_canceling()) {
                result->status = false;
                goal_handle->canceled(result);
                return;
            }

            if (!std::isinf(last_angle_remaining)) {
                float perc = (goal->distance - last_angle_remaining) * 100.0f / goal->distance;
                feedback->percentage = std::clamp(perc, 0.0f, 100.0f);
                goal_handle->publish_feedback(feedback);
            }

            if (angle_status != -1)
                break;

            if ((now() - start).seconds() > 20.0) {
                RCLCPP_ERROR(get_logger(), "Rotate angle TIMEOUT");
                result->status = false;
                goal_handle->abort(result);
                return;
            }

            rate.sleep();
        }

        result->status = (angle_status == 0);
        if (result->status) goal_handle->succeed(result);
        else goal_handle->abort(result);
    }
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Actuator>());
    rclcpp::shutdown();
    return 0;
}
