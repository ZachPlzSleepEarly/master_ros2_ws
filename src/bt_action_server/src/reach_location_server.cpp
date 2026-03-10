#include "bt_action_server/action/reach_location.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <random>

class ReachLocationActionServer : public rclcpp::Node {
public:
    using ReachLocation = bt_action_server::action::ReachLocation;
    using GoalHandleReachLocation = rclcpp_action::ServerGoalHandle<ReachLocation>;

    /**
     * @brief 防止隐式转换
     *
     */
    explicit ReachLocationActionServer() : rclcpp::Node(NODE_NAME)
    {
        action_server_ = rclcpp_action::create_server<ReachLocation>(
            this, SERVER_NAME,
            std::bind(&ReachLocationActionServer::HandleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ReachLocationActionServer::HandleCancel, this, std::placeholders::_1),
            std::bind(&ReachLocationActionServer::HandleAccepted, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Action server started.");
    }

private:
    rclcpp_action::GoalResponse HandleGoal(const rclcpp_action::GoalUUID&,
                                           std::shared_ptr<const ReachLocation::Goal> goal);
    rclcpp_action::CancelResponse HandleCancel(const std::shared_ptr<GoalHandleReachLocation>);
    void HandleAccepted(const std::shared_ptr<GoalHandleReachLocation> goal_handle);
    void Execute(const std::shared_ptr<GoalHandleReachLocation> goal_handle);

    static constexpr const char* NODE_NAME = "reach_location_actoin_server";
    static constexpr const char* SERVER_NAME = "reach_locatoin";

    rclcpp_action::Server<ReachLocation>::SharedPtr action_server_;
};

rclcpp_action::GoalResponse ReachLocationActionServer::HandleGoal(const rclcpp_action::GoalUUID&,
                                                                  std::shared_ptr<const ReachLocation::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Receive goal request for location (%f, %f) with timeout %f", goal->x, goal->y,
                goal->timeout);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ReachLocationActionServer::HandleCancel(const std::shared_ptr<GoalHandleReachLocation>)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ReachLocationActionServer::HandleAccepted(const std::shared_ptr<GoalHandleReachLocation> goal_handle)
{
    std::thread{std::bind(&ReachLocationActionServer::Execute, this, std::placeholders::_1), goal_handle}.detach();
}

void ReachLocationActionServer::Execute(const std::shared_ptr<GoalHandleReachLocation> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal ...");

    const auto goal = goal_handle->get_goal();
    float target_x = goal->x;
    float target_y = goal->y;
    float timeout = goal->timeout;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> velocity_dist(0.1, 0.2);

    float current_x = 0.0;
    float current_y = 0.0;
    auto feedback = std::make_shared<ReachLocation::Feedback>();
    auto result = std::make_shared<ReachLocation::Result>();
    rclcpp::Rate rate(1.0); // 1 Hz loop rate
    auto start_time = this->now();
    while ((this->now() - start_time).seconds() < timeout) {
        float velocity = velocity_dist(gen);

        current_x += velocity;
        current_y += velocity;

        feedback->current_x = current_x;
        feedback->current_y = current_y;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Current position: (%f, %f)", current_x, current_y);

        if (current_x >= target_x && current_y >= target_y) {
            result->success = true;
            result->message = "Target reached successfully";
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            return;
        }

        rate.sleep();
    }

    result->success = false;
    result->message = "Failed to reach the target within the given timeout.";
    goal_handle->abort(result);
    RCLCPP_INFO(this->get_logger(), "Goal failed");
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReachLocationActionServer>());
    rclcpp::shutdown();
    return 0;
}