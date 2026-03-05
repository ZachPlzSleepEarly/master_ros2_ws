#include "master_ros2_interface/action/my_custom_action.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class ActionServer : public rclcpp::Node {
public:
    using MyCustomAction = master_ros2_interface::action::MyCustomAction;
    using GoalHandle = rclcpp_action::ServerGoalHandle<MyCustomAction>;

    ActionServer() : rclcpp::Node(NODE_NAME)
    {
        action_server_ = rclcpp_action::create_server<MyCustomAction>(
            this, SERVER_NAME, std::bind(&ActionServer::HandleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ActionServer::HandleCancel, this, std::placeholders::_1),
            std::bind(&ActionServer::HandleAccepted, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Started Action Server Node");
    }

private:
    rclcpp_action::GoalResponse HandleGoal(const rclcpp_action::GoalUUID& uuid,
                                           std::shared_ptr<const MyCustomAction::Goal> goal);
    rclcpp_action::CancelResponse HandleCancel(const std::shared_ptr<GoalHandle> goal_handle);
    void HandleAccepted(const std::shared_ptr<GoalHandle> goal_handle);
    void Execute(const std::shared_ptr<GoalHandle> goal_handle);

    static const std::string NODE_NAME;
    const std::string SERVER_NAME = "my_custom_action";

    rclcpp_action::Server<MyCustomAction>::SharedPtr action_server_;
};

const std::string ActionServer::NODE_NAME = "actoin_server";

rclcpp_action::GoalResponse ActionServer::HandleGoal(const rclcpp_action::GoalUUID&,
                                                     std::shared_ptr<const MyCustomAction::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request with value '%d'", goal->goal_value);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ActionServer::HandleCancel(const std::shared_ptr<GoalHandle>)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ActionServer::HandleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
{
    std::thread{
        std::bind(&ActionServer::Execute, this, std::placeholders::_1),
        goal_handle
    }.detach();
}

void ActionServer::Execute(const std::shared_ptr<GoalHandle> goal_handle)
{
    const auto goal = goal_handle->get_goal();

    auto feedback_msg = std::make_shared<MyCustomAction::Feedback>();
    for (int i = 1; i <= goal->goal_value; ++i) {
        feedback_msg->progress = i;
        goal_handle->publish_feedback(feedback_msg);
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    auto result_msg = std::make_shared<MyCustomAction::Result>();
    result_msg->result_value = goal->goal_value;
    goal_handle->succeed(result_msg);

    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActionServer>());
    rclcpp::shutdown();
    return 0;
}