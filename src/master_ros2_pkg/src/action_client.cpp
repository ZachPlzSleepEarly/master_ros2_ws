#include "master_ros2_interface/action/my_custom_action.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class ActionClient : public rclcpp::Node {
public:
    using MyCustomAction = master_ros2_interface::action::MyCustomAction;
    using GoalHandle = rclcpp_action::ClientGoalHandle<MyCustomAction>;

    ActionClient() : rclcpp::Node(NODE_NAME)
    {
        client_ = rclcpp_action::create_client<MyCustomAction>(this, SERVICE_NAME);
        SendGoal();

        RCLCPP_INFO(this->get_logger(), "Started Action Client Node");
    }

    void SendGoal();

private:
    static const std::string NODE_NAME;
    const std::string SERVICE_NAME = "my_custom_action";

    rclcpp_action::Client<MyCustomAction>::SharedPtr client_;
};

const std::string ActionClient::NODE_NAME = "action_client";

void ActionClient::SendGoal()
{
    if (!client_->wait_for_action_server()) {
        RCLCPP_INFO(this->get_logger(), "Action Server not available");
        return;
    }

    auto goal_msg = MyCustomAction::Goal();
    goal_msg.goal_value = 10;

    auto send_goal_options = rclcpp_action::Client<MyCustomAction>::SendGoalOptions();
    send_goal_options.feedback_callback = [this](GoalHandle::SharedPtr,
                                                 const std::shared_ptr<const MyCustomAction::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "Feedback: %.2f", feedback->progress);
    };
    send_goal_options.result_callback = [this](const GoalHandle::WrappedResult& result) {
        RCLCPP_INFO(this->get_logger(), "Result: %d", result.result->result_value);
    };

    client_->async_send_goal(goal_msg, send_goal_options);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActionClient>());
    rclcpp::shutdown();
    return 0;
}