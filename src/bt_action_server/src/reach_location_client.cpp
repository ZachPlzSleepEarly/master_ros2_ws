#include "bt_action_server/action/reach_location.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace ReachLocationClient {
using Action = bt_action_server::action::ReachLocation;

bool done_ = false;

static constexpr const char* NODE_NAME = "reach_location_action_client";
static constexpr const char* SERVER_NAME = "reach_location";

void ResultCallback(const rclcpp_action::ClientGoalHandle<Action>::WrappedResult& result)
{
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(rclcpp::get_logger(NODE_NAME), "Goal succeeded!");
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(rclcpp::get_logger(NODE_NAME), "Goal was aborted");
        break;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(rclcpp::get_logger(NODE_NAME), "Goal was canceled");
        break;
    default:
        RCLCPP_ERROR(rclcpp::get_logger(NODE_NAME), "Unknown result code");
        break;
    }

    done_ = true;
}
} // namespace ReachLocationClient

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared(ReachLocationClient::NODE_NAME);

    // 创建 Hello action 客户端
    auto client = rclcpp_action::create_client<ReachLocationClient::Action>(node, ReachLocationClient::SERVER_NAME);

    // 等待 action server to become available
    if (!client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
        return -1;
    }

    // 创建 Goal message
    auto goal = ReachLocationClient::Action::Goal();
    goal.x = 4;
    goal.y = 4;
    goal.timeout = 100;
    RCLCPP_INFO(node->get_logger(), "Goal: x='%f' y='%f' timeout='%f'", goal.x, goal.y, goal.timeout);

    // 发送 Goal，等待结果
    auto send_goal_future = client->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(node, send_goal_future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to send goal");
        return -1;
    }

    auto goal_handle = send_goal_future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
        return -1;
    }

    std::cout << "Acquired!" << std::endl;
    client->async_get_result(goal_handle, ReachLocationClient::ResultCallback);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}