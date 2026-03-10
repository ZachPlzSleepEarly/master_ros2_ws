#include "bt_action_server/action/reach_location.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp/behavior_tree.h> // for StatefulActionNode
#include <behaviortree_cpp/bt_factory.h>    // for Tree

namespace BTActionServer {
using Action = bt_action_server::action::ReachLocation;

static constexpr const char* BLACKBOARD_NODE = "node";
static constexpr const char* SERVER_NAME = "server_name";

enum class ActionResult : uint8_t { ActionNotCompleted, ActionFailed, ActionCanceled, ActionSucceed };

class WaitForServer : public BT::StatefulActionNode {
public:
    WaitForServer(const std::string& action_name, const BT::NodeConfig& conf)
        : BT::StatefulActionNode(action_name, conf)
    {
        node_ = conf.blackboard->get<rclcpp::Node::SharedPtr>(BLACKBOARD_NODE);
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    static BT::PortsList providedPorts();

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<Action>::SharedPtr client_;
};

BT::NodeStatus WaitForServer::onStart()
{
    std::string server_name;
    getInput<std::string>(SERVER_NAME, server_name);
    client_ = rclcpp_action::create_client<Action>(node_, server_name);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForServer::onRunning()
{
    if (!client_->action_server_is_ready()) {
        std::cout << "[Node WaitForServer]: Failure" << std::endl;
        return BT::NodeStatus::FAILURE;
    } else {
        std::cout << "[Node WaitForServer]: Success" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
}

void WaitForServer::onHalted()
{
    return;
}

BT::PortsList WaitForServer::providedPorts()
{
    return {BT::InputPort<std::string>(SERVER_NAME)};
}

class CallAction : public BT::StatefulActionNode {
public:
    CallAction(const std::string& action_name, const BT::NodeConfig& conf)
        : BT::StatefulActionNode(action_name, conf), server_called_(false)
    {
        node_ = rclcpp::Node::make_shared(NODE_NAME);
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void ResultCallback(const rclcpp_action::ClientGoalHandle<Action>::WrappedResult& result);
    void onHalted() override;
    static BT::PortsList providedPorts();

private:
    static constexpr const char* NODE_NAME = "action_client_node";
    static constexpr const char* INPUT_X = "x";
    static constexpr const char* INPUT_Y = "y";
    static constexpr const char* INPUT_TIMEOUT = "timeout";

    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<Action>::SharedPtr client_;

    float x_;
    float y_;
    float timeout_;
    bool server_called_;

    ActionResult action_result_;
};

BT::NodeStatus CallAction::onStart()
{
    std::string server_name;
    getInput<std::string>(SERVER_NAME, server_name);
    getInput<float>(INPUT_X, x_);
    getInput<float>(INPUT_Y, y_);
    getInput<float>(INPUT_TIMEOUT, timeout_);
    action_result_ = ActionResult::ActionNotCompleted;

    client_ = rclcpp_action::create_client<Action>(node_, server_name);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CallAction::onRunning()
{
    // 确保 Server 已经被调用
    if (!server_called_) {
        server_called_ = true;
        auto goal = Action::Goal();
        goal.x = x_;
        goal.y = y_;
        goal.timeout = timeout_;

        auto send_goal_future = client_->async_send_goal(goal);

        if (rclcpp::spin_until_future_complete(node_, send_goal_future) != rclcpp::FutureReturnCode::SUCCESS) {
            std::cout << "Failed to send goal" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        auto goal_handle = send_goal_future.get();
        if (!goal_handle) {
            std::cout << "Goal was rejected by server" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        client_->async_get_result(goal_handle, std::bind(&CallAction::ResultCallback, this, std::placeholders::_1));
    }

    rclcpp::spin(node_);

    if (action_result_ == ActionResult::ActionSucceed) {
        std::cout << "[Node CallAction]: Success" << std::endl;
        return BT::NodeStatus::SUCCESS;
    } else if (action_result_ == ActionResult::ActionFailed) {
        std::cout << "[Node CallAction]: Failure" << std::endl;
        return BT::NodeStatus::FAILURE;
    } else if (action_result_ == ActionResult::ActionCanceled) {
        std::cout << "[Node CallAction]: Failure" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[Node CallAction]: Waiting server execution" << std::endl;
    return BT::NodeStatus::RUNNING;
}

void CallAction::ResultCallback(const rclcpp_action::ClientGoalHandle<Action>::WrappedResult& result)
{
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
        action_result_ = ActionResult::ActionSucceed;
        break;
    case rclcpp_action::ResultCode::ABORTED:
        action_result_ = ActionResult::ActionFailed;
        break;
    case rclcpp_action::ResultCode::CANCELED:
        action_result_ = ActionResult::ActionCanceled;
    default:
        break;
    }
}

void CallAction::onHalted()
{
    return;
}

BT::PortsList CallAction::providedPorts()
{
    return {
        BT::InputPort<std::string>(SERVER_NAME),
        BT::InputPort<float>(INPUT_X),
        BT::InputPort<float>(INPUT_Y),
        BT::InputPort<float>(INPUT_TIMEOUT),
    };
}

using namespace std::chrono_literals;
class BTExecutor : public rclcpp::Node {
public:
    BTExecutor() : rclcpp::Node(NODE_NAME), first_(true)
    {
        timer_ = this->create_wall_timer(0.5s, std::bind(&BTExecutor::TickFunction, this));
        blackboard_ = BT::Blackboard::create();
    }

private:
    void TickFunction();
    void InitBTree();

    static constexpr const char* NODE_NAME = "bt_executor";
    static constexpr const char* TREE_XML_FILE = "tree_xml_file";
    static constexpr const char* BT_NODE_WAIT_FOR_SERVER = "WaitForServer";
    static constexpr const char* BT_NODE_CALL_ACTION = "CallAction";

    bool first_;

    rclcpp::TimerBase::SharedPtr timer_;

    BT::Blackboard::Ptr blackboard_;
    BT::Tree tree_;
    BT::BehaviorTreeFactory factory_;
};

void BTExecutor::InitBTree()
{
    blackboard_->set(BLACKBOARD_NODE, this->shared_from_this());

    factory_.registerNodeType<WaitForServer>(BT_NODE_WAIT_FOR_SERVER);
    factory_.registerNodeType<CallAction>(BT_NODE_CALL_ACTION);
    this->declare_parameter(TREE_XML_FILE, "");
    std::string tree_file;
    this->get_parameter(TREE_XML_FILE, tree_file);
    tree_ = factory_.createTreeFromFile(tree_file, blackboard_);
}

void BTExecutor::TickFunction()
{
    if (first_) {
        InitBTree();
        first_ = false;
    }
    tree_.tickOnce();
}

} // namespace BTActionServer

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BTActionServer::BTExecutor>());
    rclcpp::shutdown();
    return 0;
}