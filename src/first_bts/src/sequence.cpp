#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>

class HelloNode : public BT::StatefulActionNode {
public:
    HelloNode(const std::string& action_name, const BT::NodeConfig& conf) : BT::StatefulActionNode(action_name, conf)
    {
        ;
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    static BT::PortsList providedPorts();

private:
    static constexpr const char* INPUT_MSG = "msg";

    std::string hello_msg_;
};

BT::NodeStatus HelloNode::onStart()
{
    getInput(INPUT_MSG, hello_msg_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus HelloNode::onRunning()
{
    std::cout << hello_msg_ << std::endl;
    return BT::NodeStatus::SUCCESS;
}

void HelloNode::onHalted()
{
    return;
}

BT::PortsList HelloNode::providedPorts()
{
    return {BT::InputPort<std::string>(INPUT_MSG)};
}

using namespace std::chrono_literals;
class BTExecutor : public rclcpp::Node {
public:
    BTExecutor() : rclcpp::Node(NODE_NAME)
    {
        InitBTree();
        timer_ = this->create_wall_timer(TICK_PERIOD, std::bind(&BTExecutor::TickFunction, this));
    }

private:
    void InitBTree();
    void TickFunction();

    static constexpr const char* NODE_NAME = "bt_executor";
    static constexpr const char* PARAM_TREE_FILE = "tree_xml_file";
    static constexpr auto TICK_PERIOD = 0.5s;

    rclcpp::TimerBase::SharedPtr timer_;
    BT::BehaviorTreeFactory factory_;
    BT::Tree tree_;
};

void BTExecutor::InitBTree()
{
    factory_.registerNodeType<HelloNode>("HelloNode1");
    factory_.registerNodeType<HelloNode>("HelloNode2");
    this->declare_parameter<std::string>(PARAM_TREE_FILE, "");
    std::string tree_file;
    this->get_parameter(PARAM_TREE_FILE, tree_file);
    tree_ = factory_.createTreeFromFile(tree_file);
}

void BTExecutor::TickFunction()
{
    tree_.tickOnce();
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BTExecutor>());
    rclcpp::shutdown();
    return 0;
}