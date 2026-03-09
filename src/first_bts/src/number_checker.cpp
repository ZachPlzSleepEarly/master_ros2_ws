#include <random>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <behaviortree_cpp/behavior_tree.h> // for StatefulActionNode
#include <behaviortree_cpp/bt_factory.h>    // for Tree
#include <std_msgs/msg/int32.hpp>

namespace NumberChecker {
static constexpr const char* KEY_BLACKBOARD = "node";
static constexpr const char* BLACKBOARD_GENERATED_NUMBER = "generated_number";
static constexpr const char* PARAM_TREE_XML_FILE = "tree_xml_file";
static constexpr int QOS = 1;

class PublishResult : public BT::StatefulActionNode {
public:
    PublishResult(const std::string& action_name, const BT::NodeConfig& conf)
        : BT::StatefulActionNode(action_name, conf)
    {
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    static BT::PortsList providedPorts();

    static constexpr const char* NODE_NAME = "PublishResult";

private:
    static constexpr const char* PUBRESULT_IN_TOPIC_NAME = "topic_name";
    static constexpr const char* PUBRESULT_IN_GENERATED_NUMBER = BLACKBOARD_GENERATED_NUMBER;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
};

BT::NodeStatus PublishResult::onStart()
{
    // BehaviorTree.CPP 官方建议不要在 constructor 里访问 blackboard。
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>(KEY_BLACKBOARD);

    std::string topic_name;
    getInput<std::string>(PUBRESULT_IN_TOPIC_NAME, topic_name);
    pub_ = node_->create_publisher<std_msgs::msg::Int32>(topic_name, QOS);

    RCLCPP_INFO(rclcpp::get_logger(NODE_NAME), "PublishResult::onStart success");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PublishResult::onRunning()
{
    int value;
    getInput<int>(PUBRESULT_IN_GENERATED_NUMBER, value);
    std_msgs::msg::Int32 msg;
    msg.data = value;
    pub_->publish(msg);
    return BT::NodeStatus::SUCCESS;
}

void PublishResult::onHalted()
{
    return;
}

BT::PortsList PublishResult::providedPorts()
{
    return {BT::InputPort<std::string>(PUBRESULT_IN_TOPIC_NAME), BT::InputPort<int>(PUBRESULT_IN_GENERATED_NUMBER)};
}

class NumberChecker : public BT::StatefulActionNode {
public:
    NumberChecker(const std::string& action_name, const BT::NodeConfig& conf)
        : BT::StatefulActionNode(action_name, conf)
    {
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    static BT::PortsList providedPorts();

private:
    static constexpr const char* NODE_NAME = "NumberChecker";
    static constexpr const char* CHECKNUMBER_IN_CHECK_VALUE = "check_value";
    static constexpr const char* CHECKNUMBER_OUT_GENERATED_NUMBER = BLACKBOARD_GENERATED_NUMBER;
    static constexpr int LEFT_BOUND = 1;
    static constexpr int RIGHT_BOUND = 100;
    static constexpr int OUTPUT_FAILURE = -1;

    int num_threshold_;
};

BT::NodeStatus NumberChecker::onStart()
{
    if (!getInput<int>(CHECKNUMBER_IN_CHECK_VALUE, num_threshold_)) {
        throw BT::RuntimeError("missing required input [goal]");
    }

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NumberChecker::onRunning()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(LEFT_BOUND, RIGHT_BOUND);
    int random_number = distrib(gen);

    std::string log_msg = "Generated number: " + std::to_string(random_number)
        + " - Threshold: " + std::to_string(num_threshold_) + " - ";
    if (random_number < num_threshold_) {
        setOutput(CHECKNUMBER_OUT_GENERATED_NUMBER, random_number);
        log_msg = log_msg + "Success!";
        RCLCPP_INFO(rclcpp::get_logger("NumberChecker"), "%s", log_msg.c_str());
        return BT::NodeStatus::SUCCESS;
    } else {
        setOutput(CHECKNUMBER_OUT_GENERATED_NUMBER, OUTPUT_FAILURE);
        log_msg = log_msg + "Failure!";
        RCLCPP_INFO(rclcpp::get_logger(NODE_NAME), "%s", log_msg.c_str());
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
}

void NumberChecker::onHalted()
{
    return;
}

BT::PortsList NumberChecker::providedPorts()
{
    return {BT::InputPort<int>(CHECKNUMBER_IN_CHECK_VALUE), BT::OutputPort<int>(CHECKNUMBER_OUT_GENERATED_NUMBER)};
}

using namespace std::chrono_literals;
class BTExecutor : public rclcpp::Node {
public:
    BTExecutor() : rclcpp::Node(NODE_NAME), is_initialization_needed_(true)
    {
        timer_ = this->create_wall_timer(TICK_PERIOD, std::bind(&BTExecutor::TickFunction, this));
        blackboard_ = BT::Blackboard::create();
    }

private:
    void TickFunction();
    void InitBTree();

    static constexpr const char* CHECKNUMBER_1 = "CheckNumber1";
    static constexpr const char* CHECKNUMBER_2 = "CheckNumber2";
    static constexpr const char* CHECKNUMBER_3 = "CheckNumber3";
    static constexpr const char* PUBLISHRESULT = PublishResult::NODE_NAME;
    static constexpr const char* NODE_NAME = "bt_executor";
    static constexpr auto TICK_PERIOD = 0.5s;

    bool is_initialization_needed_;

    rclcpp::TimerBase::SharedPtr timer_;

    BT::Blackboard::Ptr blackboard_;
    BT::Tree tree_;
    BT::BehaviorTreeFactory factory_;
};

void BTExecutor::InitBTree()
{
    blackboard_->set<rclcpp::Node::SharedPtr>(KEY_BLACKBOARD, this->shared_from_this());

    factory_.registerNodeType<NumberChecker>(CHECKNUMBER_1);
    factory_.registerNodeType<NumberChecker>(CHECKNUMBER_2);
    factory_.registerNodeType<NumberChecker>(CHECKNUMBER_3);
    factory_.registerNodeType<PublishResult>(PUBLISHRESULT);

    this->declare_parameter<std::string>(PARAM_TREE_XML_FILE, "");
    std::string tree_file;
    this->get_parameter(PARAM_TREE_XML_FILE, tree_file);
    tree_ = factory_.createTreeFromFile(tree_file, blackboard_);
}

void BTExecutor::TickFunction()
{
    if (is_initialization_needed_) {
        InitBTree();
        is_initialization_needed_ = false;
    }

    tree_.tickOnce();
}
} // namespace NumberChecker

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NumberChecker::BTExecutor>());
    rclcpp::shutdown();
    return 0;
}