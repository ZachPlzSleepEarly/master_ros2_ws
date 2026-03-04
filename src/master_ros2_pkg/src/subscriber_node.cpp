#include "master_ros2_interface/msg/custom_msg.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SubscriberNode : public rclcpp::Node {
public:
    SubscriberNode() : rclcpp::Node(NODE_NAME)
    {
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(QOS_KEEPLAST_DEPTH)).reliable().transient_local();

        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            STD_SUB_TOPIC_NAME, qos_profile, std::bind(&SubscriberNode::StringCallback, this, std::placeholders::_1));

        custom_subscriber_ = this->create_subscription<master_ros2_interface::msg::CustomMsg>(
            CUSTOM_SUB_TOPIC_NAME, qos_profile,
            std::bind(&SubscriberNode::CustomCallback, this, std::placeholders::_1));
    }

private:
    void StringCallback(const std_msgs::msg::String::ConstSharedPtr& msg);
    void CustomCallback(const master_ros2_interface::msg::CustomMsg::ConstSharedPtr& msg);

    static const std::string NODE_NAME;
    const std::string STD_SUB_TOPIC_NAME = "std_string_topic";
    const std::string CUSTOM_SUB_TOPIC_NAME = "custom_topic";
    const int QOS_KEEPLAST_DEPTH = 10;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    rclcpp::Subscription<master_ros2_interface::msg::CustomMsg>::SharedPtr custom_subscriber_;
};

const std::string SubscriberNode::NODE_NAME = "subscriber_node";

void SubscriberNode::StringCallback(const std_msgs::msg::String::ConstSharedPtr& msg)
{
    RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
}

void SubscriberNode::CustomCallback(const master_ros2_interface::msg::CustomMsg::ConstSharedPtr& msg)
{
    RCLCPP_INFO(this->get_logger(), "Received custom message: data='%s', number='%d'", msg->data.c_str(), msg->number);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubscriberNode>());
    rclcpp::shutdown();
    return 0;
}