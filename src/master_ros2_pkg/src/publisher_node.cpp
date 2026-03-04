#include "master_ros2_interface/msg/custom_msg.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class PublisherNode : public rclcpp::Node {
public:
    PublisherNode() : rclcpp::Node(NODE_NAME)
    {
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(QOS_KEEPLAST_DEPTH)).reliable().transient_local();

        publisher_ = this->create_publisher<std_msgs::msg::String>(STD_PUB_TOPIC_NAME, qos_profile);

        custom_pubsliher_ =
            this->create_publisher<master_ros2_interface::msg::CustomMsg>(CUSTOM_PUB_TOPIC_NAME, qos_profile);

        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&PublisherNode::PublishMessages, this));
    }

private:
    void PublishMessages();

    static const std::string NODE_NAME;
    const std::string STD_PUB_TOPIC_NAME = "std_string_topic";
    const std::string CUSTOM_PUB_TOPIC_NAME = "custom_topic";
    const int QOS_KEEPLAST_DEPTH = 10;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<master_ros2_interface::msg::CustomMsg>::SharedPtr custom_pubsliher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

const std::string PublisherNode::NODE_NAME = "publisher_node";

void PublisherNode::PublishMessages()
{
    auto string_msg = std_msgs::msg::String();
    string_msg.data = "Hello, world";
    RCLCPP_INFO(this->get_logger(), "Publishing: %s", string_msg.data.c_str());
    publisher_->publish(string_msg);

    auto custom_msg = master_ros2_interface::msg::CustomMsg();
    custom_msg.data = "Custom Hello";
    custom_msg.number = 42;
    RCLCPP_INFO(this->get_logger(), "Publishing custom message: data=%s, number=%d", custom_msg.data.c_str(),
                custom_msg.number);
    custom_pubsliher_->publish(custom_msg);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherNode>());
    rclcpp::shutdown();
    return 0;
}