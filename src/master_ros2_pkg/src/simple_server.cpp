#include "master_ros2_interface/srv/concat_strings.hpp"

#include <rclcpp/rclcpp.hpp>

class StringConcatService : public rclcpp::Node {
public:
    StringConcatService() : rclcpp::Node(NODE_NAME)
    {
        service_ = this->create_service<master_ros2_interface::srv::ConcatStrings>(
            SERVICE_NAME,
            std::bind(&StringConcatService::HandleService, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Started ROS2 Service Server");
    }

private:
    void HandleService(const master_ros2_interface::srv::ConcatStrings_Request::ConstSharedPtr request,
                       master_ros2_interface::srv::ConcatStrings_Response::SharedPtr response);

    static const std::string NODE_NAME;
    const std::string SERVICE_NAME = "concat_strings";

    rclcpp::Service<master_ros2_interface::srv::ConcatStrings>::SharedPtr service_;
};

const std::string StringConcatService::NODE_NAME = "string_concat_service";

void StringConcatService::HandleService(const master_ros2_interface::srv::ConcatStrings_Request::ConstSharedPtr request,
                                        master_ros2_interface::srv::ConcatStrings_Response::SharedPtr response)
{
    response->concatenated_str = request->str1 + request->str2;
    RCLCPP_INFO(this->get_logger(), "Received: str1='%s', str2='%s'; Responding with: '%s'", request->str1.c_str(),
                request->str2.c_str(), response->concatenated_str.c_str());
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StringConcatService>());
    rclcpp::shutdown();
    return 0;
}