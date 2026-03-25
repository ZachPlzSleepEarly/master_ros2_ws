#include <controller_interface/controller_interface.hpp>

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <cmath>
#include <cstddef>
#include <functional>
#include <string>
#include <vector>

namespace sine_controller {

class SineController final : public controller_interface::ControllerInterface {
public:
    SineController() = default;
    ~SineController() override = default;

    controller_interface::CallbackReturn on_init() override
    {
        auto_declare<std::vector<std::string>>(JOINTS_PARAM_NAME, {});
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override
    {
        joint_names_ = get_node()->get_parameter(JOINTS_PARAM_NAME).as_string_array();

        if (joint_names_.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "Parameter '%s' is empty", JOINTS_PARAM_NAME);
            return controller_interface::CallbackReturn::ERROR;
        }

        joint_count_ = joint_names_.size();

        InitializeControllerState();

        sine_param_subscription_ = get_node()->create_subscription<std_msgs::msg::Float32MultiArray>(
            SINE_PARAM_NAME, SINE_PARAM_QOS_DEPTH,
            std::bind(&SineController::SineParamCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_node()->get_logger(), "Configured SineController with %zu joints", joint_count_);

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration state_interface_configuration() const override
    {
        std::vector<std::string> interface_names;
        interface_names.reserve(joint_count_ * STATE_INTERFACES_PER_JOINT);

        for (const auto& joint_name : joint_names_) {
            interface_names.emplace_back(joint_name + POSITION_INTERFACE);
            interface_names.emplace_back(joint_name + VELOCITY_INTERFACE);
        }

        return {controller_interface::interface_configuration_type::INDIVIDUAL, interface_names};
    }

    controller_interface::InterfaceConfiguration command_interface_configuration() const override
    {
        std::vector<std::string> interface_names;
        interface_names.reserve(joint_count_);

        for (const auto& joint_name : joint_names_) {
            interface_names.emplace_back(joint_name + POSITION_INTERFACE);
        }

        return {controller_interface::interface_configuration_type::INDIVIDUAL, interface_names};
    }

    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override
    {
        if (!IsInterfaceSizesValid()) {
            return controller_interface::CallbackReturn::ERROR;
        }

        for (std::size_t joint_index = 0; joint_index < joint_count_; ++joint_index) {
            const auto position_value = state_interfaces_[StatePositionIndex(joint_index)].get_optional();

            if (!position_value.has_value()) {
                RCLCPP_ERROR(get_node()->get_logger(), "Failed to read position state interface for joint '%s'",
                             joint_names_[joint_index].c_str());
                return controller_interface::CallbackReturn::ERROR;
            }

            initial_positions_[joint_index] = *position_value;
        }

        elapsed_time_seconds_ = 0.0;

        RCLCPP_INFO(get_node()->get_logger(), "SineController activated");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration& period) override
    {
        elapsed_time_seconds_ += period.seconds();

        for (std::size_t joint_index = 0; joint_index < joint_count_; ++joint_index) {
            desired_positions_[joint_index] = initial_positions_[joint_index]
                + amplitudes_[joint_index] * std::sin(TWO_PI * frequencies_hz_[joint_index] * elapsed_time_seconds_);

            const bool write_ok = command_interfaces_[joint_index].set_value(desired_positions_[joint_index]);

            if (!write_ok) {
                RCLCPP_WARN(get_node()->get_logger(), "Failed to write position command for joint '%s'",
                            joint_names_[joint_index].c_str());
            }
        }

        return controller_interface::return_type::OK;
    }

private:
    static constexpr const char* JOINTS_PARAM_NAME = "joints";
    static constexpr const char* SINE_PARAM_NAME = "/sine_param";
    static constexpr const char* POSITION_INTERFACE = "/position";
    static constexpr const char* VELOCITY_INTERFACE = "/velocity";

    static constexpr std::size_t SINE_PARAM_QOS_DEPTH = 10;
    static constexpr std::size_t STATE_INTERFACES_PER_JOINT = 2;

    static constexpr double PI = 3.14159265358979323846;
    static constexpr double TWO_PI = 2.0 * PI;

    std::size_t joint_count_{0};
    double elapsed_time_seconds_{0.0};
    
    std::vector<std::string> joint_names_;
    std::vector<double> amplitudes_;
    std::vector<double> frequencies_hz_;
    std::vector<double> initial_positions_;
    std::vector<double> desired_positions_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sine_param_subscription_;

    void InitializeControllerState()
    {
        amplitudes_.assign(joint_count_, 0.0);
        frequencies_hz_.assign(joint_count_, 0.0);
        initial_positions_.assign(joint_count_, 0.0);
        desired_positions_.assign(joint_count_, 0.0);
        elapsed_time_seconds_ = 0.0;
    }

    bool IsInterfaceSizesValid() const
    {
        const std::size_t expected_state_interface_count = joint_count_ * STATE_INTERFACES_PER_JOINT;
        const std::size_t expected_command_interface_count = joint_count_;

        if (state_interfaces_.size() != expected_state_interface_count) {
            RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu state interfaces, but got %zu",
                         expected_state_interface_count, state_interfaces_.size());
            return false;
        }

        if (command_interfaces_.size() != expected_command_interface_count) {
            RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu command interfaces, but got %zu",
                         expected_command_interface_count, command_interfaces_.size());
            return false;
        }

        return true;
    }

    void SineParamCallback(const std_msgs::msg::Float32MultiArray::SharedPtr message)
    {
        const std::size_t expected_param_count = joint_count_ * 2U;

        if (message->data.size() != expected_param_count) {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "Expected %zu sine parameters [amp1, freq1, amp2, freq2, ...], got %zu", expected_param_count,
                         message->data.size());
            return;
        }

        for (std::size_t joint_index = 0; joint_index < joint_count_; ++joint_index) {
            amplitudes_[joint_index] = message->data[2U * joint_index];
            frequencies_hz_[joint_index] = message->data[2U * joint_index + 1U];
        }
    }

    static constexpr std::size_t StatePositionIndex(const std::size_t joint_index)
    {
        return joint_index * STATE_INTERFACES_PER_JOINT;
    }
};

} // namespace sine_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(sine_controller::SineController, controller_interface::ControllerInterface)