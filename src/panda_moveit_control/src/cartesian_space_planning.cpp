#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <chrono>
#include <memory>
#include <thread>
#include <vector>

class CartesianPlanning : public rclcpp::Node {
public:
    CartesianPlanning() : rclcpp::Node(NODE_NAME) {}
    void run();
    void plan();

private:
    static constexpr const char* NODE_NAME = "cartesian_planning";
    rclcpp::Node::SharedPtr shared_self_;
};

void CartesianPlanning::plan()
{
    geometry_msgs::msg::TransformStamped tf_base_to_eef;

    auto tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    moveit::planning_interface::MoveGroupInterface move_group(shared_self_, "arm");

    const std::string robot_base_frame = move_group.getPlanningFrame();
    const std::string end_effector_frame = move_group.getEndEffectorLink();

    while (rclcpp::ok()) {
        try {
            // 末端执行器相对于机器人, 基座标系的转换。
            tf_base_to_eef = tf_buffer->lookupTransform(robot_base_frame, end_effector_frame, tf2::TimePointZero);
            break;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s. Retry...", end_effector_frame.c_str(),
                        robot_base_frame.c_str(), ex.what());
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
    }

    if (!rclcpp::ok()) {
        return;
    }

    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation = tf_base_to_eef.transform.rotation;
    target_pose.position.x = tf_base_to_eef.transform.translation.x;
    target_pose.position.y = tf_base_to_eef.transform.translation.y;
    target_pose.position.z = tf_base_to_eef.transform.translation.z;

    std::vector<geometry_msgs::msg::Pose> waypoints;

    // 点位 1
    target_pose.position.z -= 0.2;
    waypoints.push_back(target_pose);

    // 点位 2
    target_pose.position.y -= 0.2;
    waypoints.push_back(target_pose);

    // 点位 3
    target_pose.position.z += 0.2;
    target_pose.position.y += 0.2;
    target_pose.position.x -= 0.2;
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory cartesian_trajectory;
    moveit_msgs::msg::MoveItErrorCodes error_code;

    const double eef_step_m = 0.01;
    const bool avoid_collisions = true;

    const double path_completion_ratio =
        move_group.computeCartesianPath(waypoints, eef_step_m, cartesian_trajectory, avoid_collisions, &error_code);

    if (path_completion_ratio > 0.8) {
        const auto exec_result = move_group.execute(cartesian_trajectory);
        if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Trajectory execution failed");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Cartesian path planning failed, completion ratio = %.3f, error_code = %d",
                     path_completion_ratio, error_code.val);
    }

    rclcpp::shutdown();
}

void CartesianPlanning::run()
{
    // 当前对象本身就是一个 Node，但后面两类组件都更适合接收 shared_ptr<Node>：
    // 1) executor 需要托管这个 Node，持续处理 TF / action / service 等回调
    // 2) MoveIt 需要使用这个 Node 发起 planning / execute
    // 因此这里把“当前对象自己”转换成 shared_ptr 形式，统一交给它们使用。
    // 注意：这不是创建新的 Node，只是当前这个 Node 的 shared_ptr 视图。
    shared_self_ = shared_from_this();

    // 子线程跑规划 planning
    std::thread planning_thread(&CartesianPlanning::plan, this);

    // 主线程跑Node TF以及各种信息回调处理
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(shared_self_);
    executor.spin();

    // 如果Planning线程没有结束,就等待结束。
    if (planning_thread.joinable()) {
        planning_thread.join();
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CartesianPlanning>();
    node->run();

    return 0;
}