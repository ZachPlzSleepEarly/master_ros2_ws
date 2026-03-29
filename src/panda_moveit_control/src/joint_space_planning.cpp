#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    auto node = rclcpp::Node::make_shared("JointSpacePlanning");
    moveit::planning_interface::MoveGroupInterface move_group(node, "arm");

	// 在 joint space 规划并移到到点位 1
    std::vector<double> desired_joint_pos = {0.0, 0.0, 0.0, -0.35, 0.0, 1.57, 0.78};
    if (!move_group.setJointValueTarget(desired_joint_pos)) {
        std::cout << "Joint outer bounds" << std::endl;
        return -1;
    }
    move_group.setMaxVelocityScalingFactor(1.0);
    move_group.setMaxAccelerationScalingFactor(1.0);
    if (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group.move();
    }

    rclcpp::sleep_for(std::chrono::seconds(3));

	// 在 joint space 规划并移到到点位 1
    desired_joint_pos = {0.0, -0.78, 0.0, -2.35, 0.0, 1.57, 0.78};
	if (!move_group.setJointValueTarget(desired_joint_pos)) {
        std::cout << "Joint outer bounds" << std::endl;
        return -1;
    }
    move_group.setMaxVelocityScalingFactor(1.0);
    move_group.setMaxAccelerationScalingFactor(1.0);
    if (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group.move();
    }
}
