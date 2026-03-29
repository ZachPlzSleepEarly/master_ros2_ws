#include <rclcpp/rclcpp.hpp>

#include <rclcpp/utilities.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>

class PlanningWithObstacles : public rclcpp::Node {
public:
    PlanningWithObstacles() : Node(NODE_NAME) {}

    void run();
    void plan();
    void setup_world();

private:
    static constexpr const char* NODE_NAME = "planning_with_obstacles";

    /**
     * @brief
     * 当前对象自己这个 Node 的 shared_ptr 形式。
     * 这里需要 shared_ptr<Node>，是因为：
     * 1) executor 需要托管这个 Node，持续处理 TF / service / action 等回调
     * 2) MoveIt 的 MoveGroupInterface 也更适合接收 shared_ptr<Node>
     * 注意：这不是创建新的 Node，只是当前这个 Node 的 shared_ptr 视图。
     */
    rclcpp::Node::SharedPtr shared_self_;

    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

void PlanningWithObstacles::plan()
{
    geometry_msgs::msg::TransformStamped tf_base_to_eef;

    auto tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_self_, "arm");

    // 机器人基座坐标系(srdf virtual_joint 的 parent)
    const std::string planning_frame = move_group_->getPlanningFrame();
    // 法兰盘坐标系
    const std::string end_effector_frame = move_group_->getEndEffectorLink();

    setup_world();

    rclcpp::sleep_for(std::chrono::seconds(3));

    while (rclcpp::ok()) {
        try {
            tf_base_to_eef = tf_buffer->lookupTransform(planning_frame, end_effector_frame, tf2::TimePointZero);
            break;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s. Retry...", end_effector_frame.c_str(),
                        planning_frame.c_str(), ex.what());
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
    }

    // 设置终点坐标
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.w = tf_base_to_eef.transform.rotation.w;
    target_pose.orientation.x = tf_base_to_eef.transform.rotation.x;
    target_pose.orientation.y = tf_base_to_eef.transform.rotation.y;
    target_pose.orientation.z = tf_base_to_eef.transform.rotation.z;
    target_pose.position.x = tf_base_to_eef.transform.translation.x;
    target_pose.position.y = tf_base_to_eef.transform.translation.y;
    target_pose.position.z = tf_base_to_eef.transform.translation.z;

    target_pose.position.x += 0.3;
    move_group_->setPoseTarget(target_pose);

    // 运动路径规划、执行
    moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
    if (move_group_->plan(motion_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group_->move();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Motion planning failed");
    }

    rclcpp::shutdown();
}

void PlanningWithObstacles::run()
{
    shared_self_ = shared_from_this();

    // 子线程执行 planning / move
    std::thread planning_thread(&PlanningWithObstacles::plan, this);

    // 主线程持续处理 ROS 回调
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(shared_self_);
    executor.spin();

    if (planning_thread.joinable()) {
        planning_thread.join();
    }
}

void PlanningWithObstacles::setup_world()
{
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // 添加碰撞物（桌子）到世界
    moveit_msgs::msg::CollisionObject table_collision_object;
    table_collision_object.id = "table";
    table_collision_object.header.frame_id = move_group_->getPlanningFrame();
    // 形状、尺寸
    shape_msgs::msg::SolidPrimitive table_primitive;
    table_primitive.type = table_primitive.BOX;
    table_primitive.dimensions.resize(3);
    table_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.1;
    table_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 1.5;
    table_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.3;
    // 位姿
    geometry_msgs::msg::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = 0.48;
    table_pose.position.y = 0.0;
    table_pose.position.z = 0.25;

    table_collision_object.primitives.push_back(table_primitive);
    table_collision_object.primitive_poses.push_back(table_pose);
    table_collision_object.operation = table_collision_object.ADD;

    planning_scene_interface.applyCollisionObject(table_collision_object);

    // 添加碰撞物（抓取物体）到世界
    moveit_msgs::msg::CollisionObject grasp_collision_object;
    grasp_collision_object.id = "grasp";
    grasp_collision_object.header.frame_id = move_group_->getEndEffectorLink();
    // 形状、尺寸
    shape_msgs::msg::SolidPrimitive grasp_primitive;
    grasp_primitive.type = grasp_primitive.CYLINDER;
    grasp_primitive.dimensions.resize(2);
    grasp_primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = 0.1;
    grasp_primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = 0.04;
    // 位姿
    geometry_msgs::msg::Pose grasp_object_pose;
    grasp_object_pose.orientation.w = 1.0;
    grasp_object_pose.position.z = 0.28;

    grasp_collision_object.primitives.push_back(grasp_primitive);
    grasp_collision_object.primitive_poses.push_back(grasp_object_pose);
    grasp_collision_object.operation = grasp_collision_object.ADD;

    planning_scene_interface.applyCollisionObject(grasp_collision_object);

    // 把抓取物体附着在手上（告诉MoveIt grasp 物体不是普通障碍物，而是被机械手 hand 抓住）
    std::vector<std::string> touch_links;
    touch_links.push_back("panda_rightfinger");
    touch_links.push_back("panda_leftfinger");

    move_group_->attachObject(grasp_collision_object.id, "hand", touch_links);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto planning_node = std::make_shared<PlanningWithObstacles>();
    planning_node->run();

    return 0;
}