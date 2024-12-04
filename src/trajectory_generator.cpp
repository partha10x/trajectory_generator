#include <trajectory_generator/trajectory_generator.hpp>
#include <moveit/planning_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace trajectory_generator
{

trajectory_generator_c::trajectory_generator_c(
    rclcpp::Node::SharedPtr node,
    const std::string& group_name)
    : node_(node),
      move_group_(node, group_name)
{
    visual_tools_ = std::make_shared<moveit::visual_tools::VisualTools>(move_group_.getRobotModel()->getRootLinkName(), node_);
    visual_tools_->loadRobotStatePub();
    visual_tools_->loadTrajectoryPub();
}

void trajectory_generator_c::generate_square_path()
{
    // Start by defining the current pose (end effector pose)
    geometry_msgs::msg::Pose current_pose = move_group_.getCurrentPose().pose;

    // Define a set of waypoints to form a square path
    waypoints_.clear();
    waypoints_.push_back(current_pose);  // Start point
    geometry_msgs::msg::Pose target_pose = current_pose;

    target_pose.position.x += 0.5;  // Move to the right (X direction)
    waypoints_.push_back(target_pose);

    target_pose.position.y += 0.5;  // Move up (Y direction)
    waypoints_.push_back(target_pose);

    target_pose.position.x -= 0.5;  // Move to the left (X direction)
    waypoints_.push_back(target_pose);

    target_pose.position.y -= 0.5;  // Move down (Y direction)
    waypoints_.push_back(target_pose);  // End point (back to the start)
}

void trajectory_generator_c::plan_square_path()
{
    generate_square_path();

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const double eef_step = 0.001;
    const double jump_threshold = 0.0;

    move_group_.computeCartesianPath(waypoints_, eef_step, jump_threshold, trajectory_);
    RCLCPP_INFO(node_->get_logger(), "Trajectory computed with %zu waypoints", trajectory_.joint_trajectory.points.size());
}

void trajectory_generator_c::execute_trajectory()
{
    move_group_.execute(trajectory_);
}

void trajectory_generator_c::visualize_trajectory()
{
    visual_tools_->publishTrajectoryLine(trajectory_, move_group_.getEndEffectorLink());
    visual_tools_->trigger();
}

}  // namespace trajectory_generator