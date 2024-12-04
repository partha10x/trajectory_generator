#ifndef TRAJECTORY_GENERATOR_HPP
#define TRAJECTORY_GENERATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_interface/move_group_interface.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_interface/planning_scene_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <moveit/visual_tools/visual_tools.h>
#include <vector>

namespace trajectory_generator
{

class trajectory_generator_c
{
public:
    trajectory_generator_c(rclcpp::Node::SharedPtr node, const std::string& group_name);
    void plan_square_path();
    void execute_trajectory();
    void visualize_trajectory();

private:
    void generate_square_path();
    void add_waypoints_for_square_path();

    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    moveit::visual_tools::VisualToolsPtr visual_tools_;
    std::vector<geometry_msgs::msg::Pose> waypoints_;
    moveit_msgs::msg::RobotTrajectory trajectory_;
};

}  // namespace trajectory_generator

#endif  // TRAJECTORY_GENERATOR_HPP