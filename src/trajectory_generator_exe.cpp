#include "rclcpp/rclcpp.hpp"
#include <trajectory_generator/trajectory_generator.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("square_trajectory_planner_node");

    trajectory_generator::trajectory_generator_c planner(node, "ur_manipulator");

    // Plan and visualize the trajectory
    planner.plan_square_path();
    planner.visualize_trajectory();

    // Execute the planned trajectory
    planner.execute_trajectory();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}