#ifndef TRAJECTORY_GENERATOR_HPP
#define TRAJECTORY_GENERATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/cartesian_interpolator.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

namespace local_planner
{
/// \class trajectory_generator_c
/// \brief Class to generate and publish Cartesian trajectories for a robotic arm. 
class trajectory_generator_c : public rclcpp::Node
{
public:
    /// \brief Constructor for TrajectoryGenerator node.
    trajectory_generator_c();

    /// \brief Generates a geometric path for the end-effector in a square pattern.
    void generate_square_path();

    /// \brief Computes a trajectory from the generated geometric path using Cartesian planning.
    void compute_trajectory();

    /// \brief Publishes the computed trajectory on a ROS topic.
    void publish_trajectory();

private:
    /// \brief Publisher for the trajectory
    rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr m_trajectory_pub;
    /// \brief MoveGroupInterface for controlling the robotic arm
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> m_move_group;
    /// \brief Waypoints defining the geometric path
    std::vector<geometry_msgs::msg::Pose> m_waypoints;
    /// \brief The computed trajectory
    moveit_msgs::msg::RobotTrajectory m_trajectory;
};
} // namespace local_planner

#endif // TRAJECTORY_GENERATOR_HPP