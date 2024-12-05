#include <trajectory_generator/trajectory_generator.hpp>

///////////////////////////////////////////////////////////////////////////////
local_planner::trajectory_generator_c::trajectory_generator_c()
    : Node("trajectory_generator"),
      m_move_group(nullptr)
{
    // Create a separate node handle for MoveGroup
    auto move_group_node = std::make_shared<rclcpp::Node>(
        "trajectory_generator_move_group_node",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create a planning scene monitor
    auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        move_group_node, "robot_description"
    );
    
    if (!planning_scene_monitor->getPlanningScene()) {
        RCLCPP_ERROR(this->get_logger(), "Planning scene not properly loaded");
        throw std::runtime_error("Planning scene not properly loaded");
    }

    planning_scene_monitor->startStateMonitor();
    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->startWorldGeometryMonitor();

    // Wait for the planning scene to be ready
    if (!planning_scene_monitor->waitForCurrentRobotState(rclcpp::Time(0), 10.0)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get robot state");
        throw std::runtime_error("Failed to get robot state");
    }

    // Initialize MoveGroup after planning scene is ready
    m_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        move_group_node,
        "ur_manipulator",
        std::make_shared<tf2_ros::Buffer>(this->get_clock()),
        rclcpp::Duration::from_seconds(5.0)
    );

    m_trajectory_pub = this->create_publisher<moveit_msgs::msg::RobotTrajectory>("/trajectory", 10);
    
    // Wait for move_group to be ready
    if (!m_move_group->startStateMonitor()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start state monitor");
        throw std::runtime_error("Failed to start state monitor");
    }

    RCLCPP_INFO(this->get_logger(), "Trajectory Generator initialized.");
}

///////////////////////////////////////////////////////////////////////////////
void local_planner::trajectory_generator_c::generate_square_path()
{
    // Ensure we have a valid robot state before proceeding
    auto current_state = m_move_group->getCurrentState(10.0);
    if (!current_state) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get current robot state");
        return;
    }

    // Get current end-effector pose
    auto current_pose = m_move_group->getCurrentPose().pose;

    // Define square waypoints
    geometry_msgs::msg::Pose pose = current_pose;
    m_waypoints.clear();

    // First point (current position)
    m_waypoints.push_back(pose);

    // Move in X direction
    pose.position.x += 0.2;
    m_waypoints.push_back(pose);

    // Move in Z direction
    pose.position.z += 0.2;
    m_waypoints.push_back(pose);

    // Move in -X direction
    pose.position.x -= 0.2;
    m_waypoints.push_back(pose);

    // Return to start (move in -Z direction)
    pose.position.z -= 0.2;
    m_waypoints.push_back(pose);
}

///////////////////////////////////////////////////////////////////////////////
void local_planner::trajectory_generator_c::compute_trajectory()
{
    const double jump_threshold = 0.0;
    const double eef_step = 0.01; // Step size for smoother path
    moveit_msgs::msg::RobotTrajectory raw_trajectory;

    // Compute Cartesian Path
    double fraction = m_move_group->computeCartesianPath(m_waypoints, eef_step, jump_threshold, raw_trajectory);

    if (fraction < 1.0)
    {
        RCLCPP_WARN(this->get_logger(), "Cartesian path incomplete: %.2f%% completed", fraction * 100.0);
    }

    // Apply time parameterization for smooth velocity profiles
    robot_trajectory::RobotTrajectory robot_trajectory(m_move_group->getRobotModel(), "ur_manipulator");
    robot_trajectory.setRobotTrajectoryMsg(*m_move_group->getCurrentState(), raw_trajectory);

    trajectory_processing::IterativeParabolicTimeParameterization time_param;
    if (time_param.computeTimeStamps(robot_trajectory))
    {
        RCLCPP_INFO(this->get_logger(), "Time parameterization applied successfully.");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Time parameterization failed.");
    }

    // Convert back to ROS message
    robot_trajectory.getRobotTrajectoryMsg(m_trajectory);
}

///////////////////////////////////////////////////////////////////////////////
void local_planner::trajectory_generator_c::publish_trajectory()
{
    m_trajectory_pub->publish(m_trajectory);
    RCLCPP_INFO(this->get_logger(), "Trajectory published.");
}