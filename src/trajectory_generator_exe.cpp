#include <trajectory_generator/trajectory_generator.hpp>

///////////////////////////////////////////////////////////////////////////////
/// \brief Main function to initialize and run the TrajectoryGenerator node.
/// \param argc Number of command-line arguments.
/// \param argv Array of command-line arguments.
/// \return int Exit code.

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<local_planner::trajectory_generator_c>();
    node->generate_square_path();
    node->compute_trajectory();
    node->publish_trajectory();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}