## Trajectory Generator 
The trajectory_generator package provides a ROS 2 node that generates a square geometric path for a UR10e robot arm in Cartesian space. This package leverages MoveIt 2 for planning and execution and visualizes the trajectory in RViz. The main goal is to allow the robot's end-effector to follow a square trajectory while simulating or controlling the robot in a MoveIt 2-supported environment.

### Installation
Clone the repository into your ROS 2 workspace:

```
cd ~/ros2_ws/src
git clone <trajectory_generator repo url>
```

#### Install dependencies:

```
sudo apt update
sudo apt install ros-humble-moveit ros-humble-moveit-core ros-humble-moveit-ros-planning-interface
```

#### Build the workspace:

```
cd ~/ros2_ws
colcon build --packages-select trajectory_generator --mixin-release --parallel-workers 2
source install/setup.bash
```

### Usage

1. Launch the UR10e Robot with MoveIt 2
Before running the node, ensure that the UR10e robot description and MoveIt 2 configuration are running in the ROS 2 environment. Use an appropriate launch file, such as:
```
ros2 launch ur10e_moveit_config ur10e_moveit_planning_execution.launch.py
```

2. Run the trajectory_generator_node
Start the trajectory_generator node to generate and visualize the square trajectory:
```
ros2 run trajectory_generator trajectory_generator_exe
```