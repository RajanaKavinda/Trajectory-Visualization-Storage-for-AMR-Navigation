# Autonomous Mobile Robot (AMR) Trajectory Visualization and Storage

## Overview
This ROS 2 package provides a solution for visualizing and storing trajectory data during an Autonomous Mobile Robot (AMR) navigation task. The system automates trajectory visualization in RViz and enables easy storage in various formats, reducing manual efforts in monitoring and analyzing the robot's movement.

## Problem Statement
In real-world AMR applications, such as in a manufacturing facility, tracking the robot's trajectory is crucial for analysis and optimization. Manual handling of trajectory data in each navigation session is inefficient. This package provides automated ROS nodes for:
- **Collecting and storing** robot trajectory data.
- **Visualizing** the trajectory in RViz.
- **Reading and replaying** stored trajectories.

## Features
### 1. Trajectory Publisher and Saver Node (`trajectory_pub_saver`)
- Collects the path followed by the robot in real-time.
- Publishes trajectory data as a MarkerArray for visualization in RViz.
- Provides a ROS 2 service to save trajectory data in **JSON** format.
- Allows users to specify a time duration to store only recent trajectory data.

### 2. Trajectory Reader and Publisher Node (`trajectory_reader`)
- Reads a previously stored trajectory file.
- Transforms trajectory data to the `odom` frame.
- Publishes the stored trajectory for visualization in RViz.

## System Requirements
- ROS 2 Humble
- TurtleBot3 Simulation
- RViz2

### Installation & Setup
1. Install the necessary dependencies:
   ```bash
   sudo apt install ros-humble-turtlebot3*
   ```
2. Set up the TurtleBot3 environment by adding the following line to `~/.bashrc`:
   ```bash
   export TURTLEBOT3_MODEL=waffle
   ```
   Then, source the updated `bashrc`:
   ```bash
   source ~/.bashrc
   ```

3. Launch the TurtleBot3 simulation:
   ```bash
   ros2 launch turtlebot3_gazebo empty_world.launch.py
   ```
4. Clone and build the package:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository_url>
   cd ~/ros2_ws
   colcon build --packages-select trajectory_pkg
   source install/setup.bash
   ```

## Usage
### 1. Launch the Trajectory Nodes
```bash
ros2 launch trajectory_pkg trajectory_visualization.launch.py
```
### 2. Open Rviz2
```bash
rviz2
```
### 3. Control the TurtleBot3
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

### 4. Store the Trajectory
```bash
ros2 service call /store_path trajectory_pkg/srv/SaveTrajectory "filename: 'my_path' duration: 10.0"
```

### 5. Read and Visualize a Stored Trajectory
```bash
ros2 service call /read_path trajectory_pkg/srv/ReadTrajectory "filename: 'my_path'"
```

### 5. RViz Setup
Ensure that RViz is configured to display:
- **MarkerArray** for `/path_visual` (live trajectory visualization).
- **MarkerArray** for `/loaded_path_markers` (stored trajectory visualization).

## Algorithm / Pseudocode
### Trajectory Collection & Storage (Pseudo-Code)
```
Initialize ROS node
Subscribe to odometry topic
Create a timer to record trajectory at regular intervals
For each received odometry message:
    Store position and orientation
    Update trajectory visualization
On service request to save:
    Filter trajectory data based on requested duration
    Save trajectory to JSON file
```

### Trajectory Reading & Visualization (Pseudo-Code)
```
Initialize ROS node
On service request to read trajectory:
    Load JSON file
    Publish trajectory as a visualization marker array
```

## Code Modularity & Best Practices
✅ **Object-Oriented Design** - The package follows OOP principles with separate header and source files.

✅ **Flexible Configuration** - File paths and topics are parameterized.

✅ **ROS Standards** - Adheres to ROS 2 best practices for modularity and maintainability.
