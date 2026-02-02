# Custom Dynamic Window Approach (DWA) Local Planner

**Technical Assessment: Robotics Software Apprentice | Origin (Formerly 10xConstruction)**

This repository contains a comprehensive, from-scratch implementation of the Dynamic Window Approach (DWA) local planner developed for the ROS 2 Humble framework. This project serves as a technical submission for the Robotics Software Apprentice position at Origin, demonstrating proficiency in autonomous navigation, kinematic modeling, and real-time obstacle avoidance.

---

## 🚀 Project Overview

The objective of this project is to automate the navigation of a TurtleBot3 Burger within a simulated environment. By implementing the DWA algorithm independently of existing libraries such as Nav2 or DWB, this project showcases a fundamental understanding of local path planning and motion control.

### Core Functionalities

- **Independent Algorithm Implementation**: The DWA logic was developed entirely in Python, adhering strictly to the requirement of avoiding pre-existing navigation plugins.
- **Goal-Oriented Navigation**: The system successfully navigates to specified coordinates with high precision and automated termination upon reaching the goal tolerance.
- **Collision Avoidance**: Real-time LiDAR data from the `/scan` topic is utilized to ensure safe passage through dynamic and static obstacles.
- **Enhanced Visualization**: The planner provides telemetry and trajectory candidates via RViz MarkerArray messages for comprehensive system auditing.

---

## 🛠 System Configuration and Installation

This project is deployed within a VS Code Dev Container to ensure a standardized and reproducible development environment, mitigating cross-platform dependency conflicts.

### 1. Environment Initialization
- Clone this repository.
- Open the directory in Visual Studio Code.
- Select the "Reopen in Container" option when prompted by the editor.

### 2. Dependency Management

Execute the following commands within the container terminal to install the necessary TurtleBot3 simulation packages:

```bash
# Navigate to the workspace source directory
cd ~/turtlebot3_ws/src

# Clone official ROBOTIS dependencies for Humble
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git

# Compile the workspace
cd ~/turtlebot3_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Environment Specification

Define the TurtleBot3 hardware model for the simulation:

```bash
export TURTLEBOT3_MODEL=burger
```

---

## 🏃 Simulation Execution

A unified launch file is provided to initialize the Gazebo environment, spawn the robot model, and execute the custom DWA planner node simultaneously.

```bash
ros2 launch dwa_planner_py dwa_sim.launch.py
```

### Goal Specification

A navigation goal may be assigned via the RViz "2D Goal Pose" interface or by publishing directly to the `/goal_pose` topic:

```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "
header:
  frame_id: 'odom'
pose:
  position: {x: 3.0, y: 0.0, z: 0.0}
  orientation: {w: 1.0}"
```

### Visualization and Analysis

To inspect the internal state of the planner:

1. Initialize RViz: `rviz2`
2. Configure the Fixed Frame to `odom`.
3. Add a MarkerArray display subscribed to the `/dwa_trajectories` topic.

**Visual Indicators:**
- **Blue Vectors**: Represent the set of sampled candidate trajectories.
- **Red Vector**: Represents the optimal trajectory selected for execution based on the cost function.

---

## 🧠 Technical Methodology

The DWA planner operates through a four-stage cyclic process:

1. **Velocity Sampling**: Linear and angular velocities are sampled within the dynamic and kinematic constraints of the robot.

2. **Trajectory Prediction**: The system predicts forward states for a 2.0-second horizon using a discrete-time unicycle kinematic model.

3. **Cost Function Evaluation**: Each candidate trajectory is scored based on weighted objectives:
   - **Target Proximity**: Minimizing Euclidean distance to the goal.
   - **Safety Margin**: Maximizing clearance from LiDAR-detected obstacles.
   - **Alignment**: Minimizing heading error to ensure smooth, efficient motion.

4. **Command Publication**: The velocity pair yielding the minimum cost is published to the `/cmd_vel` controller.

---

## 📚 References

The development of this planner was informed by the following academic and technical resources:

- **Primary Algorithm**: [The Dynamic Window Approach to Collision Avoidance](https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf) (Fox, Burgard, and Thrun, 1997)
- **Technical Tutorials**: Concepts of DWA implementation in Python - Reference Video
- **AI Assistance**: Generative AI tools were utilized to assist with documentation formatting and debugging.

---

## 📂 Repository Structure

```
dwa_planner_py/
├── launch/             # Orchestration files for simulation and node deployment
├── worlds/             # Gazebo world files containing obstacle configuration
├── dwa_planner_py/     # Primary Python implementation of the DWA node
├── setup.py            # Package metadata and entry point definitions
├── package.xml         # ROS 2 manifest and dependency declarations
└── README.md           # Documentation and setup guide
```
