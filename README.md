# Image-Guided Robotic Navigation

A hands-on implementation of an end-to-end pipeline that combines 3D Slicer path planning with ROS 2 kinematic validation. Define your entry/target in Slicer, compute a clearance-maximising straight-line trajectory, send it over OpenIGTLink, and drive your robot (in simulation or on hardware) via MoveIt 2 and RViz.

---

## Features

- **3D Slicer Module** (`PathPlanning_copy`)  
  - Place entry/target fiducials (`vtkMRMLMarkupsFiducialNode`) and define critical/target masks (`vtkMRMLScalarVolumeNode`)  
  - Adjustable `lengthThreshold` (QDoubleSpinBox, default 150 mm)  
  - Danielsson distance transform for clearance mapping  
  - Collision- and length-based filtering, with automatic selection of the safest straight-line path

- **OpenIGTLink Export**  
  - Wraps the RAS→base_link transform in an `IGTL_LINEAR_TRANSFORM` message  
  - Sends over TCP (default port 18944) via the OpenIGTLinkIF Slicer extension

- **ros2_igtl_bridge**  
  - Listens as a client on the same port  
  - Deserialises IGTL, builds a `geometry_msgs/TransformStamped` → `PoseStamped`  
  - Publishes `/slicer_to_ros/target_pose` for downstream planners

- **MoveIt 2 Mover** (`robot_move_to_pose`)  
  - Loads your robot’s URDF via `robot_description`  
  - Spawns `robot_state_publisher` and TF tree  
  - Plans & executes via MoveIt’s C++ `MoveGroupInterface`  
  - Publishes `/robot_execution_status` (`std_msgs/Bool`) on success/failure

- **RViz Visualization**  
  - Visualise start & goal poses, joint trajectories, and real-time `ee_link` TF  
  - Handy **Plan** / **Execute** widgets for manual overrides

---

## Prerequisites

- **3D Slicer** (≥ 5.0) with Python support  
- **SimpleITK**, **VTK**, and the **OpenIGTLinkIF** extension installed in Slicer  
- **ROS 2** (Foxy / Galactic / Humble) on Ubuntu 20.04 (guest)  
- **MoveIt 2** and `robot_state_publisher` packages  
- **Oracle VirtualBox** (host on Windows)  
- **Python 3.8+** with `pyigtl`, `rclpy`

---

## Installation

1. **Clone this repo** into your ROS 2 workspace:  
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/yourusername/image-guided-navigation.git
