# IGNR
Image-Guided Robotic Navigation
A hands-on implementation of an end-to-end pipeline that combines 3D Slicer path planning with ROS 2 kinematic validation. Define your entry/target in Slicer, compute a clearance-maximizing straight-line trajectory, send it over OpenIGTLink, and drive your robot (in simulation or on hardware) via MoveIt 2 and RViz.

Features
3D Slicer Module (PathPlanning_copy)

Place entry/target fiducials (vtkMRMLMarkupsFiducialNode) and define critical/target masks (vtkMRMLScalarVolumeNode)

Adjustable lengthThreshold (QDoubleSpinBox, default 150 mm)

Danielsson distance transform for clearance mapping

Collision‐ and length‐based filtering, with automatic selection of the safest straight‐line path

OpenIGTLink Export

Wraps the RAS→base_link transform in an IGTL_LINEAR_TRANSFORM message

Pushes over TCP (default port 18944) via the OpenIGTLinkIF Slicer extension

ros2_igtl_bridge

Listens as a client on the same port

Deserialises IGTL, builds a geometry_msgs/TransformStamped → PoseStamped

Publishes /slicer_to_ros/target_pose for downstream planners

MoveIt 2 Mover (robot_move_to_pose)

Loads your robot’s URDF via robot_description

Spawns robot_state_publisher and TF tree

Plans & executes via MoveIt’s C++ MoveGroupInterface

Publishes /robot_execution_status (std_msgs/Bool) on success/failure

RViz Visualization

Visualise start & goal poses, joint trajectories, and real-time ee_link TF

Handy “Plan” / “Execute” widgets for manual overrides

Prerequisites
3D Slicer (≥ 5.0) with Python support

SimpleITK, VTK, and the OpenIGTLinkIF extension installed in Slicer

ROS 2 (Foxy / Galactic / Humble) on Ubuntu 20.04 (guest)

MoveIt 2 and robot_state_publisher packages

Oracle VirtualBox (host on Windows)

Python 3.8+ with pyigtl, rclpy

Installation
Clone this repo into your ROS 2 workspace:

bash
Copy
Edit
cd ~/ros2_ws/src
git clone https://github.com/yourusername/image-guided-navigation.git
Build ROS 2 packages:

bash
Copy
Edit
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
Install the Slicer module:

Copy PathPlanning_copy/ into Slicer’s Modules/Scripted/ directory, or install via the Extensions Manager.

Restart Slicer; you’ll find “PathPlanning_copy” under Extensions ➔ ImageGuidedNavigation.

VirtualBox Network Setup
By default VirtualBox uses NAT, which hides the VM behind its own subnet. To let Slicer talk to the bridge on port 18944:

Bridged Adapter:

Open VM settings → Network → Adapter 1 → set “Attached to” → Bridged Adapter → choose your Wi-Fi card.

Both host and guest now live on the same LAN (e.g. 192.168.1.x).

Alternative:

Use Host-Only Adapter or port-forwarding (host port 18944 → guest 18944).

Usage
Compute path in Slicer

Load your anatomical volume (e.g. cortex.nii) and binary masks (vessels.nrrd, tumour.nrrd).

Open PathPlanning_copy: select entry/target fiducials, critical/target masks, adjust lengthThreshold, click Apply.

Verify outputFiducials appears with two control points.

Send to ROS

In PathPlanning_copy: click Send to ROS (OpenIGTLinkIF connector must be SERVER on port 18944).

Run the bridge

bash
Copy
Edit
source ~/ros2_ws/install/setup.bash
ros2 run ros2_igtl_bridge bridge_node --ros-args -p ip_address:=192.168.1.42 -p port:=18944
Drive the robot

bash
Copy
Edit
ros2 launch image_guided_navigation robot_move_to_pose.launch.py
Watch /slicer_to_ros/target_pose in ros2 topic echo.

Observe /robot_execution_status for success/failure.

Visualize in RViz

bash
Copy
Edit
rviz2 -d ~/ros2_ws/src/image-guided-navigation/launch/robot_move.rviz
Use the Plan and Execute buttons, and watch the ee_link TF in real time.

Validation
Unit tests for the Slicer module: ctest -R PathPlanning_copy

Integration check: send identity transform from Slicer, confirm PoseStamped on /slicer_to_ros/target_pose

System demo: run Gazebo, issue a reachable pose, and verify end-effector accuracy < 2 mm RMS.

Contributing
Fork the repo

Create a feature branch (git checkout -b feature/your-feature)

Commit your changes, push to your fork

Open a Pull Request—describe your change, reference any issue

Repository for an Image-Guided Navigation system for the KCL module 7MRI0070 
