# Robot modeling (URDF/TF) and Vision-Guided Autonomous Pick & Place via ROS, Gazebo & Stateflow (Robotics project)

An advanced simulation and robot integration laboratory covering **Coordinate Transformation Trees (`tf`)**, **Kinematic Robot Modeling (`URDF`)**, **Bag File Analysis**, and **Gazebo Co-Simulation (Pick-and-Place with Kinova Gen3)**, developed for the **Robotics** course at Politecnico di Torino.

---

## Table of contents
1. [Overview](#overview)
2. [Prerequisites & Requirements](#prerequisites--requirements)
3. [Architecture & Subsystems](#architecture--subsystems)
   - [Spatial Transformations (`tf` & `tf2`)](#1-spatial-transformations-tf--tf2)
   - [Kinematic & Dynamic Modeling (`URDF`)](#2-kinematic--dynamic-modeling-urdf)
   - [Telemetry Recording & Analysis (`rosbag`)](#3-telemetry-recording--analysis-rosbag)
   - [Gazebo Co-Simulation & Manipulation](#4-gazebo-co-simulation--manipulation-kinova-gen3)
4. [File & Task Mapping](#file--task-mapping)
5. [Project Structure](#project-structure)
6. [Usage Guide](#usage-guide)
   - [Coordinate Frame Transforms (`tf/`)](#coordinate-frame-transforms-tf)
   - [Rigid Body Tree Modeling from URDF (`urdf/`)](#rigid-body-tree-modeling-from-urdf-urdf)
   - [Rosbag Telemetry Processing (`rosbag/`)](#rosbag-telemetry-processing-rosbag)
   - [Autonomous Pick-and-Place in Gazebo (`gazebo_pickandplace/`)](#autonomous-pick-and-place-in-gazebo-gazebo_pickandplace)

---

## Overview

This laboratory extends robotic software design into geometric coordinate management, telemetry playback, dynamic robot descriptions, and full 3D physics-based manipulation:
* **Frame Tracking (`tf`):** Constructing dynamic and static coordinate tree transformations between base, sensor, and tool reference frames.
* **Robot Definition (`URDF`):** Parsing XML-based Unified Robot Description Format files into MATLAB `rigidBodyTree` models.
* **Data Log Analysis (`rosbag`):** Reading, filtering, deserializing, and plotting trajectories logged from live or simulated ROS systems (`turtlesim`).
* **Physics Co-Simulation (Gazebo):** Interfacing MATLAB with Gazebo to control a 7-DOF Kinova Gen3 robotic arm performing vision-guided object detection (YOLOv2), inverse kinematics, trajectory generation via `ros_control`, and Stateflow-based job coordination.

---

## Requirements

* **MATLAB Version:** R2022b or later.
* **Required Toolboxes:**
  * **ROS Toolbox**
  * **Robotics System Toolbox**
  * **Computer Vision Toolbox** (for YOLOv2 part detection)
  * **Deep Learning Toolbox** (for object classification)
  * **Stateflow** (for pick-and-place supervisory logic)
* **External Simulation:**
  * Ubuntu 20.04 LTS (native or virtualized) running **ROS Noetic** and **Gazebo 11** *(for live Gazebo pick-and-place synchronization)*.
* Set the MATLAB Current Folder to `/L2_ROS_MATLAB` before running the scripts.

---

## Architecture

### 1. Spatial Transformations (`tf` & `tf2`)
* **Transformation Tree (`rostf`):** Maintains the relative 3D spatial relationship between coordinate frames over time:
  $$\mathbf{T}_{\text{target}}^{\text{source}} = \begin{bmatrix} \mathbf{R} & \mathbf{t} \\ \mathbf{0}_{1\times3} & 1 \end{bmatrix}$$
* **Geometric Conversions:** Transforms 3D stamped points (`geometry_msgs/PointStamped`) across frames (e.g., from `camera_center` to `robot_base`).
* **Broadcasting:** Periodic publication of `geometry_msgs/TransformStamped` messages onto `/tf` utilizing interval timers.

### 2. Kinematic & Dynamic Modeling (`URDF`)
* **Parsing:** Native import of robot kinematic trees into MATLAB via `urdf2robot` or Simscape Multibody `smimport`.
* **Inertial & Geometric Verification:** Visualizes mass, center of mass, link geometry (box, cylinder, mesh), and joint dynamics (revolute, continuous, damping).

### 3. Telemetry Recording & Analysis (`rosbag`)
* **Offline Telemetry Processing:** High-performance retrieval of recorded sensor logs (`ros_turtlesim.bag`) without an active ROS Master.
* **Filtering & Serialization:** Extracting topic-specific sequences (`/turtle1/pose`) and parsing fields (`X`, `Y`, `Theta`) to reproduce Cartesian trajectories.

### 4. Gazebo Co-Simulation & Manipulation (Kinova Gen3)
* **Perception:** Overhead simulated RGB camera streaming raw frames to MATLAB; parts are localized and classified into predefined categories using a trained YOLOv2 detector (`exampleHelperYolo2DetectorROSGazebo.mat`).
* **Motion Planning & IK:** 
  * Task-space trajectories interpolated via trapezoidal velocity profiles (`trapveltraj`, `transformtraj`).
  * Numerical Inverse Kinematics (`inverseKinematics`) computing joint coordinates $q(t)$, velocities $\dot{q}(t)$, and accelerations $\ddot{q}(t)$.
* **Control Execution:**
  * Joint trajectories are packaged into `control_msgs/FollowJointTrajectoryAction` goals and dispatched to Gazebo's `gen3_joint_trajectory_controller`.
  * Robotiq 2F-85 gripper actuation executed through `control_msgs/GripperCommandAction`.
* **Supervisory Logic:** Stateflow chart (`exampleHelperFlowChartPickPlaceROSGazebo.sfx`) governing state transitions: Detection $\rightarrow$ Classification $\rightarrow$ Approach $\rightarrow$ Grasp $\rightarrow$ Lift $\rightarrow$ Place $\rightarrow$ Home.

---

## File mapping

| Subsystem / Path | File | Purpose |
| :--- | :--- | :--- |
| **TF** | `tf/exampleHelperROSStartTfPublisher.m` | Initializes `/tf` publisher with periodic timer |
| **TF** | `tf/exampleHelperROSTfPubTimer.m` | Publishes `robot_base` $\to$ `mounting_point` $\to$ `camera_center` |
| **TF** | `tf/ExampleHelperROSTimer.m` | Timer abstraction class for cyclic callback execution |
| **TF** | `tf/exampleHelperROSStopTfPublisher.m` | Gracefully cleans up timers and `/tf` handles |
| **URDF** | `urdf/DoublePendulum.urdf` | Rigid body description of a 2-DOF planar link system |
| **URDF** | `urdf/exampleDoublePendulum.txt` | Human-readable specification of link/joint parameters |
| **ROSBAG** | `rosbag/ros_turtlesim.bag` | Recorded dataset with `/tf`, `/turtle1/pose`, `/cmd_vel` |
| **GAZEBO** | `gazebo_pickandplace/exampleHelperCoordinatorPickPlaceROSGazebo.m` | Main coordinator holding robot models, ROS clients, and YOLO detector |
| **GAZEBO** | `gazebo_pickandplace/exampleCommandDetectPartsROSGazebo.m` | YOLOv2 vision inference & 3D coordinate de-projection |
| **GAZEBO** | `gazebo_pickandplace/exampleCommandClassifyPartsROSGazebo.m` | Part classification logic (Type 1 vs. Type 2) |
| **GAZEBO** | `gazebo_pickandplace/exampleCommandPickingLogicROSGazebo.m` | Selection of destination conveyor belt |
| **GAZEBO** | `gazebo_pickandplace/exampleCommandComputeGraspPoseROSGazebo.m` | Gripper grasp pose calculation |
| **GAZEBO** | `gazebo_pickandplace/exampleCommandMoveToTaskConfigROSGazebo.m` | Task-space trajectory generation, IK, and action dispatch |
| **GAZEBO** | `gazebo_pickandplace/exampleCommandActivateGripperROSGazebo.m` | Robotiq gripper action client call |
| **GAZEBO** | `gazebo_pickandplace/exampleHelperPackageJointTrajectoryKINOVAROSGazebo.m` | Trajectory packaging for `ros_control` |
| **GAZEBO** | `gazebo_pickandplace/exampleHelperFlowChartPickPlaceROSGazebo.sfx` | Stateflow coordination state machine |
| **GAZEBO** | `gazebo_pickandplace/exampleHelperYolo2DetectorROSGazebo.mat` | Pre-trained YOLOv2 detection model |
| **GAZEBO** | `gazebo_pickandplace/exampleHelperKINOVAGen3GripperROSGazebo.mat` | Kinova Gen3 with 2F-85 rigid body tree model |

---

## Project structure

```plaintext
root/
├── L2 - ROS Basics with MATLAB - part2.pdf               # Slide deck & theoretical material
│
├── tf/                                                  # Coordinate Transformation Tree
│   ├── ExampleHelperROSTimer.m
│   ├── exampleHelperROSSimTimer.m
│   ├── exampleHelperROSStartTfPublisher.m
│   ├── exampleHelperROSTfPubTimer.m
│   └── exampleHelperROSStopTfPublisher.m
│
├── urdf/                                                # Unified Robot Description Format
│   ├── DoublePendulum.urdf
│   └── exampleDoublePendulum.txt
│
├── rosbag/                                              # Data Logging & Analysis
│   └── ros_turtlesim.bag
│
└── gazebo_pickandplace/                                 # Kinova Gen3 Co-Simulation
    ├── exampleCommandActivateGripperROSGazebo.m
    ├── exampleCommandClassifyPartsROSGazebo.m
    ├── exampleCommandComputeGraspPoseROSGazebo.m
    ├── exampleCommandDetectPartsROSGazebo.m
    ├── exampleCommandMoveToTaskConfigROSGazebo.m
    ├── exampleCommandPickingLogicROSGazebo.m
    ├── exampleHelperCoordinatorPickPlaceROSGazebo.m
    ├── exampleHelperFlowChartPickPlaceROSGazebo.sfx
    ├── exampleHelperKINOVAGen3GripperROSGazebo.mat
    ├── exampleHelperPackageJointTrajectoryKINOVAROSGazebo.m
    └── exampleHelperYolo2DetectorROSGazebo.mat
```

---

## Usage

### Coordinate Frame Transforms (`tf/`)

1. Start ROS and launch the background transform publisher:
   ```matlab
   rosinit
   exampleHelperROSStartTfPublisher
   ```
2. Initialize the transformation tree object and list available frames:
   ```matlab
   tftree = rostf;
   pause(2);
   tftree.AvailableFrames
   ```
3. Query the static transform between `mounting_point` and `camera_center`:
   ```matlab
   mountToCamera = getTransform(tftree, 'mounting_point', 'camera_center');
   translation = mountToCamera.Transform.Translation
   ```
4. Transform a 3D coordinate point from camera space to the robot base:
   ```matlab
   pt = rosmessage('geometry_msgs/PointStamped');
   pt.Header.FrameId = 'camera_center';
   pt.Point.X = 3.0; pt.Point.Y = 1.5; pt.Point.Z = 0.2;
   transformed_pt = transform(tftree, 'robot_base', pt);
   ```
5. Broadcast a new frame (`wheel`) attached to `robot_base`:
   ```matlab
   tfStampedMsg = rosmessage('geometry_msgs/TransformStamped');
   tfStampedMsg.ChildFrameId = 'wheel';
   tfStampedMsg.Header.FrameId = 'robot_base';
   tfStampedMsg.Transform.Translation.Y = -0.2;
   tfStampedMsg.Transform.Translation.Z = -0.3;
   quat = axang2quat([0 1 0 deg2rad(30)]);
   tfStampedMsg.Transform.Rotation.W = quat(1);
   tfStampedMsg.Transform.Rotation.X = quat(2);
   tfStampedMsg.Transform.Rotation.Y = quat(3);
   tfStampedMsg.Transform.Rotation.Z = quat(4);
   tfStampedMsg.Header.Stamp = rostime('now');
   sendTransform(tftree, tfStampedMsg);
   ```
6. Cleanup:
   ```matlab
   exampleHelperROSStopTfPublisher
   rosshutdown
   ```

---

### Rigid Body Tree Modeling from URDF (`urdf/`)

1. Import the double pendulum model into a MATLAB `rigidBodyTree`:
   ```matlab
   robot = importrobot('urdf/DoublePendulum.urdf');
   robot.DataFormat = 'column';
   ```
2. Inspect robot properties and tree topology:
   ```matlab
   showdetails(robot)
   ```
3. Visualize the kinematic structure in 3D:
   ```matlab
   q = homeConfiguration(robot);
   figure;
   show(robot, q);
   title('Double Pendulum URDF Model');
   ```

---

### Rosbag Telemetry Processing (`rosbag/`)

1. Load the recorded log file:
   ```matlab
   bag = rosbag('rosbag/ros_turtlesim.bag');
   ```
2. Filter the bag for the desired telemetry topic:
   ```matlab
   bSel = select(bag, 'Topic', '/turtle1/pose');
   ```
3. Deserialize messages into high-speed structs:
   ```matlab
   msgStructs = readMessages(bSel, 'DataFormat', 'struct');
   ```
4. Extract 2D positions and reconstruct the driven path:
   ```matlab
   xPoints = cellfun(@(m) double(m.X), msgStructs);
   yPoints = cellfun(@(m) double(m.Y), msgStructs);

   figure;
   plot(xPoints, yPoints, 'LineWidth', 2);
   grid on;
   xlabel('X Position [m]');
   ylabel('Y Position [m]');
   title('Turtlesim Path Reconstructed from Rosbag');
   ```

---

### Autonomous Pick-and-Place in Gazebo (`gazebo_pickandplace/`)

*Ensure the Gazebo simulation instance running the Kinova Gen3 arm is launched in ROS on the host or connected virtual machine.*

1. Connect MATLAB to the ROS Master managing Gazebo:
   ```matlab
   rosinit('http://<GAZEBO_HOST_IP>:11311')
   ```
2. Load the robot model and initialize the coordinator:
   ```matlab
   cd('gazebo_pickandplace')
   load('exampleHelperKINOVAGen3GripperROSGazebo.mat'); % loads 'robot'
   homeConfig = [0; 0.26; 3.14; -2.27; 0; 0.96; 1.57];
   coordinator = exampleHelperCoordinatorPickPlaceROSGazebo(robot, homeConfig, 'EndEffector_Link');
   ```
3. Load and launch the Stateflow coordination state machine:
   ```matlab
   coordinator.FlowChart = exampleHelperFlowChartPickPlaceROSGazebo('coordinator', coordinator);
   ```
4. Observe autonomous operation:
   * **Object Detection:** The arm moves to the sensing pose, captures an overhead image, and applies the YOLOv2 network.
   * **Pick & Transfer:** Trajectories are generated via inverse kinematics and streamed to Gazebo.
   * **Place:** Objects are released on their corresponding destination conveyors based on class.
5. Once complete, stop execution and clean up:
   ```matlab
   delete(coordinator)
   rosshutdown
   cd('..')
   ```
