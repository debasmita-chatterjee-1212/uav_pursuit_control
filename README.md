# Vision-Based Autonomous UAV Pursuit Control

**Autonomous Aerial Robotics | Model Predictive Control | Computer Vision | ROS**

Mitacs Globalink Research Internship — 2022  
Department of Mechanical Engineering  
University of Alberta, Canada

---

## Project Overview

This research investigates autonomous pursuit between unmanned aerial vehicles (UAVs), where a pursuer UAV detects, localizes and tracks a moving target UAV using visual observations and closed-loop motion control.

The project builds a laboratory research framework for autonomous aerial pursuit, with an emphasis on motion control, relative pose estimation, UAV communication and monocular 3D object detection.

The broader research involved Parrot AR.Drone 2.0 and Parrot ANAFI platforms, Vicon motion capture, ROS-based control architectures and MAVLink-based flight operations.

This repository preserves research implementations and experimental prototypes from the 2022 internship, including classical and predictive motion control, spatial transformations, reference generation and flight-system integration.

## Research Objectives

- Investigate autonomous vision-based detection and tracking of a moving UAV.
- Develop closed-loop motion control for UAV position and yaw regulation.
- Explore classical and model predictive control strategies for trajectory tracking.
- Process UAV pose information using quaternion representations and coordinate transformations.
- Investigate relative localization and moving reference generation.
- Develop MAVLink-based communication and flight-control routines.
- Support monocular 3D object detection through annotated in-flight imagery.
- Investigate UAV control and perception through simulation and indoor hardware experiments.

## System Architecture

The broader research framework comprises five functional components:

| Module | Functionality |
|---|---|
| Perception | Monocular UAV detection and relative 3D position estimation |
| State processing | Vicon motion capture, attitude representation and coordinate transformations |
| Reference generation | Relative pose processing and moving target references |
| Motion control | PID and Linear Model Predictive Control |
| Flight execution | ROS-based command publication and MAVLink-based flight operations |

The research investigates the integration of perception and control for autonomous UAV pursuit. The repository contains individual implementations and experimental modules rather than a single fully integrated software package.

## Technical Implementation

### 1. Relative Localization and Reference Generation

The Vicon-based pursuit framework processes the spatial positions and orientations of the pursuer and target UAVs.

The implementation includes:

- Acquisition of UAV pose information through ROS TF.
- Relative position and orientation calculations.
- Coordinate transformations between world and body reference frames.
- Generation of moving reference positions for target tracking.
- Publication of reference position, velocity and yaw through custom ROS messages.

Relevant implementation:

- [Pursuit_vicon.cpp](./src/Pursuit_vicon.cpp)
- [af_speed.cpp](./src/af_speed.cpp)

### 2. Proportional-Integral-Derivative Control

The PID control framework regulates translational position along three spatial axes and rotational position about the yaw axis.

The repository contains PID implementations associated with both the AR.Drone and ANAFI platforms.

The ANAFI implementation includes:

- Four-channel PID motion control.
- World-to-body frame transformation of position errors.
- Quaternion-based attitude processing.
- Angular error normalization.
- Error deadband.
- Interactive tuning of 12 PID gains using OpenCV trackbars.
- ROS-based velocity command publication.
- CSV-based experimental telemetry logging.

The controller is structured around a nominal 25 Hz ROS control loop.

Relevant implementation:

- [AR_PID.cpp](./src/AR_PID.cpp)
- [af_pid.cpp](./src/af_pid.cpp)

### 3. Linear Model Predictive Control

The predictive control framework investigates UAV trajectory tracking through a linear dynamic model and finite-horizon optimization.

The archived LMPC implementation includes:

- An eight-state UAV dynamic model.
- Position, yaw and velocity state representations.
- Finite-horizon prediction.
- Control-input constraints.
- Luenberger observer-based velocity estimation.
- Low-pass filtering.
- ROS-based control command publication.

The LMPC implementation provides the predictive control component of the earlier AR.Drone research framework.

Relevant implementation:

- [AR_LMPC.cpp](./src/AR_LMPC.cpp)

### 4. Attitude Representation and Spatial Transformations

UAV orientation information obtained from motion-capture systems is processed using quaternion representations and Euler angles.

The repository includes C++ and Python implementations investigating:

- Quaternion-to-Euler conversion.
- Roll, pitch and yaw extraction.
- Rotation matrices.
- Spatial coordinate transformations.
- ROS TF-based pose processing.

Relevant implementation:

- [converter.cpp](./src/converter.cpp)
- [eulerangles.cpp](./src/eulerangles.cpp)
- [eulerangles2.cpp](./src/eulerangles2.cpp)
- [quaternion_euler.py](./src/quaternion_euler.py)

### 5. MAVLink Communication and Flight Operations

The research also explored MAVLink-based communication and MAVSDK-based flight-control routines.

The experimental implementations investigate:

- UAV system discovery and communication.
- Telemetry acquisition.
- Flight-operation commands.
- Takeoff and landing sequences.
- Simulation-based evaluation using PX4 and jMAVSim.

A separate CMake-based C++17 project is included for MAVSDK flight operations.

Relevant implementation:

- [takeoff_and_land/](./src/takeoff_and_land/)
- [mavlink_takeoff.cpp](./src/mavlink_takeoff.cpp)
- [mavtakeoff.cpp](./src/mavtakeoff.cpp)

### 6. ROS Communication and System Configuration

Supporting implementations investigate ROS publishers, subscribers, custom message interfaces and launch configurations.

Relevant components:

- [first_publisher.cpp](./src/first_publisher.cpp)
- [publisher_quaternion.cpp](./src/publisher_quaternion.cpp)
- [subscriber_quaternion.cpp](./src/subscriber_quaternion.cpp)
- [message.cpp](./src/message.cpp)
- [spData.msg](./src/spData.msg)
- [traj.msg](./src/traj.msg)
- [anafi.launch](./src/anafi.launch)
- [anafi2.launch](./src/anafi2.launch)

### 7. Monocular 3D Object Detection and Dataset Preparation

The broader research investigated monocular 3D object detection using a BirdNet3D-derived framework to estimate the relative position of a target UAV.

Dataset preparation involved:

- Acquisition of images during indoor UAV flight experiments.
- Annotation of target UAV bounding boxes.
- Annotation of structural components and keypoints.
- Preparation of visual data for learning-based UAV detection and localization.

The perception model and complete training dataset are not included in this repository. This component is described to establish the broader research context.

## Experimental Environment

### Hardware

- Parrot AR.Drone 2.0 — foundational laboratory control platform.
- Parrot ANAFI quadrotors — platform investigated during the internship.
- Vicon motion-capture system.
- UAV-mounted monocular RGB camera.
- Linux-based research workstation.

### Software and Technologies

| Category | Technologies |
|---|---|
| Programming languages | C++, Python |
| Robotics middleware | ROS 1 |
| Control methodologies | PID, LMPC |
| State estimation | Luenberger observer, Vicon, ROS TF |
| Numerical computation | Eigen |
| Computer vision | OpenCV, BirdNet3D-derived framework |
| UAV communication | MAVLink, MAVSDK, MAVROS |
| Simulation | PX4, jMAVSim, Parrot Sphinx |
| UAV development interface | Parrot Olympe |
| Dataset annotation | LabelMe |
| Build systems | CMake, Catkin |

These technologies represent the overall research environment. Not all associated software components are distributed in this repository.

## Repository Structure

| File / Directory | Description |
|---|---|
| `src/AR_LMPC.cpp` | Linear model predictive controller |
| `src/AR_PID.cpp` | Vicon-based AR.Drone PID controller |
| `src/Pursuit_vicon.cpp` | Relative localization and pursuit reference generation |
| `src/af_pid.cpp` | ANAFI PID controller |
| `src/af_speed.cpp` | Reference setpoint publisher |
| `src/converter.cpp` | ROS-based attitude conversion |
| `src/eulerangles.cpp` | Attitude conversion prototype |
| `src/eulerangles2.cpp` | Alternative attitude conversion prototype |
| `src/quaternion_euler.py` | Python quaternion-to-Euler conversion |
| `src/first_publisher.cpp` | Experimental Vicon transform processing |
| `src/publisher_quaternion.cpp` | ROS publisher prototype |
| `src/subscriber_quaternion.cpp` | ROS subscriber prototype |
| `src/message.cpp` | Experimental MAVROS communication component |
| `src/mavlink_takeoff.cpp` | MAVSDK flight-operation prototype |
| `src/mavtakeoff.cpp` | Alternative flight-operation prototype |
| `src/takeoff_and_land/` | Separate MAVSDK C++ project |
| `src/anafi.launch` | ROS launch configuration |
| `src/anafi2.launch` | Alternative ROS launch configuration |
| `src/spData.msg` | Reference setpoint message definition |
| `src/traj.msg` | Trajectory and analysis message definition |
| `CMakeLists.txt` | ROS package build configuration |
| `package.xml` | ROS package metadata |

## Individual Research Contributions

The work undertaken during the Mitacs Globalink Research Internship included:

- Development of modular PID-based UAV motion control in C++ and ROS.
- Implementation of quaternion-to-Euler conversion for motion-capture pose processing.
- Investigation of coordinate transformations and reference tracking.
- Development of interactive controller gain-tuning and experimental logging interfaces.
- MAVSDK library configuration and development of flight-operation routines.
- Simulation-based evaluation of flight-control routines using PX4 and jMAVSim.
- Preparation of annotated in-flight UAV imagery for monocular 3D object detection.
- Investigation of existing laboratory methodologies for UAV pursuit and predictive motion control.

## Experimental Evaluation

The broader research involved indoor UAV experiments using Vicon motion capture, alongside simulation-based evaluation.

The research investigated:

- Translational position and yaw tracking.
- Controller response characteristics.
- Classical and predictive motion control.
- Relative position estimation.
- Monocular UAV detection.
- MAVLink-based flight operations.
- Integration of perception and motion control.

Quantitative experimental results and supporting visual documentation are maintained separately from the source-code implementations.

## Build Configuration and Reproducibility

The repository contains two principal build configurations:

| Build configuration | Purpose |
|---|---|
| Root `CMakeLists.txt` | ROS package configuration for the ANAFI PID controller and reference publisher |
| `src/takeoff_and_land/CMakeLists.txt` | Separate C++17 MAVSDK flight-operation project |

The original ROS build configuration defines executable targets for `af_pid.cpp` and `af_speed.cpp`. Other research implementations are retained as experimental modules.

The AR.Drone implementations depend on the earlier laboratory ROS package namespace and associated interfaces. The archived ANAFI PID implementation also contains an inactive Vicon transform lookup.

The original source code and build configurations have been preserved. Consequently, additional dependency configuration, message-directory organization and compatibility adjustments may be necessary to reproduce the historical research environment.

## Research Limitations

- Dependence on laboratory-specific hardware and ROS interfaces.
- Differences between AR.Drone and ANAFI software configurations.
- Limited availability of original experimental flight logs and motion-capture data.
- Absence of the complete perception model and training dataset.
- Limited standalone build reproducibility.
- Additional numerical robustness and safety verification required before physical deployment.

## Author

**Debasmita Chatterjee**

Mitacs Globalink Research Intern — 2022  
Department of Mechanical Engineering  
University of Alberta, Canada

## Disclaimer

This repository is maintained as an archival academic research portfolio. Original experimental implementations and existing attribution have been preserved.

The software is not intended for direct deployment on physical UAVs without appropriate dependency configuration, technical validation and safety verification.
