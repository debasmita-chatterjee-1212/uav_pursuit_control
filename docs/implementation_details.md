# Software Implementation Details

## 1. Overview

This document provides a source-level description of the software modules archived from the UAV pursuit research conducted at the University of Alberta.

The repository contains implementations associated with:

- UAV motion control
- Reference setpoint generation
- Relative pose estimation
- Quaternion and Euler angle conversion
- ROS communication
- MAVLink-based flight operations

The original research code has been preserved. This documentation describes the implemented functionality, experimental interfaces and known limitations without introducing algorithmic modifications.

## 2. Source-Code Organization

| Source File | Primary Function |
|---|---|
| `af_pid.cpp` | ANAFI PID motion-control implementation |
| `af_speed.cpp` | ANAFI reference setpoint publisher |
| `AR_PID.cpp` | Archived AR.Drone PID controller |
| `AR_LMPC.cpp` | Archived AR.Drone predictive controller |
| `Pursuit_vicon.cpp` | Vicon-based relative pose processing and reference generation |
| `converter.cpp` | Quaternion-to-Euler conversion |
| `eulerangles.cpp` | Experimental attitude conversion |
| `eulerangles2.cpp` | Experimental attitude conversion |
| `quaternion_euler.py` | Python-based attitude conversion experiment |
| `first_publisher.cpp` | Vicon transform acquisition |
| `publisher_quaternion.cpp` | ROS string-message publication experiment |
| `subscriber_quaternion.cpp` | ROS string-message subscription experiment |
| `message.cpp` | Experimental MAVROS keyboard-command plugin |
| `mavlink_takeoff.cpp` | MAVSDK flight-operation experiment |
| `mavtakeoff.cpp` | MAVSDK flight-operation experiment |
| `takeoff_and_land/` | Independent MAVSDK application |
| `spData.msg` | Custom reference setpoint message |
| `traj.msg` | Custom trajectory and analysis message |
| `anafi.launch` | Archived ROS launch configuration |
| `anafi2.launch` | Archived ROS launch configuration |

## 3. ANAFI PID Controller

Source: [af_pid.cpp](../src/af_pid.cpp)

### 3.1 Purpose

The ANAFI PID controller investigates four-channel UAV motion control using translational position errors and yaw error.

The implementation uses:

- ROS for communication and timing
- Eigen for vector and matrix operations
- TF for orientation and coordinate transformations
- OpenCV for interactive controller gain adjustment

### 3.2 Principal Functions

| Function | Description |
|---|---|
| `main()` | Initializes the ROS node, communication interfaces, controller parameters and execution loop |
| `spData_Callback()` | Receives and stores reference setpoint information |
| `i2d_gain()` | Converts integer trackbar settings into numerical PID gains |
| `pid_controller()` | Processes pose information and calculates PID control terms |
| `moveFunc()` | Publishes motion commands |
| `analysisFunc()` | Constructs and publishes controller analysis information |

### 3.3 Controller Execution

The controller is structured around the following operations:

1. Initialize the ROS node and communication interfaces.
2. Receive reference setpoint information.
3. Process the available pose and orientation representation.
4. Calculate world-frame position errors.
5. Transform translational errors into the UAV body frame.
6. Calculate and normalize yaw error.
7. Update proportional, integral and derivative terms.
8. Generate translational and yaw commands.
9. Publish controller analysis information.

The nominal controller loop frequency is 25 Hz.

The archived Vicon lookup is commented out. Consequently, the available implementation does not establish a complete live pose-feedback loop.

### 3.4 Interactive Gain Adjustment

The implementation provides 12 OpenCV trackbars for controller tuning.

| Control Channel | Adjustable Parameters |
|---|---|
| X translation | Kp, Ki, Kd |
| Y translation | Kp, Ki, Kd |
| Z translation | Kp, Ki, Kd |
| Yaw | Kp, Ki, Kd |

The trackbar values are converted into floating-point gains using the scaling operations defined in the implementation.

This interface supports interactive investigation of controller behaviour without requiring recompilation for each gain adjustment.

### 3.5 Communication Interfaces

| Interface | Direction | Message Type |
|---|---|---|
| `/anafi/spData` | Subscriber | `anafi_ros1/spData` |
| `/cmd_vel` | Publisher | `geometry_msgs/Twist` |
| `/lmpc_analysis` | Publisher | `anafi_ros1/traj` |

The `/lmpc_analysis` topic name is retained from the historical research framework, although this particular implementation uses PID control.

The implementation also includes CSV output through `Output.csv`.

### 3.6 Implementation Considerations

The archived controller requires further verification before operational use.

Areas requiring attention include:

- Activation and validation of the pose-feedback interface
- Initialization of controller state variables
- Handling of zero or unusually small time intervals
- Conditional logic in the PID calculation
- Integral anti-windup
- Command saturation
- Platform-specific command conventions

These limitations are documented to preserve transparency regarding the current implementation status.

## 4. ANAFI Reference Setpoint Publisher

Source: [af_speed.cpp](../src/af_speed.cpp)

### 4.1 Purpose

This module provides an experimental reference-message publisher for the ANAFI control framework.

It separates reference publication from the controller implementation, allowing the communication interface to be investigated independently.

### 4.2 Communication

| Parameter | Value |
|---|---|
| ROS node | `uav_speed` |
| Published topic | `/anafi/spData` |
| Message type | `anafi_ros1/spData` |
| Publication frequency | 0.5 Hz |

### 4.3 Reference Information

The custom reference message supports:

- Desired X, Y and Z positions
- Desired X, Y and Z velocities
- Desired yaw
- Timing information

The archived publisher assigns fixed position and velocity values.

It should therefore be interpreted as an experimental setpoint publisher rather than a complete trajectory-generation or motion-planning algorithm.

The ANAFI PID controller receives the reference velocity fields, but its principal PID error calculation regulates position and yaw.

## 5. Archived AR.Drone PID Controller

Source: [AR_PID.cpp](../src/AR_PID.cpp)

### 5.1 Purpose

This module contains the foundational PID control implementation associated with the earlier AR.Drone pursuit research.

It provides historical context for the subsequent ANAFI control experiments.

### 5.2 Implementation

The controller includes:

- Vicon-based pose acquisition
- Reference setpoint reception
- Body-frame error processing
- Four-channel PID control
- Interactive gain adjustment
- Motion-command publication
- Controller analysis publication

### 5.3 Communication Interfaces

| Interface | Function |
|---|---|
| `/ar_drone_ros/spData` | Reference setpoint subscription |
| `/cmd_vel` | Motion-command publication |
| `/lmpc_analysis` | Controller analysis publication |

The pose-feedback interface obtains the AR.Drone transform relative to the global map frame.

The controller operates at a nominal frequency of 25 Hz.

## 6. Archived Linear Model Predictive Controller

Source: [AR_LMPC.cpp](../src/AR_LMPC.cpp)

### 6.1 Purpose

The LMPC module implements model-based UAV motion control using a finite prediction horizon and an eight-dimensional state representation.

### 6.2 Principal Functions

| Function | Description |
|---|---|
| `main()` | Initializes the model, ROS interfaces and control loop |
| `lmpc_gain()` | Constructs predictive-control matrices and calculates controller gains |
| `pose()` | Processes pose information, reference states and state estimates |
| `sys_in()` | Calculates predictive control outputs |
| `moveFunc()` | Publishes UAV motion commands |
| `spData_Callback()` | Receives reference setpoint information |
| `S02_mat()` | Constructs a yaw-dependent planar rotation matrix |

### 6.3 Implementation Stages

The predictive-control implementation includes:

1. Definition of the UAV dynamic model.
2. Discretization of the continuous-time model.
3. Construction of the augmented prediction model.
4. Definition of state and control-input weighting matrices.
5. Calculation of predictive-control gains.
6. Processing of pose and reference information.
7. Observer-based state estimation.
8. Calculation of control-input increments.
9. Application of command saturation.
10. Publication of motion commands.

The implementation uses an analytical predictive-control calculation followed by command clipping.

It does not employ a general constrained quadratic-programming solver.

### 6.4 State Estimation

A Luenberger observer is incorporated into the controller to estimate velocity-related states from pose information and previous control inputs.

The implementation also contains filtering and coordinate-transformation operations associated with state processing.

For the corresponding mathematical derivations, see [Mathematical Control Formulation](./control_formulation.md).

## 7. Vicon-Based Pursuit Reference Generation

Source: [Pursuit_vicon.cpp](../src/Pursuit_vicon.cpp)

### 7.1 Purpose

This module processes the motion-capture transforms of two UAVs and generates reference and relative-position information for the archived AR.Drone pursuit framework.

### 7.2 Principal Functions

| Function | Description |
|---|---|
| `main()` | Initializes ROS interfaces and acquires UAV transforms |
| `rel_pose()` | Processes the relative pose of the two UAVs |
| `pubFunc()` | Publishes reference setpoint information |
| `relFunc()` | Publishes relative-position information |

### 7.3 Pose Acquisition

The implementation uses the following TF frames:

- `/map`
- `/vicon/ARDrone_Black/ARDrone_Black`
- `/vicon/ARDrone_White/ARDrone_White`

The two UAV transforms are processed to obtain information relevant to the pursuit task.

### 7.4 Published Information

| Topic | Purpose |
|---|---|
| `/ar_drone_ros/spData` | Pursuit reference setpoints |
| `/ar_drone_ros/relpos` | Relative UAV position information |

The implementation operates at a nominal frequency of 25 Hz.

The original AR.Drone package and its additional message definitions are required to reproduce this module.

## 8. Attitude Representation and ROS Experiments

### 8.1 Quaternion-to-Euler Conversion

Source: [converter.cpp](../src/converter.cpp)

The converter receives a ROS `geometry_msgs/TransformStamped` message and extracts orientation information for conversion into roll, pitch and yaw.

| Interface | Value |
|---|---|
| Subscribed topic | `vicon/bebop/bebop` |
| Published topic | `rpy_angles` |
| Output message type | `geometry_msgs/Vector3` |

The module demonstrates the processing of motion-capture orientation information through a ROS communication interface.

### 8.2 Additional Attitude Conversion Experiments

The repository retains additional experimental implementations:

- [eulerangles.cpp](../src/eulerangles.cpp)
- [eulerangles2.cpp](../src/eulerangles2.cpp)
- [quaternion_euler.py](../src/quaternion_euler.py)

These files represent exploratory approaches to orientation processing.

They are not configured as executable targets in the current root CMake configuration.

### 8.3 Vicon Transform Acquisition

Source: [first_publisher.cpp](../src/first_publisher.cpp)

This module contains a TF lookup for the ANAFI motion-capture frame:

`/vicon/anafi/anafi`

It is retained as part of the historical motion-capture interface experiments.

### 8.4 ROS Communication Experiments

The following modules contain basic ROS publisher and subscriber experiments:

- [publisher_quaternion.cpp](../src/publisher_quaternion.cpp)
- [subscriber_quaternion.cpp](../src/subscriber_quaternion.cpp)

Despite their filenames, these implementations communicate using `std_msgs/String` messages through the `/cpp_msg` topic.

They are not complete quaternion-processing modules.

### 8.5 MAVROS Plugin Experiment

Source: [message.cpp](../src/message.cpp)

This file contains an experimental MAVROS keyboard-command plugin that subscribes to keyboard-command messages.

It is retained as an exploratory communication implementation and is not integrated into the current root build targets.

## 9. MAVSDK Flight-Operation Modules

### 9.1 Purpose

The MAVSDK experiments investigate communication with MAVLink-enabled UAV systems and the execution of basic flight operations.

Relevant source files:

- [mavlink_takeoff.cpp](../src/mavlink_takeoff.cpp)
- [mavtakeoff.cpp](../src/mavtakeoff.cpp)
- [takeoff_and_land](../src/takeoff_and_land/)

### 9.2 Principal Functions

| Function | Description |
|---|---|
| `usage()` | Describes supported connection URL formats |
| `get_system()` | Discovers an available system with an autopilot |
| `main()` | Manages connection, telemetry and flight-operation logic |

### 9.3 Functional Scope

The flight-operation examples contain functionality for:

- MAVLink connection establishment
- Autopilot discovery
- Telemetry monitoring
- Vehicle health checks
- Arming and takeoff
- Landing and flight-state monitoring
- Basic error handling

The independent `takeoff_and_land` application uses MAVSDK Action and Telemetry interfaces.

### 9.4 Connection Configuration

The archived examples contain both command-line connection handling and experimental hard-coded connection settings.

In particular, the independent takeoff-and-land application checks for a command-line argument but uses a hard-coded UDP connection URL.

These settings are specific to the historical experimental environment and require review before reproduction.

### 9.5 Integration Status

The MAVSDK applications are retained as independent flight-operation experiments.

The repository does not establish a verified communication bridge between these applications and the ROS pursuit controllers.

The archived applications should be evaluated in an appropriate simulation environment before any physical flight testing.

## 10. Custom ROS Messages

### 10.1 Reference Setpoint Message

Source: [spData.msg](../src/spData.msg)

| Field | Description |
|---|---|
| `Setpoint_Data` | ROS message header |
| `t` | Reference time information |
| `t_ros` | ROS-related time information |
| `xpos`, `ypos`, `zpos` | Reference position |
| `xvel`, `yvel`, `zvel` | Reference translational velocity |
| `yaw` | Reference yaw |

The message provides a structured interface between reference-generation and motion-control modules.

### 10.2 Trajectory and Analysis Message

Source: [traj.msg](../src/traj.msg)

The trajectory message supports current and predicted state information.

| Field Group | Description |
|---|---|
| `LMPC_Simulation` | ROS message header |
| `xtraj`, `ytraj`, `ztraj` | Current trajectory position |
| `xtraj_vel`, `ytraj_vel`, `ztraj_vel` | Translational velocity |
| `yaw`, `pitch`, `roll` | Orientation |
| `yaw_vel` | Yaw velocity |
| `xtraj_pred`, `ytraj_pred`, `ztraj_pred` | Predicted trajectory position |
| `xtraj_pred_vel`, `ytraj_pred_vel`, `ztraj_pred_vel` | Predicted translational velocity |
| `yaw_pred`, `yaw_vel_pred` | Predicted yaw and yaw velocity |

The message definition is retained from the historical research framework.

Not every field is necessarily populated by every controller.

## 11. Build Configuration

### 11.1 ROS Package

The root CMake configuration defines the ROS package:

`anafi_ros1`

The following executable targets are explicitly configured:

| Executable | Source |
|---|---|
| `uav_pid` | `src/af_pid.cpp` |
| `uav_speed` | `src/af_speed.cpp` |

The remaining source files are retained as historical or experimental implementations and are not explicitly compiled by the root CMake configuration.

### 11.2 Independent MAVSDK Application

The `src/takeoff_and_land/` directory contains a separate CMake configuration for the MAVSDK application.

This application is distinct from the root ROS package.

### 11.3 Build Limitations

The archived repository requires additional configuration before standalone reproduction.

Known considerations include:

- Custom ROS messages are retained under `src/` rather than the conventional `msg/` directory expected by the root message-generation configuration.
- The AR.Drone modules depend on the historical `ar_drone_ros` package.
- Additional AR.Drone message definitions are not included.
- MAVSDK API compatibility depends on the installed library version.
- The original motion-capture and UAV communication environment is not distributed.

Successful compilation and end-to-end execution have not been independently revalidated.

## 12. Research Contribution and Attribution

The ANAFI PID controller and reference publisher represent the individual implementation work documented in this research archive.

The earlier AR.Drone PID, LMPC and pursuit implementations provide foundational research context.

The repository also retains exploratory communication, attitude-processing and flight-operation modules.

## 13. Implementation Status

| Module | Current Status |
|---|---|
| ANAFI PID controller | Archived experimental implementation |
| ANAFI reference publisher | Archived experimental implementation |
| AR.Drone PID controller | Historical research implementation |
| AR.Drone LMPC controller | Historical research implementation |
| Vicon pursuit module | Historical research implementation |
| Attitude conversion modules | Exploratory implementations |
| MAVSDK applications | Independent flight-operation experiments |
| Complete perception-to-control pipeline | Not distributed |
| Standalone build reproducibility | Not independently verified |
| Physical deployment readiness | Not established |

## 14. Related Documentation

- [Project Overview](../README.md)
- [System Architecture](./system_architecture.md)
- [Mathematical Control Formulation](./control_formulation.md)
- [Research Acknowledgements](../ACKNOWLEDGEMENTS.md)
