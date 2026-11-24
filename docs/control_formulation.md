# Mathematical Control Formulation

## 1. Overview

This document describes the mathematical foundations of the UAV motion-control implementations archived in this repository.

The control framework investigates two approaches:

- Proportional-Integral-Derivative (PID) control
- Linear Model Predictive Control (LMPC)

The formulations are based on the archived C++ implementations and the foundational UAV pursuit research conducted at the University of Alberta.

## 2. Coordinate Systems

The control framework considers two principal coordinate systems:

| Coordinate System | Description |
|---|---|
| World frame (W) | Global reference frame associated with the motion-capture environment |
| Body frame (B) | Local reference frame associated with the UAV |

The UAV position in the world frame is represented as:

$$
\mathbf{p}_w =
\begin{bmatrix}
x_w \\ y_w \\ z_w
\end{bmatrix}
$$

The desired reference position is:

$$
\mathbf{p}_r =
\begin{bmatrix}
x_r \\ y_r \\ z_r
\end{bmatrix}
$$

The world-frame position error is:

$$
\mathbf{e}_w = \mathbf{p}_r - \mathbf{p}_w
$$

## 3. Quaternion-Based Attitude Representation

The UAV orientation is represented using a quaternion:

$$
\mathbf{q} =
\begin{bmatrix}
q_w & q_x & q_y & q_z
\end{bmatrix}^{T}
$$

Quaternion-based orientation representations avoid the singularities associated with directly parameterizing orientation using Euler angles.

The corresponding Euler angles are:

- Roll: $\phi$
- Pitch: $\theta$
- Yaw: $\psi$

The standard quaternion-to-Euler conversion for a normalized quaternion is:

$$
\phi =
\operatorname{atan2}
\left(
2(q_wq_x+q_yq_z),
1-2(q_x^2+q_y^2)
\right)
$$

$$
\theta =
\arcsin
\left(
2(q_wq_y-q_zq_x)
\right)
$$

$$
\psi =
\operatorname{atan2}
\left(
2(q_wq_z+q_xq_y),
1-2(q_y^2+q_z^2)
\right)
$$

These equations describe the underlying mathematical conversion. The ANAFI PID implementation obtains Euler angles through the ROS TF rotation-matrix interface.

Relevant source files:

- [converter.cpp](../src/converter.cpp)
- [eulerangles.cpp](../src/eulerangles.cpp)
- [eulerangles2.cpp](../src/eulerangles2.cpp)
- [quaternion_euler.py](../src/quaternion_euler.py)

## 4. World-to-Body Coordinate Transformation

Let $R$ denote the rotation matrix mapping body-frame vectors into the world frame.

The world-frame position error is transformed into the UAV body frame using:

$$
\mathbf{e}_b = R^T\mathbf{e}_w
$$

Therefore:

$$
\mathbf{e}_b =
R^T(\mathbf{p}_r-\mathbf{p}_w)
$$

This transformation allows the controller to express translational position errors relative to the UAV's orientation.

The ANAFI PID implementation uses the rotation obtained from the TF transform, whereas the archived LMPC implementation constructs a yaw-only rotation for its body-frame position and velocity transformations.

## 5. PID Motion Control

### 5.1 Control Error Vector

The four-channel PID controller regulates translational position and yaw.

The error vector is:

$$
\mathbf{e} =
\begin{bmatrix}
e_x \\ e_y \\ e_z \\ e_{\psi}
\end{bmatrix}
$$

The first three components represent body-frame position errors.

The fourth component represents yaw error.

### 5.2 Yaw Error Normalization

The yaw error is initially calculated as:

$$
e_{\psi} = \psi_r-\psi
$$

To account for angular discontinuities, the implementation adjusts the error according to:

$$
e_{\psi} =
\begin{cases}
e_{\psi}+2\pi, & e_{\psi}<-\pi \\
e_{\psi}-2\pi, & e_{\psi}>\pi \\
e_{\psi}, & \text{otherwise}
\end{cases}
$$

This single-step correction handles angular wrapping for the expected range of yaw differences.

### 5.3 Continuous PID Formulation

For each control channel, the standard PID control law is:

$$
u(t)=K_pe(t)+K_i\int_0^t e(\tau)d\tau+
K_d\frac{de(t)}{dt}
$$

where:

| Parameter | Description |
|---|---|
| $K_p$ | Proportional gain |
| $K_i$ | Integral gain |
| $K_d$ | Derivative gain |
| $e(t)$ | Tracking error |
| $u(t)$ | Control output |

### 5.4 Discrete PID Implementation

The discrete integral error is calculated as:

$$
I_k=I_{k-1}+e_k\Delta t
$$

The derivative error is approximated using:

$$
D_k=\frac{e_k-e_{k-1}}{\Delta t}
$$

The resulting control command is:

$$
u_k=K_pe_k+K_iI_k+K_dD_k
$$

The four-channel control vector is:

$$
\mathbf{u}_k =
\begin{bmatrix}
u_x \\ u_y \\ u_z \\ u_{\psi}
\end{bmatrix}
$$

These control outputs are assigned to the translational and yaw components of a ROS `geometry_msgs::Twist` message.

### 5.5 PID Gain Matrices

The four independent control channels can be represented using diagonal gain matrices:

$$
K_p=\operatorname{diag}
(K_{px},K_{py},K_{pz},K_{p\psi})
$$

$$
K_i=\operatorname{diag}
(K_{ix},K_{iy},K_{iz},K_{i\psi})
$$

$$
K_d=\operatorname{diag}
(K_{dx},K_{dy},K_{dz},K_{d\psi})
$$

The ANAFI PID implementation provides interactive adjustment of all 12 gains through OpenCV trackbars.

### 5.6 Initial Controller Gains

The following values are obtained from the initial trackbar settings and the scaling factors used in the main control loop of `af_pid.cpp`.

| Control Axis | Kp | Ki | Kd |
|---|---:|---:|---:|
| X | 4.2 | 0.0062 | 3.6 |
| Y | 2.1 | 0.0060 | 2.2 |
| Z | 3.5 | 0.0030 | 1.1 |
| Yaw | 4.0 | 0.0070 | 0.1 |

These values describe the archived initial configuration and should not be interpreted as validated gains for arbitrary UAV platforms.

### 5.7 Error Deadband

The implementation suppresses the proportional, integral and derivative output terms when:

$$
|e_k|\leq 0.01
$$

The deadband is intended to reduce corrective commands for small tracking errors.

### 5.8 Control Frequency

The nominal ROS control frequency is:

$$
f_s=25\ \mathrm{Hz}
$$

The corresponding nominal sampling interval is:

$$
T_s=\frac{1}{f_s}=0.04\ \mathrm{s}
$$

The PID implementation also calculates elapsed time between iterations using ROS timestamps.

Relevant source files:

- [AR_PID.cpp](../src/AR_PID.cpp)
- [af_pid.cpp](../src/af_pid.cpp)

## 6. Linear Model Predictive Control

### 6.1 State-Space Representation

The archived LMPC implementation uses an eight-dimensional UAV state representation:

$$
\mathbf{x} =
\begin{bmatrix}
x & y & z & \psi &
v_x & v_y & v_z & \dot{\psi}
\end{bmatrix}^{T}
$$

The control input contains four components:

$$
\mathbf{u} =
\begin{bmatrix}
u_x & u_y & u_z & u_{\psi}
\end{bmatrix}^{T}
$$

A continuous-time linear state-space model can be expressed as:

$$
\dot{\mathbf{x}}=A_c\mathbf{x}+B_c\mathbf{u}
$$

The corresponding discrete-time model is:

$$
\mathbf{x}_{k+1}=A_d\mathbf{x}_k+B_d\mathbf{u}_k
$$

### 6.2 Model Discretization

The discrete state-transition matrix is obtained using the matrix exponential:

$$
A_d=e^{A_cT_s}
$$

The discrete input matrix is:

$$
B_d=\int_0^{T_s}e^{A_c\tau}B_c\,d\tau
$$

The archived implementation calculates these matrices using Eigen matrix-exponential operations and an augmented block matrix.

### 6.3 Augmented State Model

The LMPC implementation formulates the prediction model using control-input increments:

$$
\Delta\mathbf{u}_k =
\mathbf{u}_k-\mathbf{u}_{k-1}
$$

An augmented state vector is defined as:

$$
\mathbf{z}_k =
\begin{bmatrix}
\mathbf{x}_k \\
\mathbf{u}_{k-1}
\end{bmatrix}
$$

The augmented model becomes:

$$
\mathbf{z}_{k+1}
=
\widetilde{A}\mathbf{z}_k+
\widetilde{B}\Delta\mathbf{u}_k
$$

where:

$$
\widetilde{A}=
\begin{bmatrix}
A_d & B_d \\
0 & I
\end{bmatrix}
$$

$$
\widetilde{B}=
\begin{bmatrix}
B_d \\
I
\end{bmatrix}
$$

The augmented state dimension is 12.

### 6.4 Prediction Horizon

The archived controller uses a prediction horizon of:

$$
N_p=30
$$

With a nominal sampling interval of 0.04 seconds, the nominal prediction duration is:

$$
T_p=N_pT_s=1.2\ \mathrm{s}
$$

### 6.5 Objective Function

The predictive control objective penalizes predicted state-tracking errors and changes in control input.

A representative mathematical formulation consistent with the implemented prediction matrices is:

$$
J=
\sum_{i=1}^{N_p}
\left[
(\mathbf{x}_{k+i}-\mathbf{r}_{k+i})^T
Q
(\mathbf{x}_{k+i}-\mathbf{r}_{k+i})
+
\Delta\mathbf{u}_{k+i-1}^{T}
R
\Delta\mathbf{u}_{k+i-1}
\right]
$$

where:

- $Q$ is the state-tracking weight matrix.
- $R$ is the control-increment weight matrix.
- $\mathbf{r}$ is the reference state.
- $N_p$ is the prediction horizon.

### 6.6 Weight Matrices

The archived LMPC implementation uses the following diagonal state-weight matrix:

$$
Q=\operatorname{diag}
(3.25,3.75,3.25,10,4.25,4.75,1.75,0)
$$

The control-increment weight matrix is:

$$
R=\operatorname{diag}(1,1,1,2)
$$

The values are retained from the archived implementation.

### 6.7 Prediction and Optimization

The stacked augmented-state prediction can be represented as:

$$
\mathbf{Z}
=
\mathcal{A}\mathbf{z}_k+
\mathcal{B}\Delta\mathbf{U}
$$

The implementation constructs block prediction matrices and calculates a predictive control gain using a matrix inverse.

Using the matrix notation corresponding to the source implementation:

$$
G_{\mathrm{LMPC}}=-H^{-1}F
$$

The control-increment sequence is calculated from the augmented current state and reference trajectory.

The first predicted control input is selected for command generation.

### 6.8 Control-Input Saturation

The resulting control outputs are clipped to the interval:

$$
-1\leq u_i\leq1
$$

This saturation is applied after calculating the predictive control solution.

The archived implementation therefore uses an unconstrained analytical predictive-control calculation followed by command clipping, rather than a constrained quadratic-programming solver.

Relevant source file:

- [AR_LMPC.cpp](../src/AR_LMPC.cpp)

## 7. Luenberger Observer

### 7.1 State Estimation

The LMPC implementation incorporates a Luenberger observer to estimate velocity-related states from pose measurements and previous control inputs.

The measurement model is:

$$
\mathbf{y}_k=C_o\mathbf{x}_k
$$

The observer measurement matrix is:

$$
C_o=
\begin{bmatrix}
I_4 & 0_{4\times4}
\end{bmatrix}
$$

This matrix selects the four position and yaw components of the state vector.

### 7.2 Observer Formulation

A conventional discrete Luenberger observer is expressed as:

$$
\hat{\mathbf{x}}_{k+1}
=
A_d\hat{\mathbf{x}}_k+
B_d\mathbf{u}_k+
K_o(\mathbf{y}_k-C_o\hat{\mathbf{x}}_k)
$$

The archived implementation uses an alternative observer-gain sign convention:

$$
A_{\mathrm{obs}}=A_d+LC_o
$$

The corresponding observer update is:

$$
\hat{\mathbf{x}}_{k+1}
=
(A_d+LC_o)\hat{\mathbf{x}}_k+
B_d\mathbf{u}_k-
L\mathbf{y}_k
$$

This is equivalent to the conventional formulation when:

$$
K_o=-L
$$

The implementation uses an explicitly defined observer gain matrix of dimension:

$$
L\in\mathbb{R}^{8\times4}
$$

The source processes previous measurements and state estimates in the relevant body-frame representation before applying the observer update.

## 8. Comparison of Control Approaches

| Characteristic | PID | LMPC |
|---|---|---|
| Control principle | Error-based feedback | Model-based prediction |
| Explicit dynamic model | Not required | Required |
| Prediction horizon | None | 30 steps |
| Control channels | 4 | 4 |
| Tuning parameters | PID gains | State and control weights |
| State estimation | Pose-processing interface | Luenberger observer |
| Command constraints | No explicit saturation in ANAFI PID | Post-calculation saturation |
| Computational complexity | Generally lower | Generally higher |

The table describes the implemented methodologies and their general characteristics. It does not constitute an experimental performance comparison.

## 9. Implementation Scope and Limitations

The mathematical formulations describe the intended control methodologies and their relationship to the archived source code.

The original research implementations have been preserved without numerical or algorithmic modifications.

Important implementation considerations include:

- The ANAFI PID Vicon transform lookup is inactive.
- Initial PID calculations require additional numerical safeguards.
- The archived PID implementation requires verification of state initialization and time-step handling.
- The ANAFI PID implementation does not explicitly implement integral anti-windup or command saturation.
- The LMPC implementation applies input saturation after the analytical optimization.
- Controller stability and robustness have not been independently revalidated.
- The archived configurations should not be treated as deployment-ready flight controllers.
