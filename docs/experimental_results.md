# Experimental Results and Discussion

## 1. Overview

This document summarizes representative experimental outcomes described in the 2022 summer training report on vision-based UAV pursuit control.

The results concern the broader research framework investigated at the University of Alberta.

The numerical values are transcribed from the internship report and have not been independently reproduced using the source code in this repository.

## 2. Evaluation Methodology

The internship report describes evaluation in simulated and controlled indoor environments.

The evaluation considered:

- Position-tracking performance
- Controller transient response
- Disturbance rejection
- Monocular target detection
- Relative depth estimation
- Closed-loop pursuit behaviour

Vicon motion capture provided ground-truth pose information for the indoor experimental framework.

## 3. Motion-Control Evaluation

The report describes empirical PID tuning using simulated step-response data and comparison with LMPC on trajectory-tracking tasks under simulated disturbances.

### 3.1 Reported Controller Performance

| Performance Metric | PID | LMPC |
|---|---:|---:|
| Steady-state position error (cm, mean +/- standard deviation) | 7.8 +/- 3.1 | 5.2 +/- 2.4 |
| Rise time (s) | 1.8 | 1.2 |
| Overshoot (%) | 12.4 | 7.3 |

Source: Summer Training Report, Results and Discussion, p. 10 (printed page numbering).

### 3.2 Relative Performance Comparison

Using the representative values reported above:

| Metric | Relative Reduction with LMPC |
|---|---:|
| Mean steady-state position error | 33.3% |
| Rise time | 33.3% |
| Overshoot | 41.1% |

The percentages are calculated from the reported representative values.

They do not establish statistical significance or generalizable superiority across operating conditions.

### 3.3 Discussion

The reported comparison indicates lower tracking error, shorter rise time and reduced overshoot for LMPC under the evaluated conditions.

The report also identifies greater computational requirements and tuning effort associated with predictive control.

PID control remains relevant for resource-constrained platforms because of its simpler implementation and lower computational complexity.

## 4. Vision-Based Tracking Evaluation

### 4.1 Dataset

The internship report describes a dataset of approximately 2,000 annotated image frames acquired from the pursuer UAV.

The annotation procedures investigated keypoints, structural component labels and bounding boxes.

### 4.2 Reported Detection Performance

| Metric | Reported Value |
|---|---|
| Annotated dataset size | Approximately 2,000 frames |
| Average precision | 84.6% |
| Intersection-over-Union threshold | 0.5 |
| Depth estimation error | 8-12 cm |
| Evaluated depth range | 3-5 m |

Source: Summer Training Report, Results and Discussion, pp. 10-11.

### 4.3 Qualitative Observations

The report describes target detection under moderate lighting variations and partial occlusions.

The research investigated the feasibility of monocular 3D perception as a source of relative position information for UAV pursuit.

The complete training dataset, trained model and evaluation scripts are not distributed in this repository.

## 5. Closed-Loop Pursuit Evaluation

The internship report describes indoor pursuit experiments in which estimated relative position agreed with Vicon ground truth to within approximately 10 cm over short time horizons of approximately five seconds.

Minor oscillations were observed during rapid yaw changes.

Communication latency between perception and control was identified as a contributing factor.

These observations concern the broader experimental framework and do not establish that the archived ANAFI PID source independently reproduces the same behaviour.

## 6. MAVSDK Simulation Evaluation

The report describes successful simulation of a basic MAVSDK-based flight-operation sequence using PX4 and jMAVSim.

The simulation investigated:

- MAVLink communication
- Autopilot discovery
- Telemetry processing
- Flight-operation commands

No quantitative flight-operation timing or reliability statistics are included in the repository.

## 7. Experimental Limitations

The internship report identifies several limitations:

- Short-duration indoor experiments
- Limited training dataset size
- Onboard computational constraints
- Real-time perception latency
- Communication delays
- Linearized UAV dynamic modelling
- Sensitivity to lighting and motion blur
- Limited evaluation of outdoor generalization

Additional reproducibility limitations include the absence of raw experimental logs, exact dependency versions and the complete perception pipeline.

## 9. Reproducibility Status

| Evaluation Component | Status |
|---|---|
| Representative report-derived results | Documented |
| Raw controller response logs | Not distributed |
| Ground-truth Vicon recordings | Not distributed |
| Complete image dataset | Not distributed |
| Trained model weights | Not distributed |
| Independent statistical analysis | Not available |
| Independent reproduction | Not established |

## 10. Future Experimental Work

Potential extensions include:

- Repeated trajectory-tracking experiments
- Controlled disturbance-response evaluation
- Statistical analysis across multiple trials
- Evaluation under varying communication latency
- Robustness testing under perception uncertainty
- Comparison of alternative state-estimation methods
- Nonlinear and learning-based predictive control
- Outdoor and GPS-denied pursuit evaluation
