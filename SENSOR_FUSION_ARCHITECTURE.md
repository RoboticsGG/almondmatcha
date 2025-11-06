# Sensor Fusion Architecture for Autonomous Mobile Rover

**Document Version:** 1.0  
**Date:** November 6, 2025  
**Status:** Architecture Finalized, Implementation Pending  
**Author:** GitHub Copilot with User Input

---

## Executive Summary

The Almondmatcha rover system implements a **two-domain ROS2 architecture optimized for centralized Extended Kalman Filter (EKF) sensor fusion**:

- **Domain 5 (Rover Internal):** All rover-internal processing with direct low-latency sensor access
- **Domain 2 (Bridge Only):** Base station communication bridge

This architecture enables **advanced autonomous capabilities** through multi-sensor fusion:
- **State Estimation:** Position (x, y), heading (θ), linear (vx, vy) and angular (ωz) velocities
- **Sensor Integration:** IMU, GNSS, encoders, vision (lane offset)
- **Fault Detection:** Sensor anomaly detection through covariance monitoring
- **Predictive Control:** Velocity and position prediction for trajectory planning

---

## Why Domain 5 for Rover Internals?

### Problem with Mixed-Domain Architecture

Previous architecture had sensors and control spread across Domains 2, 5, 6:
- Vision nodes publishing steering commands on Domain 2
- Chassis controller subscribing on Domain 2 but publishing to Domain 5
- GNSS nodes on Domain 2, requiring relay to Domain 5
- **Result:** Network relay overhead, cross-domain latency, fragmented sensor access

### Solution: Centralized Domain 5

**All rover-internal nodes on Domain 5 provides:**

1. **Direct Sensor Access:** EKF node can subscribe directly to all sensor streams without bridges
2. **Low Latency:** No cross-domain relay delays (eliminates 10-50 ms overhead per bridge hop)
3. **Unified Timestamp:** All sensors on same domain enable synchronized time-stamping
4. **Atomic Updates:** Transaction-like semantics for multi-sensor fusion cycles
5. **Scalability:** Easy to add new sensors (lidar, radar, depth cameras) without architecture changes

---

## Architecture Overview

```
┌────────────────────────────────────────────────────────────────────┐
│                   SENSOR FUSION ARCHITECTURE                       │
│                    Domain 5 (Rover Internal)                       │
└────────────────────────────────────────────────────────────────────┘

    ┌─────────────────────────────────────────────────────────────┐
    │                    SENSOR LAYER                             │
    │  (STM32 boards + initial processing on Raspberry Pi)        │
    └─────────────────────────────────────────────────────────────┘
              │              │              │              │
              ▼              ▼              ▼              ▼
        ┌────────────┐ ┌────────────┐ ┌──────────┐ ┌────────────┐
        │   IMU      │ │   GNSS     │ │ Encoders │ │   Vision   │
        │ 10 Hz      │ │ 10 Hz      │ │ 10 Hz    │ │ 30 FPS     │
        └────────────┘ └────────────┘ └──────────┘ └────────────┘
              │              │              │              │
              └──────────────┬──────────────┴──────────────┘
                             │
                             ▼
    ┌─────────────────────────────────────────────────────────────┐
    │                  PREPROCESSING LAYER                        │
    │  (Sensor processing, calibration, outlier rejection)        │
    │  • node_chassis_imu: IMU calibration, noise filtering       │
    │  • node_chassis_sensors: Encoder processing, bias removal   │
    │  • lane_detection: Vision feature extraction                │
    │  • node_gnss_spresense: GPS fix quality assessment          │
    └─────────────────────────────────────────────────────────────┘
              │              │              │              │
              └──────────────┬──────────────┴──────────────┘
                             │
                             ▼
    ┌─────────────────────────────────────────────────────────────┐
    │            FUSION ESTIMATION LAYER (FUTURE)                 │
    │       Extended Kalman Filter (EKF) - node_ekf_fusion        │
    │                                                              │
    │  State: [x, y, θ, vx, vy, ωz, imu_bias_ax, imu_bias_ay]   │
    │  Covariance: Diagonal 8x8 uncertainty matrix                │
    │  Update Rate: 10 Hz (synchronized with slowest sensor)      │
    │  Measurement Innovation: [IMU, GNSS, Odometry, Vision]      │
    │                                                              │
    │  EKF Cycle:                                                  │
    │    1. Prediction: Motion model + IMU integration (1 ms)     │
    │    2. Measurement Update: GNSS, odometry, vision (9 ms)     │
    │    3. Covariance Update: Innovation covariance (1 ms)       │
    │    4. Publish: Fused state estimate                         │
    └─────────────────────────────────────────────────────────────┘
              │
              ▼
    ┌─────────────────────────────────────────────────────────────┐
    │                   DECISION LAYER                            │
    │  (Autonomous control algorithms using fused state)          │
    │  • Motion planning using predicted trajectory               │
    │  • Adaptive control gains based on covariance               │
    │  • Fault detection through innovation statistics            │
    │  • Sensor health monitoring                                 │
    └─────────────────────────────────────────────────────────────┘
              │
              └──────────────┬──────────────────┬──────────────────┐
                             │                  │                  │
                             ▼                  ▼                  ▼
                    ┌──────────────┐  ┌──────────────┐  ┌──────────────┐
                    │  Chassis     │  │  GNSS Mgmt   │  │  Vision Nav  │
                    │  Control     │  │  (Mission)   │  │  (Steering)  │
                    └──────────────┘  └──────────────┘  └──────────────┘
                             │                  │                  │
                             └──────────────────┼──────────────────┘
                                                │
                                                ▼
                    ┌─────────────────────────────────────────┐
                    │     Base Station Bridge Node            │
                    │   (Domain 5 ↔ Domain 2)                │
                    │   node_base_bridge                      │
                    │                                         │
                    │  Relays:                                │
                    │  • Telemetry: Fused state + health    │
                    │  • Commands: Mission waypoints          │
                    │  • Diagnostics: EKF covariance         │
                    └─────────────────────────────────────────┘
                                                │
                                                ▼
                                    ┌──────────────────────┐
                                    │   Ground Station     │
                                    │   (Domain 2)         │
                                    │   ws_base            │
                                    └──────────────────────┘
```

---

## Domain Assignments (Corrected Architecture)

### Domain 5 - Rover Internal (All Rover Nodes)

| Node | Type | IP | Purpose | Domain |
|------|------|----|---------|----|
| **Raspberry Pi (ws_rpi)** | Computer | 192.168.1.1 | Central processing | **5** |
| ├─ node_chassis_controller | ROS2 | - | Motor command coordination | **5** |
| ├─ node_chassis_imu | ROS2 | - | IMU preprocessing | **5** |
| ├─ node_chassis_sensors | ROS2 | - | Encoder/power preprocessing | **5** |
| ├─ node_gnss_spresense | ROS2 | - | GNSS positioning | **5** |
| ├─ node_gnss_mission_monitor | ROS2 | - | Mission waypoint tracking | **5** |
| ├─ **[FUTURE] node_ekf_fusion** | ROS2 | - | Centralized sensor fusion | **5** |
| └─ node_base_bridge | ROS2 | - | D5 ↔ D2 bridge | 5+2 |
| **Jetson Orin Nano (ws_jetson)** | Computer | 192.168.1.5 | Vision processing | **5** |
| ├─ camera_stream_node | ROS2 | - | D415 RGB/depth streaming | **5** |
| ├─ lane_detection_node | ROS2 | - | Lane feature extraction | **5** |
| └─ steering_control_node | ROS2 | - | PID steering control | **5** |
| **STM32 Chassis (mros2-mbed-chassis-dynamics)** | MCU | 192.168.1.2 | Motor control + IMU | **5** |
| └─ chassis_controller | mROS2 | - | Real-time control | **5** |
| **STM32 Sensors (mros2-mbed-sensors-gnss)** | MCU | 192.168.1.6 | Sensors + GNSS | **5** |
| └─ sensors_node | mROS2 | - | Real-time sensor reading | **5** |

### Domain 2 - Base Station Bridge Only

| Node | Type | Purpose | Domain |
|------|------|---------|--------|
| **node_base_bridge** (ws_rpi) | ROS2 Multi-Domain | Relay telemetry/commands | 2 (Primary) + 5 (Secondary) |
| **ws_base (Ground Station)** | ROS2 | Telemetry monitoring | **2** |

---

## Topic Architecture

### Domain 5 - Rover Internal Topics (Flat Namespace)

#### Sensor Streams (Raw Data)

| Topic | Type | Rate | Publisher | Subscribers |
|-------|------|------|-----------|-------------|
| `/tpc_chassis_imu` | ChassisIMU (accel, gyro) | 10 Hz | chassis_controller (STM32) | node_chassis_imu, node_ekf_fusion |
| `/tpc_chassis_sensors` | ChassisSensors (encoders, power) | 10 Hz | sensors_node (STM32) | node_chassis_sensors, node_ekf_fusion |
| `/tpc_gnss_spresense` | SpresenseGNSS (lat, lon, alt, fix) | 10 Hz | node_gnss_spresense | node_gnss_mission_monitor, **node_ekf_fusion** |
| `/tpc_rover_d415_rgb` | sensor_msgs/Image | 30 FPS | camera_stream_node | lane_detection_node |
| `/tpc_rover_d415_depth` | sensor_msgs/Image | 30 FPS | camera_stream_node | (reserved) |

#### Processed Sensor Streams

| Topic | Type | Rate | Publisher | Subscribers |
|-------|------|------|-----------|-------------|
| `/tpc_chassis_imu_processed` | ChassisIMU (calibrated) | 10 Hz | node_chassis_imu | **node_ekf_fusion**, (future: diagnostics) |
| `/tpc_chassis_sensors_processed` | ChassisSensors (processed) | 10 Hz | node_chassis_sensors | **node_ekf_fusion**, (future: diagnostics) |
| `/tpc_rover_nav_lane` | Float32MultiArray [theta, b, conf] | 30 FPS | lane_detection_node | steering_control_node, **node_ekf_fusion** |

#### Fusion Output (Future)

| Topic | Type | Rate | Publisher | Subscribers |
|-------|------|------|-----------|-------------|
| **`/tpc_ekf_state_estimate`** | **Fused State** | **10 Hz** | **node_ekf_fusion** | **all control nodes** |
| **`/tpc_ekf_covariance`** | **Uncertainty Matrix** | **10 Hz** | **node_ekf_fusion** | **diagnostics, health monitoring** |

#### Control Commands

| Topic | Type | Rate | Publisher | Subscribers |
|-------|------|------|-----------|-------------|
| `/tpc_rover_fmctl` | Float32MultiArray [steering, detect] | 50 Hz | steering_control_node | node_chassis_controller |
| `/tpc_chassis_ctrl_d5` | ChassisCtrl (fdr, ro_ctrl, spd, bdr) | 50 Hz | node_chassis_controller | chassis_controller (STM32) |
| `/tpc_gnss_mission_active` | Bool | 10 Hz | node_gnss_mission_monitor | node_chassis_controller |
| `/tpc_gnss_mission_remain_dist` | Float64 | 10 Hz | node_gnss_mission_monitor | (diagnostics) |

### Domain 2 - Base Station Bridge Topics

| Topic | Type | Rate | Publisher | Subscribers |
|-------|------|------|-----------|-------------|
| `/tpc_telemetry` | TelemetryMsg (aggregated) | 10 Hz | node_base_bridge | mission_monitoring_node (ws_base) |
| `/tpc_command` | CommandMsg (waypoints, modes) | Event-driven | mission_control_node (ws_base) | node_base_bridge |

---

## Extended Kalman Filter (EKF) Specification

### State Vector

$$\mathbf{x} = \begin{bmatrix} x \\ y \\ \theta \\ v_x \\ v_y \\ \omega_z \\ b_{ax} \\ b_{ay} \end{bmatrix} \in \mathbb{R}^8$$

- **Position:** $(x, y)$ - global frame [meters]
- **Heading:** $\theta$ - yaw angle [radians]
- **Velocities:** $(v_x, v_y)$ - body frame [m/s]
- **Angular Rate:** $\omega_z$ - yaw rate [rad/s]
- **IMU Biases:** $(b_{ax}, b_{ay})$ - accelerometer biases [m/s²]

### Measurement Models

#### IMU Measurement
$$\mathbf{z}_{imu} = \begin{bmatrix} a_x \\ a_y \\ \omega_z \end{bmatrix} = \begin{bmatrix} \dot{v}_x - b_{ax} \\ \dot{v}_y - b_{ay} \\ \omega_z \end{bmatrix} + \mathbf{w}_{imu}, \quad \mathbf{w}_{imu} \sim \mathcal{N}(0, \mathbf{R}_{imu})$$

#### GNSS Measurement
$$\mathbf{z}_{gnss} = \begin{bmatrix} x \\ y \end{bmatrix} + \mathbf{w}_{gnss}, \quad \mathbf{w}_{gnss} \sim \mathcal{N}(0, \mathbf{R}_{gnss})$$

Where $\mathbf{R}_{gnss}$ depends on GPS fix quality (DOP):
- Fix quality "good": $\sigma = 0.5$ m
- Fix quality "degraded": $\sigma = 2.0$ m
- Fix quality "poor": measurement rejected

#### Odometry Measurement
$$\mathbf{z}_{odo} = \begin{bmatrix} v_x \\ v_y \end{bmatrix} = \begin{bmatrix} \omega_L r + \omega_R r / 2 \\ \text{lateral component} \end{bmatrix} + \mathbf{w}_{odo}$$

Where $\omega_L, \omega_R$ are encoder angular velocities, $r$ is wheel radius.

#### Vision Measurement
$$\mathbf{z}_{vision} = b = \text{lane offset [pixels]} \Rightarrow \text{transverse position error [m]}$$

### Motion Model (Constant Velocity with IMU Integration)

$$\mathbf{x}_{k+1} = \mathbf{f}(\mathbf{x}_k, \mathbf{u}_k) + \mathbf{w}_k$$

**Time discretization (100 ms steps):**

$$\begin{align}
x_{k+1} &= x_k + v_x \cos(\theta_k) \Delta t - v_y \sin(\theta_k) \Delta t \\
y_{k+1} &= y_k + v_x \sin(\theta_k) \Delta t + v_y \cos(\theta_k) \Delta t \\
\theta_{k+1} &= \theta_k + \omega_z \Delta t \\
v_x &= v_x + (a_x - b_{ax}) \Delta t \\
v_y &= v_y + (a_y - b_{ay}) \Delta t \\
\omega_z &= \text{constant} \quad \text{(or integrated from gyro)} \\
b_{ax} &= b_{ax} + w_{b_{ax}} \\
b_{ay} &= b_{ay} + w_{b_{ay}}
\end{align}$$

### Covariance Management

**Initial covariance (at startup):**
$$\mathbf{P}_0 = \text{diag}(10^2, 10^2, (\pi/4)^2, 1^2, 1^2, (0.5)^2, (0.1)^2, (0.1)^2)$$

**Measurement noise covariances:**
$$\mathbf{R}_{imu} = \text{diag}(0.01^2, 0.01^2, 0.02^2) \quad \text{[m/s², m/s², rad/s]}$$
$$\mathbf{R}_{gnss} = \text{diag}(0.5^2, 0.5^2) \quad \text{[m, m]} \quad \text{(adaptively scaled)}$$
$$\mathbf{R}_{odo} = \text{diag}(0.05^2, 0.05^2) \quad \text{[m/s, m/s]}$$

**Process noise covariances:**
$$\mathbf{Q} = \text{diag}(0.01^2, 0.01^2, 0.01^2, 0.1^2, 0.1^2, 0.05^2, 0.001^2, 0.001^2)$$

### Innovation Monitoring (Fault Detection)

For each measurement, compute innovation:
$$\mathbf{y}_k = \mathbf{z}_k - \mathbf{h}(\mathbf{x}_{k|k-1})$$

Mahalanobis distance:
$$D_k^2 = \mathbf{y}_k^T (\mathbf{S}_k)^{-1} \mathbf{y}_k$$

**Anomaly detection:**
- If $D_k^2 > \chi^2_{threshold}$ (typically 9 for 2-DOF), flag sensor as outlier
- Skip measurement update for flagged sensors
- Log anomaly for diagnostics

---

## Future EKF Implementation Plan

### Phase 1: Infrastructure (Weeks 1-2)
- [ ] Create `node_ekf_fusion` ROS2 node with basic EKF implementation
- [ ] Define message types for fused state and covariance
- [ ] Implement synchronization mechanism for multi-rate sensors
- [ ] Unit test EKF math (prediction, update, covariance)

### Phase 2: Sensor Integration (Weeks 3-4)
- [ ] Integrate IMU measurement model
- [ ] Integrate GNSS measurement model with adaptive covariance
- [ ] Integrate odometry measurement model
- [ ] Implement vision-to-distance transformation

### Phase 3: Validation & Tuning (Weeks 5-6)
- [ ] Field testing on rover
- [ ] Empirical noise covariance estimation
- [ ] Calibration of sensor biases
- [ ] Performance metrics: position error, heading error, filter consistency

### Phase 4: Advanced Features (Weeks 7+)
- [ ] Multi-hypothesis tracking for ambiguous lane detections
- [ ] Adaptive process noise (handle different terrain)
- [ ] Sensor failure recovery modes
- [ ] Real-time diagnostics dashboard

---

## Testing & Validation

### Pre-Deployment Checklist

- [ ] All Domain 5 nodes communicate without relay overhead
- [ ] Topic latencies verified (< 50 ms end-to-end)
- [ ] GNSS data available on Domain 5 at 10 Hz
- [ ] IMU data available on Domain 5 at 10 Hz
- [ ] Encoder data available on Domain 5 at 10 Hz
- [ ] Vision lane data available on Domain 5 at 30 FPS
- [ ] node_base_bridge successfully bridges Domain 2 ↔ 5
- [ ] ws_base receives telemetry on Domain 2
- [ ] No cross-domain latency in critical paths

### Performance Metrics

| Metric | Target | Method |
|--------|--------|--------|
| Fusion latency | < 20 ms | Topic echo timestamps |
| GPS fix quality | > 95% | Fix quality statistics |
| Odometry accuracy | < 5% drift | Closed loop test |
| Vision stability | 5-10 Hz | Lane detection frequency |
| Network overhead | < 5% | DDS traffic monitoring |

---

## Migration Path from Current to Sensor Fusion

### Step 1: Verify Domain 5 Architecture ✅ (DONE)
- All rover nodes moved to Domain 5
- Base bridge on Domain 2 only
- No cross-domain latency

### Step 2: Implement node_base_bridge (TODO)
- Multi-domain node subscribing to Domain 5, publishing to Domain 2
- Aggregate sensor telemetry
- Relay base station commands

### Step 3: Deploy node_ekf_fusion (TODO)
- Subscribe to all preprocessed sensors on Domain 5
- Implement EKF state estimation
- Publish fused state estimate
- Monitor covariance for diagnostics

### Step 4: Integrate with Control Nodes (TODO)
- Update mission monitor to use fused position instead of raw GPS
- Update chassis controller to use fused heading for steering
- Update vision navigation with predicted trajectory

---

## Conclusion

The **two-domain architecture (Domain 5 rover-internal, Domain 2 bridge-only)** provides the foundation for advanced autonomous capabilities through centralized sensor fusion. This architecture:

1. **Eliminates fragmentation:** All sensors and control on same domain
2. **Reduces latency:** Direct communication without bridges
3. **Enables fusion:** EKF and other fusion algorithms have direct sensor access
4. **Scales naturally:** New sensors add topics, not new domains
5. **Improves autonomy:** Fused state enables prediction and adaptive control

The Extended Kalman Filter implementation will transform the rover from reactive control (steering to lane markers) to predictive autonomous navigation (planning trajectories using fused estimates).

---

**References:**
- Thrun, Burgard, Fox. "Probabilistic Robotics" (2005)
- ROS2 Multi-Domain Documentation
- embeddedRTPS/mROS2 Configuration Guide

**Next Review Date:** December 1, 2025  
**Assigned To:** Development Team
