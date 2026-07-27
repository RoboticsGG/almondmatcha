# Control Law

Steering and speed control law implemented across the vision (Jetson, D6/D5)
and chassis (RPi, D5) nodes. For network/domain topology see
[ARCHITECTURE.md](ARCHITECTURE.md); for topic schemas see
[TOPICS.md](TOPICS.md).

## Overview

Control is split across two processes on two machines, cascaded through one
shared command topic:

1. **`rover_kinematic_control_node.py` (Jetson, D6→D5)** — turns lane
   geometry into a steering angle and a speed *request*. Runs at the lane
   detection rate.
2. **`chassis_controller_node.cpp` (RPi, D5)** — turns that request into an
   actual motor command. Steering is a straight passthrough (angle → servo
   direction/magnitude). Speed is closed-loop: a PID corrects PWM duty
   against wheel-encoder feedback so terrain load doesn't silently slow the
   rover below the requested speed.

```mermaid
flowchart LR
    subgraph JET["Jetson · D6"]
        LD["lane_detection_node\ncurvature, θ, b, detected"]
    end

    subgraph JETD5["Jetson · D6→D5 bridge"]
        KC["rover_kinematic_control_node\nsteering PID+FF · speed policy"]
    end

    subgraph RPI["RPi · D5"]
        CC["chassis_controller_node\nsafety cap · speed PID"]
        ENC["tpc_chassis_sensors\nwheel encoders ~4 Hz"]
    end

    subgraph STM["STM32 Chassis"]
        MOT["servo + motor driver"]
    end

    LD -- "tpc_rover_nav_lane" --> KC
    KC -- "tpc_rover_ctrl_cmd\n[steer_angle, speed_cmd, detected] 50 Hz" --> CC
    ENC -. "wheel ticks" .-> CC
    CC -- "tpc_chassis_cmd" --> MOT
```

---

## 1. Steering Control Law

**File:** `ws_jetson/src/vision_navigation/vision_navigation/rover_kinematic_control_node.py`
**Config:** `ws_jetson/src/vision_navigation/config/rover_kinematic_control_params.yaml`

Feedback (PID on heading + lateral error) combined with feedforward
(proportional to curve sharpness ahead), so the rover starts steering into a
curve before heading/offset error alone would build up enough to react.

### Inputs (from `tpc_rover_nav_lane`)

| Symbol | Field | Meaning |
|---|---|---|
| `κ` | curvature | Parabola coefficient A of the fitted lane, `x = A·y² + B·y + C` |
| `θ` | theta_deg | Heading error, degrees (+ = needs right turn) |
| `b` | b_offset | Lateral pixel offset from lane center |
| `detected` | detected_flag | Raw vision detection validity |

Each raw input is clamped before filtering to keep a single bad frame from
spiking the low-pass state: `θ ∈ [-35°, 35°]`, `b ∈ [-100, 100] px`.

### 1.1 Low-pass filtering (EMA)

All three signals are smoothed with an exponential moving average before
use, gain `α = ema_alpha`:

$$
x_{\text{ema}}[k] = \alpha \, x[k] + (1-\alpha)\, x_{\text{ema}}[k-1]
$$

applied independently to `θ`, `b`, and `κ`. The filter is treated as
"warmed up" only once its internal history buffer (30 samples) is full;
until then the command falls back to the lost-lane policy even if
`detected = true`.

### 1.2 Feedback term — combined-error PID

$$
e[k] = k_{e1}\,\theta_{\text{ema}}[k] + k_{e2}\,b_{\text{ema}}[k]
$$

$$
u_{\text{pid}}[k] = k_p\,e[k] \;+\; k_i \sum_{j\le k} e[j]\,\Delta t_j \;+\; k_d\,\frac{e[k]-e[k-1]}{\Delta t_k}
$$

Integral term is clamped to `±200` (anti-windup) before being multiplied by `k_i`.

### 1.3 Feedforward term — anticipate the curve

$$
u_{\text{ff}}[k] = k_{ff} \, \kappa_{\text{ema}}[k]
$$

### 1.4 Combined output and saturation

$$
u_{\text{total}}[k] = u_{\text{pid}}[k] + u_{\text{ff}}[k]
$$

$$
\delta_{\text{cmd}}[k] =
\begin{cases}
\text{clamp}\big(u_{\text{total}}[k],\, -\delta_{\text{max}},\, \delta_{\text{max}}\big) & \text{lane detected, filters warmed up} \\
\delta_{\text{lost}} & \text{otherwise}
\end{cases}
$$

`δ_cmd` (degrees, + = right / − = left) is published as `data[0]` of
`tpc_rover_ctrl_cmd`. `chassis_controller_node` on the RPi does no further
shaping — it only maps sign → turn direction (`fdr_msg`) and forwards
`|δ_cmd|` as `ro_ctrl_msg` to the STM32 servo driver.

### 1.5 Block diagram

```mermaid
flowchart LR
    theta["θ raw"] --> clampT["clamp ±35°"] --> emaT["EMA α"] --> sumErr(("Σ"))
    b["b raw"] --> clampB["clamp ±100px"] --> emaB["EMA α"] --> sumErr
    kappa["κ raw"] --> emaK["EMA α"] --> ff["× k_ff"]

    sumErr -- "e = k_e1·θ + k_e2·b" --> pid["PID\nkp·e + ki∫e + kd·de/dt"]
    pid -- u_pid --> sumU(("+"))
    ff -- u_ff --> sumU
    sumU -- u_total --> sat["saturate ±steer_max_deg"]
    lost["lane lost /\nfilters cold"] -. overrides .-> steerWhenLost["steer_when_lost"]
    sat --> out["δ_cmd → tpc_rover_ctrl_cmd[0]"]
    steerWhenLost --> out
```

### 1.6 Current gains (as configured)

| Param | Value | Role |
|---|---|---|
| `k_e1` | 1.0 | heading-error weight |
| `k_e2` | 0.1 | lateral-offset weight |
| `k_p` | 4.0 | proportional |
| `k_i` | 0.01 | integral |
| `k_d` | 0.01 | derivative |
| `k_ff` | 1000.0 | feedforward on curvature (curvature is ~1e-4–1e-3 in the warped pixel frame) |
| `ema_alpha` | 0.05 | filter smoothing |
| `steer_max_deg` | ±60° | output saturation |
| `steer_when_lost` | 0.0° | safety default, straight |

---

## 2. Speed Control Law

Speed is set in two stages: a **request** computed on the Jetson from
detection state, then a **closed-loop correction** applied on the RPi
against wheel-encoder feedback. Both stages share the same unit: 0–100 %
PWM duty cycle (`motor_duty = speed_percent / 100.0` on the STM32).

### 2.1 Stage 1 — detection-based speed request (Jetson)

**File:** `rover_kinematic_control_node.py::_compute_speed_cmd`

$$
v_{\text{cmd}} =
\begin{cases}
v_{ref} & \text{lane detected} \\
v_{ref}\cdot r_{\text{lost}} & \text{lost, elapsed} < T_{\text{timeout}} \\
0 & \text{lost, elapsed} \ge T_{\text{timeout}}
\end{cases}
$$

This is open-loop with respect to actual chassis speed — it only reacts to
*lane visibility*, not to how fast the wheels are actually turning. `v_cmd`
is published as `data[1]` of `tpc_rover_ctrl_cmd`.

| Param | Value | Role |
|---|---|---|
| `speed_ref` | 50 % | nominal cruise duty when lane detected |
| `speed_lost_ratio` | 0.5 | caution-speed multiplier while lane briefly lost |
| `detection_timeout_sec` | 10.0 s | time lost before full stop |

### 2.2 Stage 2 — closed-loop duty correction (RPi)

**File:** `ws_rpi/src/chassis_control/src/chassis_controller_node.cpp`
**Config:** `ws_rpi/src/chassis_control/config/chassis_speed_control_params.yaml`

`v_cmd` is first clamped by the operator safety cap (`srv_spd_limit`):

$$
v_{\text{target}} = \min\big(\text{clamp}(v_{\text{cmd}}, 0, 100),\; v_{\text{cap}}\big)
$$

Wheel-encoder deltas (`tpc_chassis_sensors`, ~4 Hz) give a measured tick
rate:

$$
\dot n_{\text{meas}}[k] = \frac{(\Delta N_L + \Delta N_R)/2}{\Delta t}
$$

> **Why averaging is correct even mid-turn:** both encoders sit on the rear
> axle, and steering is applied only at the front wheels (Ackermann). The
> instantaneous center of rotation therefore always lies on the line through
> the rear axle, so the two rear wheel speeds are exactly `v_center ∓ Δ`,
> symmetric around the true centerline speed, at any steering angle — no
> small-angle approximation needed. Left/right tick-rate divergence while
> turning is expected geometry, not sensor error, and it cancels out exactly
> in `ṅ_meas`. `measured_left_tps`/`measured_right_tps` are published to
> `tpc_chassis_speed_debug` for logging only; the PID never needs to look at
> the split, and no extra tolerance is required for it to regulate forward
> speed correctly through curves. (A per-wheel expected-divergence check —
> useful for detecting a slipping or stuck wheel — would be a separate
> addition; it isn't implemented today.)

The measured rate is then normalised into the same 0–100 % unit as the
output, using the flat-ground calibration constant `ṅ_max`:

$$
v_{\text{meas}}[k] = 100\cdot\frac{\dot n_{\text{meas}}[k]}{\dot n_{\text{max}}}
\qquad
e_v[k] = v_{\text{target}} - v_{\text{meas}}[k]
$$

Working in percent rather than raw ticks/s is deliberate: the gains are then
independent of `ṅ_max`, so re-calibrating it doesn't silently rescale the
loop.

**Feedforward + PID trim.** The commanded duty is itself the feedforward
term; the PID only corrects for load:

$$
u_v[k] = \underbrace{v_{\text{target}}}_{\text{feedforward}} \;+\; \underbrace{k_p^{v}\,e_v[k] \;+\; k_i^{v}\sum_{j\le k} e_v[j]\,\Delta t_j \;+\; k_d^{v}\,\frac{e_v[k]-e_v[k-1]}{\Delta t_k}}_{\text{PID trim}}
$$

At zero error this emits exactly what open-loop would have sent, so a
healthy loop degrades gracefully toward open-loop behaviour rather than
away from it. The integral is bounded by `±speed_integral_limit`, giving it
`k_i × limit` of duty authority (currently ±50 %), and uses conditional
integration — accumulation stops once the output is saturated and the error
would only push it further out of range, so an unreachable setpoint (e.g.
climbing at full throttle) doesn't wind up a lurch for when traction
returns.

> **Why feedforward, not pure PID:** with no feedforward the integral has to
> supply the entire operating point on its own, which makes the anti-windup
> clamp double as a ceiling on reachable speed. That was a real defect here:
> at the original `speed_ki = 0.01` with `speed_integral_limit = 200` the integral could
> contribute at most 2 % duty, so a commanded 50 % settled at ~18 % — about
> a third of the intended speed, and *worse than the open-loop path it
> replaced*. No value of `ṅ_max` fixes it; the structure has to change.

Final outgoing duty, safety cap applied last regardless of PID overshoot:

$$
\text{duty}[k] = \min\Big(\text{clamp}\big(u_v[k],\,0,\,100\big),\; v_{\text{cap}}\Big)
$$

**Fallback:** if `tpc_chassis_sensors` goes stale (`> sensor_timeout_sec`)
or `use_closed_loop_speed = false`, the loop drops to open-loop passthrough
(`duty = v_target`) and the PID integrator/derivative state resets so it
doesn't resume wound-up later.

### 2.3 Block diagram

```mermaid
flowchart LR
    vcmd["v_cmd\n(tpc_rover_ctrl_cmd[1])"] --> capclamp["clamp 0–100,\nmin with v_cap"]
    capclamp --> vtarget["v_target"]

    encL["encoder L"] --> delta(("Δ / Δt"))
    encR["encoder R"] --> delta
    delta --> nmeas["ṅ_meas (ticks/s)"]

    nmeas -- "× 100 / ṅ_max" --> vmeas["v_meas (%)"]
    vtarget --> errV(("Σ"))
    vmeas -- "−" --> errV
    errV -- "e_v (%)" --> pidV["PID trim\nkp·e + ki∫e + kd·de/dt\nconditional anti-windup"]

    vtarget -- "feedforward" --> sumFF(("+"))
    pidV -- "trim" --> sumFF
    sumFF --> satV["clamp 0–100"]

    stale{"encoder feed\nstale / disabled?"}
    vtarget -. "open-loop path" .-> stale
    satV -. "closed-loop path" .-> stale
    stale --> capV["min with v_cap"]
    capV --> duty["duty → tpc_chassis_cmd.spd_msg"]
```

### 2.4 Current gains (simulation-derived starting values, not yet field-tuned)

| Param | Value | Role |
|---|---|---|
| `speed_kp` | 0.3 | proportional (dimensionless — % duty per % speed error) |
| `speed_ki` | 0.5 | integral (per second) |
| `speed_kd` | 0.0 | derivative — left off; the ~4 Hz encoder feed is too coarse to differentiate cleanly |
| `speed_integral_limit` | 100.0 | anti-windup clamp (%·s) → `k_i × limit` = ±50 % duty of trim authority |
| `max_ticks_per_sec` | 1000.0 | **placeholder** — needs flat-ground 100%-duty calibration from encoder logs |
| `sensor_timeout_sec` | 1.0 s | stale-feed fallback threshold |

Gains were chosen by simulating the loop at the 4 Hz encoder rate across
no-load / light / moderate / heavy load and a range of drivetrain lags:
every physically reachable setpoint converges to the commanded speed with
<10 % overshoot, and unreachable ones saturate cleanly. They are a safe
starting point, not a substitute for field tuning.

---

## 3. Signal rates & units summary

| Signal | Topic | Rate | Unit |
|---|---|---|---|
| Lane geometry | `tpc_rover_nav_lane` | camera FPS (D6) | px / deg / curvature coeff |
| Steering + speed request | `tpc_rover_ctrl_cmd` | 50 Hz | degrees / 0–100 % duty |
| Wheel encoders | `tpc_chassis_sensors` | ~4 Hz | raw ticks |
| Speed PID internals | `tpc_chassis_speed_debug` | ~4 Hz (paced by encoders) | ticks/s, ticks/s, % |
| Motor command | `tpc_chassis_cmd` | 50 Hz (steering-paced) | direction enum / PWM % |

Steering and speed updates are intentionally decoupled: steering re-reads
`tpc_rover_ctrl_cmd` at full 50 Hz, but the speed correction only refreshes
when a new encoder message arrives (~4 Hz) — the encoder feed is the
bottleneck, not the control loop.

## 4. Known limitations

- `max_ticks_per_sec` is still a placeholder and **must** be calibrated from
  a flat-ground 100 %-duty run before the closed loop is trusted. It defines
  what "100 % speed" means to the loop, so calibrating it from a no-load
  bench run (wheels off the ground, which spin faster than under load) makes
  the whole speed scale no-load-referenced and the loop will push extra duty
  on the ground chasing a target it can't reach. A bench run is still the
  right way to verify encoder wiring, left/right symmetry, duty→speed
  linearity, and the motor deadband — just not to set this constant.
- `speed_kp/ki/kd` are simulation-derived starting values, not field-tuned.
- The speed loop's setpoint/error live entirely in the 0–100 % duty
  abstraction (via `max_ticks_per_sec`), not physical units (m/s) — there is
  no independent ground-truth speed sensor.
- Steering has no feedback from actual wheel angle/yaw — it is an open-loop
  angle command to the servo, closed only at the perception level (lane
  error re-measured each frame).
