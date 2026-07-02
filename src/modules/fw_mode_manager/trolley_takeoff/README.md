# PX4 Trolley Takeoff

This trolley takeoff mode keeps PX4 as the flight and trajectory controller while the trolley steering servos are controlled in one of two ways:

- Direct Pixhawk PWM, using the existing Landing Gear Wheel output assignment in QGroundControl.
- Raspberry Pi Pico serial control, where PX4 sends steering commands over the magnetic connector and the Pico drives the servos.

In Raspberry Pi Pico mode, the Pico reports link health back to PX4 and slowly returns the wheels to center if the connector disconnects.

## Raspberry Pi Pico Safety Concept

When `TROLLEY_SRV_CTL=1`, PX4 sends centered, inactive heartbeat commands while trolley takeoff is not running and active steering commands during trolley takeoff. The Pico sends status packets back at about 50 Hz. PX4 only considers the link healthy when the Pico reports a fresh command and echoes a recent PX4 command sequence, so either broken UART direction is detected.

Before release:

- If Pico status packets stop, PX4 treats this as a bad disconnect and aborts trolley takeoff.
- If the Pico reports stale commands, failsafe centering, or an old echoed command sequence, PX4 also treats the link as unhealthy.
- If PX4 disarms or leaves trolley takeoff before release, it sends an abort/center command to the Pico.
- The Pico slowly returns the wheels to center when command packets stop.

At release and climbout:

- PX4 commands neutral steering for `TROLLEY_STR_HOLD`.
- If the connector then separates, PX4 treats this as the expected release.
- The Pico continues slewing the wheels toward center instead of snapping them there.
- If PX4 switches to the next mission item or another mode after release, it sends one final center command before resetting the trolley state.

The Pico does not independently follow a GPS path after disconnect. That is intentional: without PX4 position/attitude estimates, the safest fallback is controlled centering.

## Wiring

For direct Pixhawk PWM mode, connect the steering servo signal wires to Pixhawk PWM outputs and assign both outputs to Landing Gear Wheel in QGroundControl. Power the servos from the trolley battery/BEC and keep the Pixhawk, servo, and BEC grounds common.

For Raspberry Pi Pico serial mode, connect the servos to the Pico and use the two reserved pogo pins for UART. The Pixhawk UART and Raspberry Pi Pico UART are both 3.3 V logic, so no logic level shifter is needed between them.

### Raspberry Pi Pico Serial Wiring

```text
Pixhawk TELEM2 TX  -> Pico GP1 / UART0 RX
Pixhawk TELEM2 RX  <- Pico GP0 / UART0 TX
Pixhawk GND        -> Pico GND / trolley battery ground
```

Recommended port: TELEM2 on Pixhawk 6C.

Important:

- Pixhawk UART logic is 3.3 V.
- Raspberry Pi Pico GPIO/UART logic is 3.3 V, matching the Pixhawk UART.
- The sketch uses `Serial1` for the Pixhawk link and sets `Serial1` to Pico GP0/GP1.
- Keep `Serial`/USB free for programming and optional debugging.
- Power the servos from the trolley battery/BEC, not from the Pixhawk.
- Power the Pico from USB for bench tests, or from a regulated 5 V source into Pico VSYS/GND for standalone trolley use.
- Connect all grounds together: Pixhawk signal ground, Pico ground, servo ground, and trolley battery/BEC ground.
- Do not connect the Pixhawk PWM servo outputs to the trolley steering servos when `TROLLEY_SRV_CTL=1`. The Pico is the servo controller.

## Raspberry Pi Pico Upload

Open this Pico sketch:

```text
src/modules/fw_mode_manager/trolley_takeoff/raspberry_pico/TrolleyTakeoffRaspberryPicoController/TrolleyTakeoffRaspberryPicoController.ino
```

Before uploading, edit the calibration constants near the top:

```cpp
static constexpr int kLeftServoPin = 14;
static constexpr int kRightServoPin = 15;

static constexpr int kLeftMinUs = 1100;
static constexpr int kLeftCenterUs = 1500;
static constexpr int kLeftMaxUs = 1900;
static constexpr bool kLeftReversed = false;

static constexpr int kRightMinUs = 1100;
static constexpr int kRightCenterUs = 1500;
static constexpr int kRightMaxUs = 1900;
static constexpr bool kRightReversed = false;
```

Set each servo center so the wheels are straight. Then set min/max so the mechanical steering stops are not overdriven. If one wheel moves the wrong way, change the matching `k...Reversed` value.

The Pico sketch uses the standard `Serial1` UART0 pins, GP0 TX and GP1 RX, at 115200 baud. PX4 must use the same baud rate.

## PX4/QGroundControl Parameters

Basic trolley selection:

```text
FW_TKOFF_METHOD = 3        # Trolley
```

Attached-trolley pitch:

```text
TROLLEY_PSP                 # Pitch while attached to trolley
TROLLEY_ROT_TIME            # Transition to normal climbout pitch limits
```

`TROLLEY_PSP` can be slightly negative for a gentle nose-down/downforce bias during ground roll. After release, `TROLLEY_ROT_TIME` smoothly transitions from that constraint to the normal fixed-wing climbout pitch limits.

Servo control source:

```text
TROLLEY_SRV_CTL = 0        # Direct Pixhawk PWM / Landing Gear Wheel output
TROLLEY_SRV_CTL = 1        # Raspberry Pi Pico serial servo controller and link monitoring
```

Direct Pixhawk PWM mode uses the same setup as the original trolley tests: assign both steering servo outputs to Landing Gear Wheel in QGroundControl.

Raspberry Pi Pico link, only used when `TROLLEY_SRV_CTL=1`:

```text
TROLLEY_COM_PORT = 1       # TELEM2 on Pixhawk 6C
TROLLEY_COM_BAUD = 115200  # Must match Pico sketch
TROLLEY_COM_LOSS = 0.20    # Link-loss timeout in seconds
```

Takeoff release condition:

```text
TROLLEY_TK_COND = 0        # Airspeed, normal real takeoff
TROLLEY_TK_COND = 1        # Estimated ground speed, test only
TROLLEY_TK_COND = 2        # Time, test only
```

Steering/path:

```text
TROLLEY_STR_MODE = 0       # Heading hold, runway-like default
TROLLEY_STR_MODE = 1       # Closed-loop path tracking using PX4 position estimate
TROLLEY_STR_MODE = 2       # Open-loop steering for no-GPS tests

TROLLEY_PATH = 0           # Straight
TROLLEY_PATH = 1           # Constant-radius right turn
TROLLEY_PATH = 2           # Constant-radius left turn

TROLLEY_RADIUS             # Reference-point turn radius
TROLLEY_WB                 # Trolley wheelbase
TROLLEY_REF_X              # Forward offset from rear axle to PX4 reference point
TROLLEY_STR_MAX            # Physical wheel angle limit, not servo horn angle
TROLLEY_TRK_GAIN           # Closed-loop heading-error feedback magnitude
TROLLEY_XTK_GAIN           # Closed-loop cross-track feedback gain [1/m]
TROLLEY_XTK_MAX            # Pre-release cross-track abort limit
TROLLEY_LAT_ACC            # Speed-dependent lateral-acceleration limit; -1 disables
TROLLEY_STR_RATE           # PX4 command slew rate before sending to the Pico
TROLLEY_STR_HOLD           # Minimum neutral-steering time after release
```

Closed-loop mode uses path curvature as steering feedforward and adds bounded heading/cross-track feedback. `TROLLEY_REF_X` compensates for the PX4 position reference being ahead of the rear axle and must be nonnegative. Before release, PX4 verifies the controller's sufficient constant-curvature stability condition using `TROLLEY_TRK_GAIN`, `TROLLEY_XTK_GAIN`, `TROLLEY_WB`, `TROLLEY_REF_X`, and `TROLLEY_STR_MAX`. Start with low positive gains and a generous `TROLLEY_XTK_MAX` during low-speed tests. Leave `TROLLEY_LAT_ACC=-1` until a safe trolley-specific limit has been measured; a positive value then uses body-longitudinal speed to limit rear-axle lateral acceleration and rejects an infeasible curve before release.

Direct-PWM wheel control remains enabled beyond `TROLLEY_STR_HOLD` when necessary for the slew-limited command to finish reaching center.

## Controller Basis And Limits

The closed-loop implementation follows the offset-reference kinematic bicycle controller proposed by Qin and Li. In the local NED frame, path error, curvature, and steering are positive to the right. For wheelbase `L`, forward reference-point offset `d`, path curvature `kappa`, yaw `psi`, and path heading `psi_D`, the implementation uses:

```text
delta_ff = atan(L * kappa / sqrt(1 - (d * kappa)^2))
theta_0  = -asin(d * kappa)
theta_e  = wrap_pi(psi - psi_D - theta_0)
delta_fb = g(-K_heading * (theta_e + atan(K_cross * e)))
delta    = constrain(delta_ff + delta_fb, -delta_limit, delta_limit)
```

Here, `e` is the signed cross-track error and `g` is the smooth bounded arctangent wrapper from the paper. The selected path is either a straight line (`kappa=0`) or a constant-radius circle (`kappa=+/-1/R`). Open-loop mode uses only `delta_ff`; it cannot correct disturbances or initial path error.

When `TROLLEY_LAT_ACC` is enabled, `delta_limit` is reduced using the estimated body-longitudinal speed `V`:

```text
delta_limit = min(TROLLEY_STR_MAX, atan(TROLLEY_LAT_ACC * L / V^2))
```

For closed-loop operation, PX4 also checks the paper's sufficient constant-curvature negative-feedback condition:

```text
TROLLEY_TRK_GAIN > 0
TROLLEY_XTK_GAIN >
    d * tan(TROLLEY_STR_MAX)^2
    / (L * sqrt(L^2 + d^2 * tan(TROLLEY_STR_MAX)^2))
```

This check is not a flight-safety proof. The model assumes forward motion, negligible tire slip, a reference point on the trolley longitudinal centerline, an accurately known wheelbase and steering angle, and steering that follows its command. The Qin and Li paper reports simulation rather than physical-vehicle validation. Rough-ground operation also introduces steering delay, compliance, tire slip, estimator error, and independent-servo mismatch that the kinematic model does not represent. These effects must be measured and validated progressively before flight.

Primary references:

- W. B. Qin and Z. Li, [A Nonlinear Lateral Controller Design for Vehicle Path-following with an Arbitrary Sensor Location](https://arxiv.org/abs/2205.07762), arXiv:2205.07762, 2022. This preprint is the source of the offset-aware feedforward, desired yaw offset, bounded feedback, lateral-acceleration limit, and sufficient gain condition.
- G. M. Hoffmann et al., [Autonomous Automobile Trajectory Tracking for Off-Road Driving: Controller Design, Experimental Validation and Racing](https://ai.stanford.edu/~gabeh/papers/hoffmann_stanley_control07.pdf), American Control Conference, 2007. This paper provides derivation and substantial real-vehicle evidence for nonlinear heading/cross-track steering, and explicitly models tire slip and steering delay.
- R. C. Coulter, [Implementation of the Pure Pursuit Path Tracking Algorithm](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf), CMU-RI-TR-92-01, 1992. This technical report is a standard geometric path-tracking reference and explains rear-axle-centered curvature geometry.
- PX4, [Fixed-Wing Takeoff Mode](https://docs.px4.io/main/en/flight_modes_fw/takeoff). This is the authoritative reference for the runway takeoff phases and climbout behavior reused by the trolley state machine.

For trolley-only push tests without UAV thrust:

```text
TROLLEY_MAX_THR = 0
TROLLEY_TK_COND = 2
TROLLEY_TK_TIME = desired test duration
TROLLEY_STR_MODE = 2
TROLLEY_PATH = 0, 1, or 2
```

## Starting A Test

1. Set `TROLLEY_SRV_CTL=0` for direct Pixhawk PWM or `TROLLEY_SRV_CTL=1` for Raspberry Pi Pico serial control.
2. For direct Pixhawk PWM, assign both servo outputs to Landing Gear Wheel in QGroundControl.
3. For Raspberry Pi Pico serial control, upload the Pico sketch and wire Pixhawk UART TX/RX/GND to the Pico.
4. Power the trolley servo BEC, and power the Pico if it is used.
5. Connect Pixhawk to QGroundControl.
6. Set the PX4 trolley parameters above.
7. If using Raspberry Pi Pico mode, make sure the chosen serial port is not used by MAVLink/GPS/another driver.
8. Reboot Pixhawk after changing serial-related parameters or the servo-control source.
9. Verify that the wheels center when PX4 is not commanding trolley steering.
10. Start trolley takeoff mode.
11. In Raspberry Pi Pico mode, watch for a bad-disconnect abort if you unplug the connector before release.
12. Verify that after release/climbout the wheels return to center slowly.

## Trolley Diagnostics And Graphs

While trolley takeoff control is active, PX4 publishes a structured `debug_array` named `trolley` with ID `4242` at 10 Hz. Before recording a test, enable the default and debug logging topic sets:

```text
SDLOG_PROFILE = 33         # Bit 0 Default + bit 5 Debug
```

Reboot PX4 after changing `SDLOG_PROFILE`. The logger will then record both `debug_array` and `landing_gear_wheel` in the `.ulg` file.

For a short live check in the QGroundControl MAVLink Console:

```text
listener debug_array 10
listener landing_gear_wheel 10
listener vehicle_local_position 5
```

The `debug_array` values use a fixed index order. Important indexes are:

| Index | CSV name | Meaning |
|---:|---|---|
| 0 | `state` | Trolley takeoff state |
| 1 | `steering_mode` | `0` heading, `1` closed-loop path tracking, `2` open loop |
| 2 | `steering_source` | `0` heading controller, `1` path tracking, `2` open loop, `3` invalid navigation |
| 3 | `path_type` | `0` straight, `1` right curve, `2` left curve |
| 4 | `navigation_available` | `1` in normal navigation takeoff, `0` in no-navigation takeoff |
| 5-8 | validity flags | Position, velocity, heading and dead-reckoning status |
| 9-17 | position/velocity | Start, current and relative local N/E position plus velocity and speed |
| 18-24 | path tracking | Yaw, ground course, desired course, angular errors and path error |
| 25-28 | steering | Raw, slew-limited, yaw-nudge and final mode-manager wheel command |
| 29-39 | geometry/health | Radius, estimator accuracy, data age, link state and estimator reset counters |
| 40-42 | guidance | Lateral acceleration and NPFG track error values |
| 43-50 | trolley controller | Path heading/curvature, heading error, reference offset, feedforward, feedback, steering limit and feasibility |
| 51-53 | model checks | Body-longitudinal speed, minimum cross-track gain and stability-condition result |

Positive `path_error_m` always means that the PX4 reference point is to the right of the path direction. This convention stays consistent for straight and curved paths.

`final_wheel_command` is populated when `fw_mode_manager` drives the wheels directly or forwards the heading-controller output to the Pico. In heading mode with direct Pixhawk PWM, the fixed-wing attitude controller generates the wheel command later in the pipeline; use the exported `actual_wheel_output` column for that value.

Convert a downloaded `.ulg` file into a graph-ready CSV:

```sh
python3 -m pip install pyulog
python3 src/modules/fw_mode_manager/trolley_takeoff/tools/export_trolley_debug.py flight.ulg
```

The default output is `flight_trolley.csv`. It contains named columns, elapsed test time, angles in both radians and degrees, and the nearest logged `landing_gear_wheel` output. It can be opened directly in Python/pandas, MATLAB, Excel, LibreOffice Calc, or another plotting tool.

For a first steering graph, plot these against `time_s`:

```text
path_error_m
controller_heading_error_deg
steering_feedforward_deg
steering_feedback_deg
longitudinal_speed_m_s
minimum_cross_track_gain_1_m
stability_condition_met
raw_wheel_setpoint
slewed_wheel_setpoint
final_wheel_command
actual_wheel_output
xy_valid
heading_good
```

An open-loop straight path intentionally produces `raw_wheel_setpoint = 0`; it cannot correct a position or heading error. Closed-loop path tracking aborts before release when navigation is invalid, its sufficient constant-curvature stability condition is not met, the configured cross-track limit is exceeded, or the requested curve violates the configured steering/lateral-acceleration limit. A `steering_source` value of `3` identifies invalid navigation in diagnostics after release.

## Serial Protocol

PX4 command packet, 13 bytes:

```text
0  'T'
1  'C'
2  protocol version
3  sequence
4  trolley takeoff state
5  flags
6  steering low byte      int16, normalized steering * 10000
7  steering high byte
8  path type
9  radius cm low byte
10 radius cm high byte
11 reserved
12 XOR checksum over bytes 0..11
```

Raspberry Pi Pico status packet, 8 bytes:

```text
0 'T'
1 'S'
2 protocol version
3 status flags
4 status sequence
5 last received command sequence
6 reserved
7 XOR checksum over bytes 0..6
```

Status flag bits are `0=controller OK`, `1=PX4 command fresh`, and `2=failsafe centering`. PX4 requires bits 0 and 1, rejects bit 2, and verifies byte 5 against its recently transmitted sequence.
