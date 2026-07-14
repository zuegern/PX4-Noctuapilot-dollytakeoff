# PX4 Trolley Takeoff

This trolley takeoff mode keeps PX4 as the flight and trajectory controller while the trolley steering servos are controlled in one of two ways:

- Direct Pixhawk PWM, using the existing Landing Gear Wheel output assignment in QGroundControl.
- Raspberry Pi Pico control, where PX4 sends steering commands over the magnetic connector (UART or I2C) and the Pico drives the servos.

In Raspberry Pi Pico mode, the Pico reports link health back to PX4 and slowly returns the wheels to center if the connector disconnects.

## Raspberry Pi Pico Safety Concept

When `TROLLEY_SRV_CTL=1` (serial) or `2` (I2C), PX4 sends centered, inactive heartbeat commands while trolley takeoff is not running and active steering commands during trolley takeoff. The Pico sends status packets back (about 50 Hz on serial; on I2C, PX4 reads one back with every command exchange). PX4 only considers the link healthy when the Pico reports a fresh command and echoes a recent PX4 command sequence, so either broken link direction is detected.

At arming, PX4 reports the link state once: "Trolley Pico link OK" (info) when the link is healthy, or "Trolley Pico link not ready, trolley takeoff would abort" (critical) when it is not. Because a healthy status requires valid checksummed packets of the correct protocol version with fresh sequence echoes, the OK message also confirms that the Pico is powered, wired correctly, and running matching trolley firmware. Arming itself is not blocked; a takeoff attempted with a dead link aborts immediately at takeoff start.

Before release:

- If Pico status packets stop, PX4 treats this as a bad disconnect and aborts trolley takeoff.
- If the Pico reports stale commands, failsafe centering, or an old echoed command sequence, PX4 also treats the link as unhealthy.
- If PX4 disarms or leaves trolley takeoff before release, it sends an abort/center command to the Pico.
- The Pico slowly returns the wheels to center when command packets stop.

At release and climbout:

- PX4 commands neutral steering for `TROLLEY_STR_HOLD`.
- While the link is still connected, the Pico centers the wheels at the normal command slew rate so they are straight before the connector separates.
- If the connector then separates, PX4 treats this as the expected release.
- After separation the Pico keeps slewing the wheels gently toward center at the slow failsafe rate instead of snapping them there.
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

Recommended port: TELEM2 on Pixhawk 6C (JST-GH 6-pin: 1 VCC, 2 TX, 3 RX, 4 CTS, 5 RTS, 6 GND). Leave VCC, CTS, and RTS unconnected.

GPS2 works the same way with `TROLLEY_COM_PORT=4` (JST-GH 6-pin: 1 VCC, 2 TX, 3 RX, 4 SCL, 5 SDA, 6 GND — leave VCC and the I2C pins unconnected, or split the I2C pins off to a sensor; the buses are independent). Keep `GPS_2_CONFIG=0` so no GPS driver owns the port, and note that this spends the port normally used for a second GPS.

Magnetic breakaway umbilical, 5 wires:

```text
pin 1  GND
pin 2  Pixhawk TX -> Pico GP1
pin 3  GND
pin 4  Pico GP0  -> Pixhawk RX
pin 5  GND
```

The protocol only needs TX, RX, and GND; the spare wires are parallel grounds because contact resistance on magnetic pogo pins is the weak point. Put a ~330 ohm resistor in series with each TX line on both sides: with the symmetric pin layout a reversed mating then only swaps the data pins into a harmless, detectable link failure instead of driving two outputs against each other.

### Raspberry Pi Pico I2C Wiring

If no Pixhawk serial port is free, the same packets can run over I2C with `TROLLEY_SRV_CTL=2` and `TROLLEY_LINK_USE_I2C 1` in the Pico sketch. PX4 is the bus master, the Pico an I2C slave at `TROLLEY_I2C_ADDR` (default 0x3B) on `TROLLEY_I2C_BUS`:

```text
Pixhawk SDA  <-> Pico GP0 / I2C0 SDA
Pixhawk SCL  <-> Pico GP1 / I2C0 SCL
Pixhawk GND  <-> Pico GND / trolley battery ground
```

On the Pixhawk 6C the dedicated 4-pin I2C port (1 VCC 5V, 2 SCL, 3 SDA, 4 GND) carries bus 2 — the same bus as the GPS2 connector pins 4/5 — so the default `TROLLEY_I2C_BUS=2` fits both hookups; leave the port's VCC pin unconnected. Exception per Holybro: 6C units with serial numbers up to pattern `...20221100` wire the dedicated port to bus 4 instead. Never put the trolley on bus 4: it carries the internal barometer and magnetometer, and a fault on the breakaway stub could take out both. On such an old unit use the GPS2 connector pins, which are bus 2 on all revisions. To verify, power the connected Pico and run `i2cdetect -b 2` in the MAVLink console: address 0x3b must appear.

Bus 1 is available on the GPS1 connector I2C pins as an alternative.

Umbilical wiring at the 5-pin magnetic pogo connector (as built):

```text
red    not connected
black  not connected
white  I2C2_SCL
green  I2C2_SDA
blue   GND
```

Because the I2C lines are open-drain and no power crosses the connector, a reversed mating cannot damage anything; it just fails the link detectably. The bus pull-ups sit on the Pixhawk side, so after separation the Pico simply stops receiving and failsafe-centers as usual. Optional improvement: the unused red and black wires can be paralleled with the blue ground wire on both sides to reduce contact resistance on the pogo pins, which is their weak point.

I2C-specific cautions:

- The trolley shares the bus with every other sensor on it. A short or heavy noise on the trolley stub disturbs those sensors until the connector separates, so pick the external bus with the fewest flight-critical devices and keep the stub short.
- `TROLLEY_I2C_ADDR` must not collide with any sensor on the same bus; command writes to a foreign device could misconfigure it. The default 0x3B avoids common PX4 sensor addresses.
- PX4 exchanges one command/status pair every 20 ms; the same `TROLLEY_COM_LOSS` timeout and health flags apply as on the serial link. `TROLLEY_COM_PORT` and `TROLLEY_COM_BAUD` are unused in I2C mode.

### Raspberry Pi Pico Pin Map

Physical pin numbers of every Pico pin this project uses. Pin 1 is the corner pin next to the USB connector; numbering runs down the USB-left edge (1-20) and back up the other edge (21-40). On the board, ground pins are the ones with square solder pads; all others are round.

| Function | Pico name | Physical pin |
|---|---|---:|
| Pixhawk link data: I2C mode SDA (green), serial mode UART0 TX | GP0 | 1 |
| Pixhawk link clock: I2C mode SCL (white), serial mode UART0 RX | GP1 | 2 |
| Umbilical ground (blue) | GND | 3 |
| Left steering servo signal | GP14 | 19 |
| Right steering servo signal | GP15 | 20 |
| 3.3 V regulator output - never power servos from this | 3V3(OUT) | 36 |
| Trolley/servo/BEC common ground | GND | 38 |
| 5 V supply input from the trolley BEC | VSYS | 39 |
| USB 5 V (present only while a USB cable is plugged in) | VBUS | 40 |

All eight GND pins (3, 8, 13, 18, 23, 28, 33, 38) are connected together internally, so any of them works for any ground wire; pins 3 and 38 are simply the closest to the link pins and the power input.

Power rules:

- No power crosses the umbilical. The Pico and the servos run entirely from the trolley battery, so the Pico keeps centering the wheels after the connector separates.
- Servos: high-current BEC (two large steering servos stall at 3-6 A each, so plan ~10 A peak) with a 470-1000 uF low-ESR capacitor at the servo rail, 18-20 AWG power wiring.
- Pico: separate small 5 V supply into VSYS (pin 39), so a servo stall brownout cannot reboot it. Do not power servos from the Pico 3V3 pin.
- One common ground point on the trolley: battery, BECs, servos, Pico.

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

The sketch is Arduino C++ and requires the "Raspberry Pi Pico/RP2040" board package by Earle Philhower (it uses `Wire.setSDA`/`Wire.setSCL` and the bundled `Servo` library, which the alternative "Arduino Mbed OS RP2040" package does not provide — the sketch will not compile there). One-time setup in the Arduino IDE:

1. File - Preferences - Additional boards manager URLs, add: `https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json`
2. Tools - Board - Boards Manager, install "Raspberry Pi Pico/RP2040/RP2350" by Earle F. Philhower III.
3. Tools - Board, select "Raspberry Pi Pico".

Flashing: compiling produces a UF2 file, the format the Pico's built-in bootloader accepts. For the first upload (and any time the IDE cannot find the board), hold the BOOTSEL button while plugging in USB; the Pico appears as a USB drive named `RPI-RP2` and the IDE's Upload button copies the UF2 onto it automatically. After the first Arduino upload, later uploads work over USB without the button. A prebuilt image of this sketch in I2C mode with the default (uncalibrated) servo values is at `build/trolley_pico_i2c_default_calibration.uf2` — good enough for bench link tests by dragging it onto the `RPI-RP2` drive, but rebuild after calibrating the servo constants for the real trolley. Verified: the sketch compiles cleanly for this target in both link modes (arduino-pico core 5.6.1, `arduino-cli compile --fqbn rp2040:rp2040:rpipico`).

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

### Steering Linkage Linearization

If the servo drives the wheel through a multi-bar linkage, the wheel angle is a nonlinear function of the servo angle, and PX4's assumption that the normalized command is proportional to the wheel angle (scaled by `TROLLEY_STR_MAX`) no longer holds. The sketch has a `kSteeringToServoMap` table that corrects this: its entries are the servo pulse fractions that produce equally spaced wheel angles from `-TROLLEY_STR_MAX` to `+TROLLEY_STR_MAX`. The identity default keeps the old linear behavior.

To calibrate, put the trolley on stands, command a sweep of servo positions, measure the actual wheel angle at each (a phone inclinometer on the wheel works), then fill the table with the servo fractions at equally spaced wheel angles. After changing the table, verify that commanding half steering physically gives half of `TROLLEY_STR_MAX` at the wheel.

Also set the link type at the top of the sketch: `TROLLEY_LINK_USE_I2C 1` for the I2C link (GP0 SDA, GP1 SCL, slave address `kI2cAddress` matching `TROLLEY_I2C_ADDR`) or `0` for the UART link (standard `Serial1` UART0 pins, GP0 TX and GP1 RX, 115200 baud — PX4 must use the same baud rate).

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
TROLLEY_SRV_CTL = 2        # Raspberry Pi Pico I2C servo controller and link monitoring
```

Direct Pixhawk PWM mode uses the same setup as the original trolley tests: assign both steering servo outputs to Landing Gear Wheel in QGroundControl.

Raspberry Pi Pico link:

```text
TROLLEY_COM_PORT = 1       # Serial mode only: TELEM2 on Pixhawk 6C
TROLLEY_COM_BAUD = 115200  # Serial mode only: must match Pico sketch
TROLLEY_I2C_BUS = 2        # I2C mode only: external bus (6C: 1 = GPS1, 2 = GPS2; never 4)
TROLLEY_I2C_ADDR = 59      # I2C mode only: Pico slave address, must match the sketch (0x3B)
TROLLEY_COM_LOSS = 0.20    # Link-loss timeout in seconds, both link types
```

Takeoff release condition:

```text
TROLLEY_TK_COND = 0        # Airspeed, normal real takeoff
TROLLEY_TK_COND = 1        # Estimated ground speed, test only
TROLLEY_TK_COND = 2        # Time, test only

TROLLEY_RLS_HOLD           # Time the speed condition must stay true before release
```

`TROLLEY_RLS_HOLD` filters gusts and estimation spikes so one sample above the release threshold does not trigger rotation. It does not apply to the test-only time condition.

Steering/path:

```text
TROLLEY_STR_MODE = 0       # Heading hold, runway-like default
TROLLEY_STR_MODE = 1       # Direct nonlinear path tracking using PX4 position estimate
TROLLEY_STR_MODE = 2       # Open-loop steering for no-GPS tests
TROLLEY_STR_MODE = 3       # Path tracking through the existing PX4 wheel PIFF controller

FW_W_EN = 1                # Required for modes 0 and 3, and for every mode with TROLLEY_SRV_CTL=0

TROLLEY_PATH = 0           # Straight
TROLLEY_PATH = 1           # Constant-radius right turn
TROLLEY_PATH = 2           # Constant-radius left turn

TROLLEY_RADIUS             # Reference-point turn radius
TROLLEY_WB                 # Trolley wheelbase
TROLLEY_REF_X              # Forward offset from rear axle to PX4 reference point
TROLLEY_STR_MAX            # Physical wheel angle limit, not servo horn angle
TROLLEY_TRK_GAIN           # Mode 1 heading-error feedback magnitude
TROLLEY_XTK_GAIN           # Modes 1 and 3 cross-track feedback gain [1/m]
TROLLEY_XTK_MAX            # Pre-release cross-track abort limit
TROLLEY_LAT_ACC            # Speed-dependent lateral-acceleration limit; -1 disables
TROLLEY_STR_RATE           # PX4 command slew rate before sending to the Pico
TROLLEY_STR_HOLD           # Minimum neutral-steering time after release

TROLLEY_ALN_THR            # Alignment taxi throttle; 0 disables the alignment phase
TROLLEY_ALN_ERR            # Heading error below which the trolley counts as aligned
TROLLEY_ALN_TO             # Alignment timeout before aborting takeoff
```

### Path Alignment Taxi Phase

For a mission takeoff, the reference path direction is the bearing from the start position to the takeoff waypoint, which the trolley on the ground usually does not match exactly. With `TROLLEY_ALN_THR > 0`, the closed-loop path modes 1 and 3 therefore begin with an alignment taxi phase, following the same low-speed-alignment practice used by automatic-taxi systems for fixed-wing aircraft (Zammit and Zammit-Mangion, 2014, in the references below): PX4 drives the trolley at the configured taxi throttle and steers with full authority until the path heading error stays below `TROLLEY_ALN_ERR` for half a second. It then re-anchors the path at the aligned position, so the alignment arc does not count as cross-track error, and starts the normal throttle ramp. The cross-track abort limit `TROLLEY_XTK_MAX` only applies after alignment; if the trolley cannot align within `TROLLEY_ALN_TO`, takeoff aborts.

In Takeoff mode (not Mission), the reference bearing is the heading at takeoff start, so the alignment phase completes after its half-second check and only adds a short taxi. Heading-hold and open-loop modes skip the phase entirely. The heading estimate used for alignment comes from the EKF (magnetometer, plus GPS course once moving), so check in QGroundControl that the displayed heading matches the real trolley direction before starting — steel, servo wiring, and the trolley battery can distort the magnetometer.

Mode 1 uses path curvature as direct steering feedforward and adds bounded heading/cross-track feedback. Mode 3 instead converts the same path geometry into yaw and yaw-rate references for PX4's existing PIFF wheel controller. `TROLLEY_REF_X` compensates for the PX4 position reference being ahead of the rear axle and must be nonnegative. The Qin-Li sufficient constant-curvature stability check applies only to mode 1. Both closed-loop modes require valid navigation and enforce `TROLLEY_XTK_MAX`, steering feasibility, and the optional lateral-acceleration limit. Leave `TROLLEY_LAT_ACC=-1` until a safe trolley-specific limit has been measured.

Direct-PWM wheel control remains enabled beyond `TROLLEY_STR_HOLD` when necessary for the slew-limited command to finish reaching center.

## Controller Basis And Limits

Mode 1 follows the offset-reference kinematic bicycle controller proposed by Qin and Li. In the local NED frame, path error, curvature, and steering are positive to the right. For wheelbase `L`, forward reference-point offset `d`, path curvature `kappa`, yaw `psi`, and path heading `psi_D`, it uses:

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

For mode 1, PX4 also checks the paper's sufficient constant-curvature negative-feedback condition:

```text
TROLLEY_TRK_GAIN > 0
TROLLEY_XTK_GAIN >
    d * tan(TROLLEY_STR_MAX)^2
    / (L * sqrt(L^2 + d^2 * tan(TROLLEY_STR_MAX)^2))
```

Mode 3 retains the existing PX4 heading/rate PIFF wheel-controller structure and changes only its reference. The path layer supplies:

```text
psi_sp = wrap_pi(psi_D + theta_0 - atan(TROLLEY_XTK_GAIN * e))
r_ff   = V * kappa / sqrt(1 - (d * kappa)^2)
r_sp   = constrain(wrap_pi(psi_sp - psi) / 0.1 + r_ff, +/-FW_W_RMAX)
```

The existing wheel rate controller then uses `FW_WR_P`, `FW_WR_I`, `FW_WR_FF`, and `FW_WR_IMAX` to calculate the normalized wheel command. `FW_W_EN=1` is required for the PIFF-based modes 0 and 3 in every wiring, and additionally for all steering modes with `TROLLEY_SRV_CTL=0`, because the direct wheel command also passes through the attitude controller's `FW_W_EN`-gated wheel-control output. Trolley takeoff aborts before release if it is disabled. On a straight path, `kappa=0`, so mode 3 becomes position-correcting heading control; on a curve, `r_ff` supplies the nominal turning rate while the yaw error corrects disturbances. Before release, PX4 aborts if the nominal `r_ff` exceeds `FW_W_RMAX`; the radius, speed, or rate limit must be changed instead of silently clipping the requested curve. The Qin-Li stability inequality above is not applied to this different PIFF cascade.

This check is not a flight-safety proof. The model assumes forward motion, negligible tire slip, a reference point on the trolley longitudinal centerline, an accurately known wheelbase and steering angle, and steering that follows its command. The Qin and Li paper reports simulation rather than physical-vehicle validation. Rough-ground operation also introduces steering delay, compliance, tire slip, estimator error, and independent-servo mismatch that the kinematic model does not represent. These effects must be measured and validated progressively before flight.

Primary references:

- W. B. Qin and Z. Li, [A Nonlinear Lateral Controller Design for Vehicle Path-following with an Arbitrary Sensor Location](https://arxiv.org/abs/2205.07762), arXiv:2205.07762, 2022. This preprint is the source of the offset-aware feedforward, desired yaw offset, bounded feedback, lateral-acceleration limit, and sufficient gain condition.
- G. M. Hoffmann et al., [Autonomous Automobile Trajectory Tracking for Off-Road Driving: Controller Design, Experimental Validation and Racing](https://ai.stanford.edu/~gabeh/papers/hoffmann_stanley_control07.pdf), American Control Conference, 2007. This paper provides derivation and substantial real-vehicle evidence for nonlinear heading/cross-track steering, and explicitly models tire slip and steering delay.
- R. C. Coulter, [Implementation of the Pure Pursuit Path Tracking Algorithm](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf), CMU-RI-TR-92-01, 1992. This technical report is a standard geometric path-tracking reference and explains rear-axle-centered curvature geometry.
- A. Zammit and D. Zammit-Mangion, [A control technique for automatic taxi in fixed wing aircraft](https://doi.org/10.2514/6.2014-1163), AIAA SciTech Forum (52nd Aerospace Sciences Meeting), 2014. DOI: 10.2514/6.2014-1163. This paper documents low-speed guidance-and-control taxiing for fixed-wing aircraft, the practice the alignment taxi phase follows.
- PX4, [Fixed-Wing Takeoff Mode](https://docs.px4.io/main/en/flight_modes_fw/takeoff). This is the authoritative reference for the runway takeoff phases and climbout behavior reused by the trolley state machine.

For trolley-only push tests without UAV thrust:

```text
TROLLEY_MAX_THR = 0
TROLLEY_TK_COND = 2
TROLLEY_TK_TIME = desired test duration
TROLLEY_STR_MODE = 2
TROLLEY_PATH = 0, 1, or 2
```

## One-Time Checks After Assembly

Do these once when the trolley and UAV first come together, before any powered test:

1. Measure the axle-to-axle wheelbase and set `TROLLEY_WB` (design value 0.68 m).
2. Set `TROLLEY_STR_MAX` to the real maximum wheel angle (10 deg for the current linkage, not the parameter default).
3. Mount the servo horn so that at wheel-neutral the crank is parallel to the wheel rod, with the servo's own mid-travel rotated roughly one half spline tooth toward the +10 deg side: the required crank travel is asymmetric (about -77 deg for one lock and +90 deg for the other), and centering the servo on that range keeps both extremes inside a 180 deg servo. Falling a degree short of one lock is fine; near the dead points that costs only ~0.1 deg of wheel angle.
4. Calibrate the Pico servo pulses: `k...CenterUs` at wheels straight (this absorbs any horn-mounting offset; it will not be 1500), `k...MinUs`/`k...MaxUs` at the pulses where the wheel FIRST reaches the maximum angle on each side. These will be strongly asymmetric around center — that is expected. Then verify the `kSteeringToServoMap` mid-points with a protractor or phone inclinometer (commanded half steering must give half the wheel angle).
5. Tilt test: with the UAV seated, tilt the loaded trolley sideways until the uphill wheels unload and note the angle. Set `TROLLEY_LAT_ACC` to at most half of `9.81 * tan(tilt angle)`.
6. With the UAV seated, check the clearance between the propeller tip circle and the trolley front structure and wheels.
7. Weigh the loaded trolley per axle if possible; the front-axle load determines the steering traction available on soft ground.
8. Measure `TROLLEY_REF_X`: longitudinal distance from the rear axle forward to the flight controller position with the UAV seated.

## Starting A Test

1. Set `TROLLEY_SRV_CTL=0` for direct Pixhawk PWM, `1` for Raspberry Pi Pico serial control, or `2` for Raspberry Pi Pico I2C control.
2. For direct Pixhawk PWM, assign both servo outputs to Landing Gear Wheel in QGroundControl.
3. For Raspberry Pi Pico control, set `TROLLEY_LINK_USE_I2C` in the sketch to match the chosen link, upload it, and wire either Pixhawk UART TX/RX/GND or I2C SDA/SCL/GND to the Pico.
4. Power the trolley servo BEC, and power the Pico if it is used.
5. Connect Pixhawk to QGroundControl.
6. Set the PX4 trolley parameters above.
7. In serial mode, make sure the chosen serial port is not used by MAVLink/GPS/another driver. In I2C mode, make sure no sensor on the chosen bus uses `TROLLEY_I2C_ADDR`.
8. Reboot Pixhawk after changing link-related parameters or the servo-control source.
9. Verify that the wheels center when PX4 is not commanding trolley steering.
10. Verify the steering direction: push the trolley by hand with a closed-loop steering mode active and check that the wheels steer back toward the path. If they steer away, flip the matching `k...Reversed` value in the Pico sketch or reverse the servo output in QGroundControl for direct PWM.
11. Start trolley takeoff mode.
12. In Raspberry Pi Pico mode, watch for a bad-disconnect abort if you unplug the connector before release.
13. Verify that after release/climbout the wheels return to center.

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
| 0 | `state` | Trolley takeoff state: `0` align to path, `1` throttle ramp, `2` clamped to trolley, `3` climbout, `4` flying, `5` aborted |
| 1 | `steering_mode` | `0` heading, `1` direct path tracking, `2` open loop, `3` PX4 PIFF path tracking |
| 2 | `steering_source` | `0` heading controller, `1` direct path tracking, `2` open loop, `3` invalid navigation, `4` PX4 PIFF path tracking |
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
| 54-55 | PIFF path reference | Desired wheel yaw and yaw-rate feedforward |

Positive `path_error_m` always means that the PX4 reference point is to the right of the path direction. This convention stays consistent for straight and curved paths.

`final_wheel_command` is populated when `fw_mode_manager` drives the wheels directly or forwards the heading-controller output to the Pico. In heading mode with direct Pixhawk PWM, the fixed-wing attitude controller generates the wheel command later in the pipeline; use the exported `actual_wheel_output` column for that value.

Convert a downloaded `.ulg` file into CSV files and ready-made graphs with one command:

```sh
python3 -m pip install pyulog matplotlib
python3 src/modules/fw_mode_manager/trolley_takeoff/tools/export_trolley_debug.py flight.ulg
```

Everything is written next to the log file:

- `flight_trolley.csv`: named columns for every debug field, elapsed test time, angles in both radians and degrees, and the nearest logged `landing_gear_wheel` output (`actual_wheel_output`). Opens directly in Python/pandas, MATLAB, Excel, or LibreOffice Calc.
- `flight_vibration.csv`: body-frame accelerations and angular rates at the full IMU log rate, on the same time base as the trolley CSV.
- `flight_plots/*.png`: graphs with the background shaded by takeoff state:
  - `steering.png`: state/mode/source and the wheel command through the whole pipeline: raw, slew-limited, final mode-manager command, and the actual `landing_gear_wheel` output. Covers all four steering modes.
  - `path_tracking.png`: cross-track errors, every heading signal (yaw, ground course, desired course, path heading, reference bearing), and the angular errors the controller acted on.
  - `controller.png`: mode-1 feedforward/feedback split against the steering limit, the PIFF wheel-yaw reference, and the stability/feasibility checks.
  - `speeds.png`: ground speed, body-longitudinal speed, and guidance lateral acceleration.
  - `health.png`: estimator validity flags, Pico link state, estimator resets, eph/evh, and position age.
  - `trajectory.png`: map view of the ground track with start point and reference bearing.
  - `vibration_accel.png`, `vibration_gyro.png`, `vibration_spectrum.png`: see the next section.

Options: `--no-plots` writes only CSV, `--no-csv` writes only graphs, `--tmin`/`--tmax` crop the vibration data, and `--rms-window` sets the vibration smoothing window in seconds.

An open-loop straight path intentionally produces `raw_wheel_setpoint = 0`; it cannot correct a position or heading error. Both closed-loop modes abort before release when navigation is invalid, the configured cross-track limit is exceeded, or the requested curve violates the configured steering/lateral-acceleration limit. Mode 1 additionally requires its sufficient constant-curvature stability condition. A `steering_source` value of `3` identifies invalid navigation in diagnostics.

## Vibration And Sliding Analysis

The same export command also extracts mechanical behavior from the logged IMU data (`sensor_combined`, body FRD frame, about 200 Hz with the default logging profile). This works on any log, even when trolley takeoff was never started, so hand-push and towing tests are enough; without trolley debug data the tool simply writes the vibration outputs alone.

- `vibration_accel.png`: one panel per axis. The blue rolling mean is the sustained low-frequency acceleration, the red band is the vibration RMS around it, and the trolley-active time span is shaded for comparison between runs.
  - X forward: a sustained offset that does not match thrust or braking while clamped means the UAV slides front-to-back against the L-stop.
  - Y right: a sustained offset on a straight run means the UAV slides left-right on the cradle; a growing oscillation is wheel shimmy.
  - Z down: reads about -9.8 m/s^2 at rest; the width of the vibration band is the ground-roughness metric. Compare the Z band between runs to judge a suspension change.
- `vibration_gyro.png`: roll = rocking on the cradle, pitch = pitching over bumps, yaw = steering response and shimmy, in deg/s.
- `vibration_spectrum.png`: one frequency-spectrum panel per axis over the trolley-active window, so low- and high-frequency content can be judged separately. A raw discrete Fourier transform of the whole run would be too noisy and its amplitude would depend on run length; the tool therefore plots a Welch power spectral density (Welch, 1967: many short Hann-windowed DFTs, averaged, normalized per Hz), which is the comparable version of the same idea. A suspension change should move energy out of the structural resonance peaks; shimmy appears as a sharp lateral/yaw peak, typically at 10-30 Hz (Somieski, 1997).

To compare trolley or suspension designs directly, pass several logs at once:

```sh
python3 src/modules/fw_mode_manager/trolley_takeoff/tools/export_trolley_debug.py old_design.ulg new_design.ulg
```

This writes `vibration_compare.png` with the per-axis spectra of all runs overlaid, and prints a per-axis vibration RMS table split into 0.5-10, 10-30, and 30+ Hz bands - single numbers for "did the suspension get better". Compare runs recorded with the same test procedure: excitation grows with speed and depends on the surface, so a faster or rougher run always looks worse regardless of the design.

The IMU measures at the Pixhawk mounting location, so every reading includes structure flex between the wheels and that point; compare runs against each other rather than against absolute limits. The default log rate resolves vibration up to roughly 100 Hz; if finer detail is needed, also set the high-rate logging bit (`SDLOG_PROFILE = 37`).

Signal-processing references:

- P. D. Welch, [The use of fast Fourier transform for the estimation of power spectra: A method based on time averaging over short, modified periodograms](https://doi.org/10.1109/TAU.1967.1161901), IEEE Transactions on Audio and Electroacoustics, vol. 15, no. 2, pp. 70-73, 1967. DOI: 10.1109/TAU.1967.1161901. This paper defines the averaged-periodogram spectral estimate the export tool computes (512-sample segments, 50 % overlap in this implementation).
- F. J. Harris, [On the use of windows for harmonic analysis with the discrete Fourier transform](https://doi.org/10.1109/PROC.1978.10837), Proceedings of the IEEE, vol. 66, no. 1, pp. 51-83, 1978. DOI: 10.1109/PROC.1978.10837. This paper is the standard reference for the Hann window and for why the segments are windowed against spectral leakage.
- G. Somieski, [Shimmy analysis of a simple aircraft nose landing gear model using different mathematical methods](https://doi.org/10.1016/S1270-9638(97)90003-1), Aerospace Science and Technology, vol. 1, no. 8, pp. 545-555, 1997. DOI: 10.1016/S1270-9638(97)90003-1. This paper analyzes landing-gear wheel shimmy, the self-excited steering oscillation the yaw-rate and spectrum graphs are checked for.

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
