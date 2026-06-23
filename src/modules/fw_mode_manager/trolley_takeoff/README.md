# PX4 Trolley Takeoff

This trolley takeoff mode keeps PX4 as the flight and trajectory controller while the trolley steering servos are controlled in one of two ways:

- Direct Pixhawk PWM, using the existing Landing Gear Wheel output assignment in QGroundControl.
- Raspberry Pi Pico serial control, where PX4 sends steering commands over the magnetic connector and the Pico drives the servos.

In Raspberry Pi Pico mode, the Pico reports link health back to PX4 and slowly returns the wheels to center if the connector disconnects.

## Raspberry Pi Pico Safety Concept

When `TROLLEY_SRV_CTL=1`, PX4 sends a command packet to the Raspberry Pi Pico every control cycle. The Pico sends status packets back at about 50 Hz.

Before release:

- If Pico status packets stop, PX4 treats this as a bad disconnect and aborts trolley takeoff.
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
static constexpr int kPixhawkTxPin = 0; // Pico GP0 / UART0 TX -> Pixhawk RX
static constexpr int kPixhawkRxPin = 1; // Pico GP1 / UART0 RX <- Pixhawk TX

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

The Pico sketch uses `Serial1.begin(115200)` for the Pixhawk link, so PX4 must use the same baud rate.

## PX4/QGroundControl Parameters

Basic trolley selection:

```text
FW_TKOFF_METHOD = 3        # Trolley
```

Attached-trolley pitch:

```text
TROLLEY_PSP                 # Pitch while attached to trolley only
```

`TROLLEY_PSP` can be slightly negative for a gentle nose-down/downforce bias during ground roll. It is not used after release; climbout immediately uses the normal fixed-wing takeoff pitch limits.

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
TROLLEY_STR_RATE           # PX4 command slew rate before sending to the Pico
TROLLEY_STR_HOLD           # Time PX4 commands neutral steering after release
```

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
