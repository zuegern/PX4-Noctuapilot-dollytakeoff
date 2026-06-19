# PX4 Trolley Takeoff With Arduino Servo Controller

This trolley takeoff mode keeps PX4 as the flight and trajectory controller, but moves the two trolley steering servos to an Arduino on the trolley. PX4 sends steering commands to the Arduino over the magnetic connector. The Arduino drives the servos from the start, reports link health back to PX4, and slowly returns the wheels to center if the connector disconnects.

## Safety Concept

During trolley takeoff, PX4 sends a command packet to the Arduino every control cycle. The Arduino sends status packets back at about 50 Hz.

Before release:

- If Arduino status packets stop, PX4 treats this as a bad disconnect and aborts trolley takeoff.
- The Arduino slowly returns the wheels to center when command packets stop.

At release and climbout:

- PX4 commands neutral steering for `TROLLEY_STR_HOLD`.
- If the connector then separates, PX4 treats this as the expected release.
- The Arduino continues slewing the wheels toward center instead of snapping them there.

The Arduino does not independently follow a GPS path after disconnect. That is intentional: without PX4 position/attitude estimates, the safest fallback is controlled centering.

## Wiring

Use the two reserved pogo pins for UART. With an Arduino UNO R4 WiFi, put a bidirectional logic level shifter between the Pixhawk and Arduino because the Pixhawk UART is 3.3 V logic and the UNO R4 WiFi GPIO/UART pins are 5 V logic.

```text
Pixhawk TELEM2 TX  -> level shifter 3.3V side -> level shifter 5V side -> Arduino RX1
Pixhawk TELEM2 RX  <- level shifter 3.3V side <- level shifter 5V side <- Arduino TX1
Pixhawk GND        -> level shifter GND -> Arduino GND / trolley battery ground
```

Recommended port: TELEM2 on Pixhawk 6C.

Important:

- Pixhawk UART logic is 3.3 V.
- Arduino UNO R4 WiFi GPIO/UART logic is 5 V, so use a logic level shifter.
- The sketch uses `Serial1` for the Pixhawk link. On the UNO R4 WiFi, connect Pixhawk through the level shifter to the board's `RX1` and `TX1` hardware UART pins.
- Keep `Serial`/USB free for programming and optional debugging.
- Power the servos from the trolley battery/BEC, not from the Pixhawk.
- Connect all grounds together: Pixhawk signal ground, Arduino ground, servo ground, and trolley battery/BEC ground.
- Do not connect the Pixhawk PWM servo outputs to the trolley steering servos when `TROLLEY_COM_EN=1`. The Arduino is the servo controller.

## Arduino Upload

Open this sketch in the Arduino IDE:

```text
src/modules/fw_mode_manager/trolley_takeoff/arduino/TrolleyTakeoffServoController/TrolleyTakeoffServoController.ino
```

Before uploading, edit the calibration constants near the top:

```cpp
static constexpr int kLeftServoPin = 9;
static constexpr int kRightServoPin = 10;

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

The Arduino sketch uses `Serial1.begin(115200)` for the Pixhawk link, so PX4 must use the same baud rate.

## PX4/QGroundControl Parameters

Basic trolley selection:

```text
FW_TKOFF_METHOD = 3        # Trolley
```

Arduino link:

```text
TROLLEY_COM_EN = 1         # Use Arduino servo controller and link monitoring
TROLLEY_COM_PORT = 1       # TELEM2 on Pixhawk 6C
TROLLEY_COM_BAUD = 115200  # Must match Arduino sketch
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
TROLLEY_STR_RATE           # PX4 command slew rate before sending to Arduino
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

1. Upload the Arduino sketch.
2. Wire Pixhawk UART TX/RX/GND to the Arduino.
3. Power the trolley servo BEC and Arduino.
4. Connect Pixhawk to QGroundControl.
5. Set the PX4 trolley parameters above.
6. Make sure the chosen serial port is not used by MAVLink/GPS/another driver.
7. Reboot Pixhawk after changing serial-related parameters.
8. Verify the Arduino centers both wheels when PX4 is not sending commands.
9. Start trolley takeoff mode.
10. Watch for a bad-disconnect abort if you unplug the connector before release.
11. Verify that after release/climbout the wheels return to center slowly.

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

Arduino status packet, 8 bytes:

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
