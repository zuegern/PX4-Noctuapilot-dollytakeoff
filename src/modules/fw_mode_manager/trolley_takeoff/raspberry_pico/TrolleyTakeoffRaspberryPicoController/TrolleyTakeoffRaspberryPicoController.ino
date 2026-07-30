/*
 * PX4 trolley takeoff servo controller for Raspberry Pi Pico H.
 *
 * The Raspberry Pi Pico drives the trolley steering servos. PX4 sends
 * steering commands while the aircraft is attached to the trolley.
 * If the link is lost, the Pico slowly returns both wheels to center.
 *
 * Link selection: set TROLLEY_LINK_USE_I2C to 1 for the I2C link (PX4 is the
 * bus master, the Pico a slave on kI2cAddress; TROLLEY_SRV_CTL=2) or to 0 for
 * the UART link on Serial1 (TROLLEY_SRV_CTL=1). Both use the same packets.
 */

#define TROLLEY_LINK_USE_I2C 0

#include <Servo.h>

#if TROLLEY_LINK_USE_I2C
#include <Wire.h>

// Must match TROLLEY_I2C_ADDR (default 59 = 0x3B).
static constexpr uint8_t kI2cAddress = 0x3B;
// Same umbilical wires as the UART link: GP0 = I2C0 SDA, GP1 = I2C0 SCL.
static constexpr int kI2cSdaPin = 0;
static constexpr int kI2cSclPin = 1;
#else
#define PixhawkSerial Serial1
#endif

static constexpr uint8_t kProtocolVersion = 1;
static constexpr uint8_t kCommandPacketLength = 13;
static constexpr uint8_t kStatusPacketLength = 8;

static constexpr uint8_t kFlagActive = 1 << 0;
static constexpr uint8_t kFlagReleased = 1 << 1;
static constexpr uint8_t kFlagAbort = 1 << 2;
static constexpr uint8_t kFlagCenterWheels = 1 << 3;

static constexpr uint8_t kStatusOk = 1 << 0;
static constexpr uint8_t kStatusCommandFresh = 1 << 1;
static constexpr uint8_t kStatusFailsafeCentering = 1 << 2;

static constexpr uint32_t kCommandTimeoutMs = 120;
static constexpr uint32_t kStatusPeriodMs = 20;

// Command tracking (including PX4-commanded centering before the connector separates) may be
// faster; only the autonomous link-loss failsafe centering should be gentle.
static constexpr float kCommandSlewPerSecond = 3.0f;
static constexpr float kFailsafeCenterSlewPerSecond = 0.8f;

// Servo signal pins on the Raspberry Pi Pico H.
static constexpr int kLeftServoPin = 14;  // Pico GP14
static constexpr int kRightServoPin = 15; // Pico GP15

// The Pico Servo library defaults attach(pin) to a conservative 1000-2000 us range and silently
// clamps writeMicroseconds() to it, so pulses beyond 2000 do nothing. Attach with an explicit wide
// range so the servo's full travel is reachable; the calibration clamp and the k...Us limits below
// still bound what is actually commanded.
static constexpr int kServoAttachMinUs = 800;
static constexpr int kServoAttachMaxUs = 2200;

// Calibrate these for your trolley. Values are servo PWM microseconds.     <----- !!!
static constexpr int kLeftMinUs = 900;
static constexpr int kLeftCenterUs = 1450;
static constexpr int kLeftMaxUs = 1980;
static constexpr bool kLeftReversed = true;

static constexpr int kRightMinUs = 1070;
static constexpr int kRightCenterUs = 1610;
static constexpr int kRightMaxUs = 2100;
static constexpr bool kRightReversed = true;

static_assert(kLeftMinUs < kLeftCenterUs && kLeftCenterUs < kLeftMaxUs,
	      "Left servo calibration must satisfy min < center < max");
static_assert(kRightMinUs < kRightCenterUs && kRightCenterUs < kRightMaxUs,
	      "Right servo calibration must satisfy min < center < max");

// Steering-linkage linearization.
//
// The four-bar linkage between servo horn and wheel makes the wheel angle a nonlinear function of
// the servo angle, so equal pulse steps do not give equal wheel-angle steps. This table maps the
// commanded wheel-angle fraction to the servo pulse fraction between center and min/max. Entries
// correspond to equally spaced wheel fractions from -1 to +1.
//
// Default: computed for the trolley linkage (crank 20 mm at O2(-100,0), coupler 60 mm, rocker
// 115.2 mm at O4(-204.3,-59.3), neutral pointing +x), validated against the mechanism simulation.
// This table is computed for a +/-9 deg wheel range: the servos are calibrated to stop at +/-9 deg,
// just short of the +/-10 deg linkage dead points where the crank-to-wheel ratio diverges and the
// servo stalls. Wheel -9 deg is reached at crank -60 deg and +9 deg at crank +69 deg, so calibrate
// kMin/Center/MaxUs to the pulses where the wheel reaches -9 / 0 / +9 degrees, and set PX4
// TROLLEY_STR_MAX = 9. Verify on the trolley (tire flex and linkage slop are not in this model).
// For the full +/-10 deg range the table is
// {-1.0f, -0.542f, -0.341f, -0.167f, 0.0f, 0.165f, 0.347f, 0.574f, 1.0f}.
// For a plain linear servo-to-wheel setup use the identity table:
// {-1.0f, -0.75f, -0.5f, -0.25f, 0.0f, 0.25f, 0.5f, 0.75f, 1.0f}
static constexpr float kSteeringToServoMap[] = {
	-1.0f, -0.679f, -0.435f, -0.215f, 0.0f, 0.192f, 0.402f, 0.649f, 1.0f
};

static constexpr int kSteeringMapPoints = sizeof(kSteeringToServoMap) / sizeof(kSteeringToServoMap[0]);
static_assert(kSteeringMapPoints >= 2, "Steering map needs at least two points");
static_assert(kSteeringMapPoints % 2 == 1, "Steering map needs a center point (odd point count)");

// Servo calibration over the USB serial monitor (bench use only, state clears on power cycle).
// Commands: "l 1520" left servo pulse, "r 1480" right, "b 1500" both, "s 0.5" normalized
// steering through the map and calibration, "p" print state, "c" back to normal operation.
static constexpr int kCalibrationFloorUs = 800;
static constexpr int kCalibrationCeilingUs = 2200;

bool calibration_active = false;
int calibration_left_us = 0;
int calibration_right_us = 0;
char calibration_line[24];
uint8_t calibration_line_length = 0;

Servo left_servo;
Servo right_servo;

uint8_t rx_buffer[kCommandPacketLength] {};
uint8_t rx_index = 0;
uint8_t status_sequence = 0;
uint8_t last_command_sequence = 0;

float target_steering = 0.0f;
float current_steering = 0.0f;

uint32_t last_command_ms = 0;
uint32_t last_status_ms = 0;
uint32_t last_update_ms = 0;
bool have_command = false;
bool center_requested_by_px4 = false;

// Link diagnostics, printed over the USB serial monitor. The USB link is independent of the
// Pixhawk I2C/UART link, so this reports on that link while you wiggle or roll the trolley:
// whether the Pico rebooted (power/firmware) versus kept running and only lost data
// (data-line / pogo-pin problem), how many disconnects there were, and how long each lasted.
void printLinkDiagnostics();

bool g_fresh_boot = true;          // re-initialised to true on every power-up / reset
bool diag_link_up = false;
bool diag_ever_up = false;
uint32_t diag_gap_start_cmd_ms = 0;
uint32_t diag_loss_count = 0;
uint32_t diag_longest_gap_ms = 0;
uint32_t diag_total_down_ms = 0;
uint32_t diag_last_ongoing_ms = 0;
uint32_t diag_last_status_ms = 0;

#if TROLLEY_LINK_USE_I2C
// Written in the I2C receive interrupt, consumed in loop().
volatile uint8_t i2c_rx_data[kCommandPacketLength] {};
volatile uint8_t i2c_rx_length = 0;
volatile bool i2c_rx_pending = false;
// Latest status packet, rebuilt in loop() with interrupts disabled, sent from the request interrupt.
uint8_t i2c_status_out[kStatusPacketLength] {};
#endif

uint8_t checksum(const uint8_t *buffer, const uint8_t length)
{
	uint8_t value = 0;

	for (uint8_t i = 0; i < length; ++i) {
		value ^= buffer[i];
	}

	return value;
}

float constrainSteering(const float value)
{
	if (value > 1.0f) {
		return 1.0f;
	}

	if (value < -1.0f) {
		return -1.0f;
	}

	return value;
}

float slew(const float current, const float target, const float rate_per_second, const float dt)
{
	const float max_delta = rate_per_second * dt;
	const float delta = target - current;

	if (delta > max_delta) {
		return current + max_delta;
	}

	if (delta < -max_delta) {
		return current - max_delta;
	}

	return target;
}

float steeringToServoFraction(const float wheel_fraction)
{
	const float position = (constrainSteering(wheel_fraction) + 1.0f) * 0.5f * (kSteeringMapPoints - 1);
	int index = static_cast<int>(position);

	if (index >= kSteeringMapPoints - 1) {
		index = kSteeringMapPoints - 2;
	}

	const float interpolation = position - index;
	return kSteeringToServoMap[index]
	       + (kSteeringToServoMap[index + 1] - kSteeringToServoMap[index]) * interpolation;
}

int servoPulseFromSteering(float steering, const int min_us, const int center_us, const int max_us,
			   const bool reversed)
{
	steering = constrainSteering(reversed ? -steering : steering);
	const float servo_fraction = steeringToServoFraction(steering);

	if (servo_fraction >= 0.0f) {
		return center_us + static_cast<int>((max_us - center_us) * servo_fraction);
	}

	return center_us + static_cast<int>((center_us - min_us) * servo_fraction);
}

void writeServos(const float steering)
{
	left_servo.writeMicroseconds(servoPulseFromSteering(steering, kLeftMinUs, kLeftCenterUs, kLeftMaxUs,
				     kLeftReversed));
	right_servo.writeMicroseconds(servoPulseFromSteering(steering, kRightMinUs, kRightCenterUs, kRightMaxUs,
				      kRightReversed));
}

bool handleCommandPacket()
{
	if (rx_buffer[2] != kProtocolVersion ||
	    rx_buffer[kCommandPacketLength - 1] != checksum(rx_buffer, kCommandPacketLength - 1)) {
		return false;
	}

	const uint8_t flags = rx_buffer[5];
	const int16_t steering_scaled = static_cast<int16_t>(rx_buffer[6] | (rx_buffer[7] << 8));
	const bool active = (flags & kFlagActive) != 0;
	center_requested_by_px4 = !active || (flags & (kFlagAbort | kFlagCenterWheels | kFlagReleased)) != 0;

	last_command_sequence = rx_buffer[3];
	last_command_ms = millis();
	have_command = true;
	target_steering = center_requested_by_px4 ? 0.0f : constrainSteering(static_cast<float>(steering_scaled) / 10000.0f);
	return true;
}

void parseByte(const uint8_t byte)
{
	if (rx_index == 0 && byte != 'T') {
		return;
	}

	rx_buffer[rx_index++] = byte;

	if (rx_index == 2 && rx_buffer[1] != 'C') {
		rx_index = (byte == 'T') ? 1 : 0;
		return;
	}

	if (rx_index >= kCommandPacketLength) {
		if (handleCommandPacket()) {
			// The checksum byte belongs to this completed packet; do not reuse it as a frame start.
			rx_index = 0;

		} else {
			rx_index = (byte == 'T') ? 1 : 0;
		}
	}
}

void buildStatusPacket(uint8_t *packet, const bool command_fresh, const bool failsafe_centering)
{
	packet[0] = 'T';
	packet[1] = 'S';
	packet[2] = kProtocolVersion;
	packet[3] = kStatusOk | (command_fresh ? kStatusCommandFresh : 0) | (failsafe_centering ? kStatusFailsafeCentering : 0);
	packet[4] = ++status_sequence;
	packet[5] = last_command_sequence;
	packet[6] = 0;
	packet[7] = checksum(packet, kStatusPacketLength - 1);
}

void publishStatus(const bool command_fresh, const bool failsafe_centering)
{
	uint8_t packet[kStatusPacketLength] {};
	buildStatusPacket(packet, command_fresh, failsafe_centering);

#if TROLLEY_LINK_USE_I2C
	noInterrupts();
	memcpy(i2c_status_out, packet, kStatusPacketLength);
	interrupts();
#else
	PixhawkSerial.write(packet, sizeof(packet));
#endif
}

#if TROLLEY_LINK_USE_I2C
void onI2CReceive(int count)
{
	(void)count;
	uint8_t received = 0;

	while (Wire.available()) {
		const uint8_t value = static_cast<uint8_t>(Wire.read());

		if (received < kCommandPacketLength) {
			i2c_rx_data[received++] = value;
		}
	}

	i2c_rx_length = received;
	i2c_rx_pending = true;
}

void onI2CRequest()
{
	Wire.write(i2c_status_out, kStatusPacketLength);
}
#endif

void printCalibrationState()
{
	Serial.print(calibration_active ? "calibration ACTIVE (link steering overridden, 'c' to exit)"
		     : "normal operation");
	Serial.print(" | left ");
	Serial.print(calibration_active ? calibration_left_us
		     : servoPulseFromSteering(current_steering, kLeftMinUs, kLeftCenterUs, kLeftMaxUs, kLeftReversed));
	Serial.print(" us, right ");
	Serial.print(calibration_active ? calibration_right_us
		     : servoPulseFromSteering(current_steering, kRightMinUs, kRightCenterUs, kRightMaxUs, kRightReversed));
	Serial.print(" us | configured L ");
	Serial.print(kLeftMinUs);
	Serial.print("/");
	Serial.print(kLeftCenterUs);
	Serial.print("/");
	Serial.print(kLeftMaxUs);
	Serial.print(" R ");
	Serial.print(kRightMinUs);
	Serial.print("/");
	Serial.print(kRightCenterUs);
	Serial.print("/");
	Serial.println(kRightMaxUs);
}

void startCalibrationFromCurrentOutput()
{
	if (!calibration_active) {
		calibration_left_us = servoPulseFromSteering(current_steering, kLeftMinUs, kLeftCenterUs,
					kLeftMaxUs, kLeftReversed);
		calibration_right_us = servoPulseFromSteering(current_steering, kRightMinUs, kRightCenterUs,
					kRightMaxUs, kRightReversed);
		calibration_active = true;
	}
}

int clampCalibrationPulse(const long pulse_us)
{
	if (pulse_us < kCalibrationFloorUs) {
		return kCalibrationFloorUs;
	}

	if (pulse_us > kCalibrationCeilingUs) {
		return kCalibrationCeilingUs;
	}

	return static_cast<int>(pulse_us);
}

void handleCalibrationLine(const char *line)
{
	const char command = line[0];

	if (command == 'c') {
		calibration_active = false;
		Serial.println("calibration off, wheels return to link/failsafe steering");
		return;
	}

	if (command == 'p') {
		printCalibrationState();
		return;
	}

	if (command == 'l' || command == 'r' || command == 'b') {
		const int pulse = clampCalibrationPulse(strtol(line + 1, nullptr, 10));
		startCalibrationFromCurrentOutput();

		if (command != 'r') {
			calibration_left_us = pulse;
		}

		if (command != 'l') {
			calibration_right_us = pulse;
		}

		printCalibrationState();
		return;
	}

	if (command == 's') {
		const float steering = constrainSteering(strtof(line + 1, nullptr));
		startCalibrationFromCurrentOutput();
		calibration_left_us = servoPulseFromSteering(steering, kLeftMinUs, kLeftCenterUs, kLeftMaxUs,
					kLeftReversed);
		calibration_right_us = servoPulseFromSteering(steering, kRightMinUs, kRightCenterUs, kRightMaxUs,
					kRightReversed);
		printCalibrationState();
		return;
	}

	if (command == 'd') {
		printLinkDiagnostics();
		return;
	}

	Serial.println("commands: l/r/b <pulse us>, s <steering -1..1>, p print, c exit, d link diagnostics");
}

void pollCalibrationSerial()
{
	while (Serial.available() > 0) {
		const char value = static_cast<char>(Serial.read());

		if (value == '\n' || value == '\r') {
			if (calibration_line_length > 0) {
				calibration_line[calibration_line_length] = '\0';
				handleCalibrationLine(calibration_line);
				calibration_line_length = 0;
			}

		} else if (calibration_line_length < sizeof(calibration_line) - 1) {
			calibration_line[calibration_line_length++] = value;
		}
	}
}

void printLinkDiagnostics()
{
	const uint32_t now = millis();
	const bool up = have_command && (now - last_command_ms) <= kCommandTimeoutMs;
	Serial.print("[diag] uptime ");
	Serial.print(now / 1000.0f, 1);
	Serial.print("s | link ");
	Serial.print(up ? "UP" : "DOWN");

	if (!up && diag_ever_up) {
		Serial.print(" for ");
		Serial.print(now - diag_gap_start_cmd_ms);
		Serial.print(" ms");
	}

	Serial.print(" | disconnects ");
	Serial.print(diag_loss_count);
	Serial.print(" | longest ");
	Serial.print(diag_longest_gap_ms);
	Serial.print(" ms | total down ");
	Serial.print(diag_total_down_ms);
	Serial.print(" ms | last cmd seq ");
	Serial.println(last_command_sequence);
}

// Watches command freshness and reports link losses/recoveries over USB serial. Because it runs
// only while the Pico is powered, its continued output during a Pixhawk-link drop proves the Pico
// stayed alive (so the loss is a data-line/pogo-pin fault, not power). A gap is timed exactly from
// the last good command to the next one, and classified GOOD once it reconnects.
void updateLinkDiagnostics(const uint32_t now_ms)
{
	// Only emit when a USB serial monitor is attached; nothing is buffered otherwise.
	if (!static_cast<bool>(Serial)) {
		return;
	}

	const bool up = have_command && (now_ms - last_command_ms) <= kCommandTimeoutMs;

	if (!diag_ever_up) {
		if (up) {
			diag_ever_up = true;
			diag_link_up = true;
			diag_last_status_ms = now_ms;
			Serial.println("[diag] LINK UP (first command received)");
		}

		return;
	}

	if (diag_link_up && !up) {
		diag_link_up = false;
		diag_gap_start_cmd_ms = last_command_ms;
		diag_loss_count++;
		diag_last_ongoing_ms = now_ms;
		Serial.print("[diag] LINK LOST @ ");
		Serial.print(now_ms / 1000.0f, 1);
		Serial.print("s  (disconnect #");
		Serial.print(diag_loss_count);
		Serial.println(")");

	} else if (!diag_link_up && up) {
		diag_link_up = true;
		const uint32_t gap = last_command_ms - diag_gap_start_cmd_ms;
		diag_total_down_ms += gap;

		if (gap > diag_longest_gap_ms) {
			diag_longest_gap_ms = gap;
		}

		Serial.print("[diag] LINK RECOVERED after ");
		Serial.print(gap);
		Serial.print(" ms  -> GOOD (reconnected).  longest so far ");
		Serial.print(diag_longest_gap_ms);
		Serial.println(" ms");
	}

	if (!diag_link_up) {
		if ((now_ms - diag_last_ongoing_ms) >= 500) {
			diag_last_ongoing_ms = now_ms;
			Serial.print("[diag]   ...still disconnected ");
			Serial.print(now_ms - diag_gap_start_cmd_ms);
			Serial.println(" ms  (BAD if it never comes back)");
		}

	} else if ((now_ms - diag_last_status_ms) >= 2000) {
		diag_last_status_ms = now_ms;
		Serial.print("[diag] link UP  uptime ");
		Serial.print(now_ms / 1000.0f, 1);
		Serial.print("s  disconnects ");
		Serial.print(diag_loss_count);
		Serial.print("  longest ");
		Serial.print(diag_longest_gap_ms);
		Serial.println(" ms");
	}
}

void setup()
{
	Serial.begin(115200);

#if TROLLEY_LINK_USE_I2C
	// A valid (stale) status packet must exist before the first master read.
	buildStatusPacket(i2c_status_out, false, true);
	Wire.setSDA(kI2cSdaPin);
	Wire.setSCL(kI2cSclPin);
	Wire.onReceive(onI2CReceive);
	Wire.onRequest(onI2CRequest);
	Wire.begin(kI2cAddress);
#else
	// Serial1 uses the Pico's standard UART0 mapping: GP0 TX and GP1 RX.
	PixhawkSerial.begin(115200);
#endif

	left_servo.attach(kLeftServoPin, kServoAttachMinUs, kServoAttachMaxUs);
	right_servo.attach(kRightServoPin, kServoAttachMinUs, kServoAttachMaxUs);

	last_update_ms = millis();
	writeServos(0.0f);
}

void loop()
{
	// Greet whenever a serial monitor (re)connects; a boot-time print would be dropped
	// because USB serial output goes nowhere until the host opens the port.
	static bool serial_monitor_was_connected = false;
	const bool serial_monitor_connected = static_cast<bool>(Serial);

	if (serial_monitor_connected && !serial_monitor_was_connected) {
		if (g_fresh_boot) {
			g_fresh_boot = false;
			Serial.println("===== TROLLEY PICO BOOTED (fresh power-up / reset) =====");
			Serial.println("[diag] If this BOOTED banner reappears during a test, the Pico REBOOTED (power/firmware).");
			Serial.println("[diag] If the link drops but [diag] keeps printing, the Pico is alive -> data-line/pogo fault.");

		} else {
			Serial.println("(serial monitor reconnected - Pico did NOT reboot)");
		}

		Serial.println("Trolley Pico servo controller ready.");
		Serial.println("Commands: l/r/b <us>, s <steering>, p print, c exit  |  d = link diagnostics");
	}

	serial_monitor_was_connected = serial_monitor_connected;

	pollCalibrationSerial();

#if TROLLEY_LINK_USE_I2C

	if (i2c_rx_pending) {
		uint8_t packet[kCommandPacketLength] {};
		noInterrupts();
		const uint8_t length = i2c_rx_length;

		for (uint8_t i = 0; i < length; ++i) {
			packet[i] = i2c_rx_data[i];
		}

		i2c_rx_pending = false;
		interrupts();

		// Every I2C write is one aligned command packet.
		rx_index = 0;

		for (uint8_t i = 0; i < length; ++i) {
			parseByte(packet[i]);
		}
	}

#else

	while (PixhawkSerial.available() > 0) {
		parseByte(static_cast<uint8_t>(PixhawkSerial.read()));
	}

#endif

	const uint32_t now_ms = millis();
	float dt = (now_ms - last_update_ms) * 0.001f;
	last_update_ms = now_ms;

	if (dt > 0.1f) {
		dt = 0.1f;
	}

	const bool command_fresh = have_command && (now_ms - last_command_ms) <= kCommandTimeoutMs;
	const bool failsafe_centering = !command_fresh;
	const float desired_steering = command_fresh ? target_steering : 0.0f;
	// PX4-commanded centering (release/abort with the link still connected) uses the normal
	// command rate so the wheels are straight before the connector separates. Only autonomous
	// centering after link loss slews gently to avoid upsetting a free-rolling trolley.
	const float slew_rate = failsafe_centering ? kFailsafeCenterSlewPerSecond : kCommandSlewPerSecond;

	current_steering = slew(current_steering, desired_steering, slew_rate, dt);

	if (calibration_active) {
		// Bench calibration overrides the link/failsafe steering until 'c' or a power cycle.
		left_servo.writeMicroseconds(calibration_left_us);
		right_servo.writeMicroseconds(calibration_right_us);

	} else {
		writeServos(current_steering);
	}

	if ((now_ms - last_status_ms) >= kStatusPeriodMs) {
		last_status_ms = now_ms;
		publishStatus(command_fresh, failsafe_centering);
	}

	updateLinkDiagnostics(now_ms);
}
