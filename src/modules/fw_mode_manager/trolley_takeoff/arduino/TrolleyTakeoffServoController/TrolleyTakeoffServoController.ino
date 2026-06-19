/*
 * PX4 trolley takeoff servo controller.
 *
 * The Arduino always drives the trolley steering servos. PX4 sends steering
 * commands over UART while the aircraft is attached to the trolley. If the
 * UART link is lost, the controller slowly returns both wheels to center.
 */

#include <Servo.h>

#define PixhawkSerial Serial1

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

// Command tracking may be faster; failsafe centering should be gentle.
static constexpr float kCommandSlewPerSecond = 3.0f;
static constexpr float kFailsafeCenterSlewPerSecond = 0.8f;

static constexpr int kLeftServoPin = 9;
static constexpr int kRightServoPin = 10;

// Calibrate these for your trolley. Values are PWM microseconds.
static constexpr int kLeftMinUs = 1100;
static constexpr int kLeftCenterUs = 1500;
static constexpr int kLeftMaxUs = 1900;
static constexpr bool kLeftReversed = false;

static constexpr int kRightMinUs = 1100;
static constexpr int kRightCenterUs = 1500;
static constexpr int kRightMaxUs = 1900;
static constexpr bool kRightReversed = false;

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

int servoPulseFromSteering(float steering, const int min_us, const int center_us, const int max_us, const bool reversed)
{
	steering = constrainSteering(reversed ? -steering : steering);

	if (steering >= 0.0f) {
		return center_us + static_cast<int>((max_us - center_us) * steering);
	}

	return center_us + static_cast<int>((center_us - min_us) * steering);
}

void writeServos(const float steering)
{
	left_servo.writeMicroseconds(servoPulseFromSteering(steering, kLeftMinUs, kLeftCenterUs, kLeftMaxUs, kLeftReversed));
	right_servo.writeMicroseconds(servoPulseFromSteering(steering, kRightMinUs, kRightCenterUs, kRightMaxUs, kRightReversed));
}

void handleCommandPacket()
{
	if (rx_buffer[2] != kProtocolVersion ||
	    rx_buffer[kCommandPacketLength - 1] != checksum(rx_buffer, kCommandPacketLength - 1)) {
		return;
	}

	const uint8_t flags = rx_buffer[5];
	const int16_t steering_scaled = static_cast<int16_t>(rx_buffer[6] | (rx_buffer[7] << 8));
	center_requested_by_px4 = (flags & (kFlagAbort | kFlagCenterWheels | kFlagReleased)) != 0;

	last_command_sequence = rx_buffer[3];
	last_command_ms = millis();
	have_command = true;
	target_steering = center_requested_by_px4 ? 0.0f : constrainSteering(static_cast<float>(steering_scaled) / 10000.0f);
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
		handleCommandPacket();
		rx_index = 0;
	}
}

void sendStatus(const bool command_fresh, const bool failsafe_centering)
{
	uint8_t packet[kStatusPacketLength] {};
	packet[0] = 'T';
	packet[1] = 'S';
	packet[2] = kProtocolVersion;
	packet[3] = kStatusOk | (command_fresh ? kStatusCommandFresh : 0) | (failsafe_centering ? kStatusFailsafeCentering : 0);
	packet[4] = ++status_sequence;
	packet[5] = last_command_sequence;
	packet[6] = 0;
	packet[7] = checksum(packet, kStatusPacketLength - 1);

	PixhawkSerial.write(packet, sizeof(packet));
}

void setup()
{
	Serial.begin(115200);
	PixhawkSerial.begin(115200);

	left_servo.attach(kLeftServoPin);
	right_servo.attach(kRightServoPin);

	last_update_ms = millis();
	writeServos(0.0f);
}

void loop()
{
	while (PixhawkSerial.available() > 0) {
		parseByte(static_cast<uint8_t>(PixhawkSerial.read()));
	}

	const uint32_t now_ms = millis();
	const float dt = (now_ms - last_update_ms) * 0.001f;
	last_update_ms = now_ms;

	const bool command_fresh = have_command && (now_ms - last_command_ms) <= kCommandTimeoutMs;
	const bool failsafe_centering = !command_fresh;
	const float desired_steering = command_fresh ? target_steering : 0.0f;
	const bool slow_centering = failsafe_centering || center_requested_by_px4;
	const float slew_rate = slow_centering ? kFailsafeCenterSlewPerSecond : kCommandSlewPerSecond;

	current_steering = slew(current_steering, desired_steering, slew_rate, dt);
	writeServos(current_steering);

	if ((now_ms - last_status_ms) >= kStatusPeriodMs) {
		last_status_ms = now_ms;
		sendStatus(command_fresh, failsafe_centering);
	}
}
