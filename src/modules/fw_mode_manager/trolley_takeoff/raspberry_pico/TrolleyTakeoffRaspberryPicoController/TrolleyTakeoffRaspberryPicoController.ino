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

#define TROLLEY_LINK_USE_I2C 1

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

// Calibrate these for your trolley. Values are servo PWM microseconds.
static constexpr int kLeftMinUs = 1100;
static constexpr int kLeftCenterUs = 1500;
static constexpr int kLeftMaxUs = 1900;
static constexpr bool kLeftReversed = false;

static constexpr int kRightMinUs = 1100;
static constexpr int kRightCenterUs = 1500;
static constexpr int kRightMaxUs = 1900;
static constexpr bool kRightReversed = false;

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
// Wheel -10 deg is first reached at crank -77 deg and +10 deg at crank +90 deg, so calibrate
// kMin/Center/MaxUs to the pulses where the wheel FIRST reaches -10 / 0 / +10 degrees and verify
// on the trolley (tire flex and linkage slop are not in this model).
// For a plain linear servo-to-wheel setup use the identity table:
// {-1.0f, -0.75f, -0.5f, -0.25f, 0.0f, 0.25f, 0.5f, 0.75f, 1.0f}
static constexpr float kSteeringToServoMap[] = {
	-1.0f, -0.602f, -0.379f, -0.186f, 0.0f, 0.165f, 0.347f, 0.574f, 1.0f
};

static constexpr int kSteeringMapPoints = sizeof(kSteeringToServoMap) / sizeof(kSteeringToServoMap[0]);
static_assert(kSteeringMapPoints >= 2, "Steering map needs at least two points");
static_assert(kSteeringMapPoints % 2 == 1, "Steering map needs a center point (odd point count)");

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

	left_servo.attach(kLeftServoPin);
	right_servo.attach(kRightServoPin);

	last_update_ms = millis();
	writeServos(0.0f);
}

void loop()
{
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
	writeServos(current_steering);

	if ((now_ms - last_status_ms) >= kStatusPeriodMs) {
		last_status_ms = now_ms;
		publishStatus(command_fresh, failsafe_centering);
	}
}
