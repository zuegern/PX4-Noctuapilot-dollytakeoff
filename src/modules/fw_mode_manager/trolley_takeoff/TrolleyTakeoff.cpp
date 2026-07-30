/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file TrolleyTakeoff.cpp
 * Trolley-assisted takeoff handling for fixed-wing UAVs.
 */

#include "TrolleyTakeoff.h"

#include <float.h>
#include <math.h>

#include <mathlib/mathlib.h>
#include <px4_platform_common/events.h>

using namespace time_literals;

namespace trolleytakeoff
{

	// Minimum time the alignment heading condition must hold before the throttle ramp starts.
	static constexpr hrt_abstime kAlignConditionHoldTime = 500_ms;

	void TrolleyTakeoff::init(const hrt_abstime &time_now, const bool trolley_link_required, const bool trolley_link_healthy)
	{
		// Alignment taxiing only helps the closed-loop path modes; heading hold follows the current
		// yaw anyway and open loop cannot measure an alignment error.
		const bool align_phase_enabled = param_trolley_aln_thr_.get() > FLT_EPSILON
						 && (param_trolley_str_mode_.get() == TrolleySteeringMode::STEERING_PATH_TRACKING
						     || param_trolley_str_mode_.get() == TrolleySteeringMode::STEERING_PATH_HEADING);

		takeoff_state_ = align_phase_enabled ? TrolleyTakeoffState::ALIGN_TO_PATH : TrolleyTakeoffState::THROTTLE_RAMP;
		initialized_ = true;
		time_initialized_ = time_now;
		takeoff_time_ = 0;
		ramp_start_time_ = time_now;
		time_last_steering_update_ = time_now;
		release_condition_met_since_ = 0;
		align_condition_met_since_ = 0;
		trolley_link_healthy_ = true;
		release_authorized_ = false;
		bad_disconnect_detected_ = false;
		good_disconnect_detected_ = false;
		wheel_steering_setpoint_ = 0.f;
		updateLinkState(trolley_link_required, trolley_link_healthy);
	}

	void TrolleyTakeoff::update(const hrt_abstime &time_now, const float takeoff_airspeed, const float calibrated_airspeed,
				    const float estimated_ground_speed, const float vehicle_altitude, const float clearance_altitude,
				    const float path_heading_error, const bool trolley_link_required, const bool trolley_link_healthy)
	{
		updateLinkState(trolley_link_required, trolley_link_healthy);

		if (trolley_link_required && !trolley_link_healthy_)
		{
			if (!release_authorized_ && !bad_disconnect_detected_)
			{
				takeoff_state_ = TrolleyTakeoffState::ABORTED;
				wheel_steering_setpoint_ = 0.f;
				bad_disconnect_detected_ = true;
				events::send(events::ID("trolley_takeoff_bad_disconnect"), events::Log::Critical,
					     "Trolley link lost before release, aborting takeoff");
			}

			if (release_authorized_ && !good_disconnect_detected_)
			{
				good_disconnect_detected_ = true;
				events::send(events::ID("trolley_takeoff_released"), events::Log::Info,
					     "Trolley link lost after release");
			}
		}

		if (takeoff_state_ == TrolleyTakeoffState::ABORTED)
		{
			return;
		}

		switch (takeoff_state_)
		{
			case TrolleyTakeoffState::ALIGN_TO_PATH:
				if (PX4_ISFINITE(path_heading_error)
				    && fabsf(path_heading_error) < math::radians(math::max(param_trolley_aln_err_.get(), 1.f)))
				{
					if (align_condition_met_since_ == 0)
					{
						align_condition_met_since_ = time_now;
					}

					if ((time_now - align_condition_met_since_) >= kAlignConditionHoldTime)
					{
						takeoff_state_ = TrolleyTakeoffState::THROTTLE_RAMP;
						ramp_start_time_ = time_now;
						events::send(events::ID("trolley_takeoff_aligned"), events::Log::Info,
							     "Trolley aligned with takeoff path, ramping throttle");
					}

				}

				else
				{
					align_condition_met_since_ = 0;
				}

				if (takeoff_state_ == TrolleyTakeoffState::ALIGN_TO_PATH
				    && (time_now - time_initialized_) > (math::max(param_trolley_aln_to_.get(), 1.f) * 1_s))
				{
					events::send(events::ID("trolley_takeoff_align_timeout"), events::Log::Critical,
						     "Trolley path alignment timed out, aborting takeoff");
					abort();
				}

				break;

			case TrolleyTakeoffState::THROTTLE_RAMP:
				if ((time_now - ramp_start_time_) > (param_trolley_ramp_time_.get() * 1_s))
				{
					takeoff_state_ = TrolleyTakeoffState::CLAMPED_TO_TROLLEY;
				}

				break;

			case TrolleyTakeoffState::CLAMPED_TO_TROLLEY:
				if (releaseConditionReached(time_now, takeoff_airspeed, calibrated_airspeed, estimated_ground_speed))
				{
					if (release_condition_met_since_ == 0)
					{
						release_condition_met_since_ = time_now;
					}

					// Speed-based conditions must hold for TROLLEY_RLS_HOLD so a single gust or
					// estimation spike does not trigger rotation. The time condition stays exact.
					const bool hold_time_elapsed = param_trolley_tk_cond_.get() == TrolleyTakeoffCondition::CONDITION_TIME
								       || (time_now - release_condition_met_since_) >=
								       (math::max(param_trolley_rls_hold_.get(), 0.f) * 1_s);

					if (hold_time_elapsed)
					{
						takeoff_time_ = time_now;
						takeoff_state_ = TrolleyTakeoffState::CLIMBOUT;
						release_authorized_ = true;
						events::send(events::ID("trolley_takeoff_release"), events::Log::Info,
							     "Trolley takeoff release condition reached, climbout");
					}

				}

				else
				{
					release_condition_met_since_ = 0;
				}

				break;

			case TrolleyTakeoffState::CLIMBOUT:
				if (vehicle_altitude > clearance_altitude)
				{
					takeoff_state_ = TrolleyTakeoffState::FLYING;
					events::send(events::ID("trolley_takeoff_reached_clearance_altitude"), events::Log::Info,
						     "Trolley takeoff reached clearance altitude");
				}

				break;

			default:
				break;
		}
	}

	bool TrolleyTakeoff::wheelSteeringEnabled() const
	{
		if (isAborted())
		{
			return false;
		}

		if (takeoff_state_ < TrolleyTakeoffState::CLIMBOUT)
		{
			return true;
		}

		return takeoff_state_ == TrolleyTakeoffState::CLIMBOUT
		       && takeoff_time_ != 0
		       && (hrt_elapsed_time(&takeoff_time_) < (param_trolley_str_hold_.get() * 1_s)
			   || fabsf(wheel_steering_setpoint_) > 0.01f);
	}

	void TrolleyTakeoff::abort()
	{
		if (takeoff_state_ < TrolleyTakeoffState::CLIMBOUT)
		{
			takeoff_state_ = TrolleyTakeoffState::ABORTED;
			release_authorized_ = false;
		}

		wheel_steering_setpoint_ = 0.f;
	}

	bool TrolleyTakeoff::directWheelSteeringEnabled() const
	{
		if (!wheelSteeringEnabled())
		{
			return false;
		}

		if (takeoff_state_ >= TrolleyTakeoffState::CLIMBOUT)
		{
			return true;
		}

		return pathTrackingSteeringEnabled() || openLoopSteeringEnabled();
	}

	bool TrolleyTakeoff::headingSteeringEnabled() const
	{
		return takeoff_state_ < TrolleyTakeoffState::CLIMBOUT
		       && param_trolley_str_mode_.get() == TrolleySteeringMode::STEERING_HEADING;
	}

	bool TrolleyTakeoff::pathTrackingSteeringEnabled() const
	{
		return takeoff_state_ < TrolleyTakeoffState::CLIMBOUT
		       && param_trolley_str_mode_.get() == TrolleySteeringMode::STEERING_PATH_TRACKING;
	}

	bool TrolleyTakeoff::openLoopSteeringEnabled() const
	{
		return takeoff_state_ < TrolleyTakeoffState::CLIMBOUT
		       && param_trolley_str_mode_.get() == TrolleySteeringMode::STEERING_OPEN_LOOP;
	}

	bool TrolleyTakeoff::pathHeadingSteeringEnabled() const
	{
		return takeoff_state_ < TrolleyTakeoffState::CLIMBOUT
		       && param_trolley_str_mode_.get() == TrolleySteeringMode::STEERING_PATH_HEADING;
	}

	bool TrolleyTakeoff::wheelControllerSteeringEnabled() const
	{
		return headingSteeringEnabled() || pathHeadingSteeringEnabled();
	}

	bool TrolleyTakeoff::navigationSteeringEnabled() const
	{
		return pathTrackingSteeringEnabled() || pathHeadingSteeringEnabled();
	}

	float TrolleyTakeoff::minimumTurnRadius() const
	{
		const float wheelbase = param_trolley_wheelbase_.get();
		const float max_steering_angle = math::radians(param_trolley_str_max_.get());
		const float reference_x = param_trolley_ref_x_.get();

		if (wheelbase <= FLT_EPSILON || max_steering_angle <= FLT_EPSILON)
		{
			return NAN;
		}

		const float max_steering_tangent = tanf(max_steering_angle);

		if (max_steering_tangent <= FLT_EPSILON)
		{
			return NAN;
		}

		const float rear_axle_min_radius = wheelbase / max_steering_tangent;

		return sqrtf(rear_axle_min_radius * rear_axle_min_radius + reference_x * reference_x);
	}

	float TrolleyTakeoff::effectivePathRadius() const
	{
		const float requested_reference_radius = param_trolley_radius_.get();
		const float min_reference_radius = minimumTurnRadius();

		if (!PX4_ISFINITE(min_reference_radius))
		{
			return requested_reference_radius;
		}

		return math::max(requested_reference_radius, min_reference_radius);
	}

	float TrolleyTakeoff::rearAxleTurnRadius(const float reference_radius) const
	{
		const float reference_x = param_trolley_ref_x_.get();

		if (!PX4_ISFINITE(reference_radius) || reference_radius <= FLT_EPSILON)
		{
			return NAN;
		}

		if (fabsf(reference_x) <= FLT_EPSILON)
		{
			return reference_radius;
		}

		// Track width is intentionally not used here: steering uses a centerline bicycle model, not inner-wheel radius.
		const float rear_axle_radius_squared = reference_radius * reference_radius - reference_x * reference_x;

		return rear_axle_radius_squared > FLT_EPSILON ? sqrtf(rear_axle_radius_squared) : NAN;
	}

	void TrolleyTakeoff::updateWheelSteeringSetpoint(const hrt_abstime &time_now, const float target_setpoint)
	{
		float steering_setpoint = PX4_ISFINITE(target_setpoint) ? math::constrain(target_setpoint, -1.f, 1.f) : 0.f;

		if (isAborted() || takeoff_state_ >= TrolleyTakeoffState::CLIMBOUT)
		{
			steering_setpoint = 0.f;
		}

		if (time_last_steering_update_ == 0)
		{
			wheel_steering_setpoint_ = steering_setpoint;
			time_last_steering_update_ = time_now;
			return;
		}

		const float slew_rate = param_trolley_str_rate_.get();

		if (slew_rate <= FLT_EPSILON)
		{
			wheel_steering_setpoint_ = steering_setpoint;

		}

		else
		{
			const float dt = math::max((time_now - time_last_steering_update_) * 1.e-6f, 0.f);
			const float max_delta = slew_rate * dt;
			const float delta = math::constrain(steering_setpoint - wheel_steering_setpoint_, -max_delta, max_delta);
			wheel_steering_setpoint_ += delta;
		}

		time_last_steering_update_ = time_now;
	}

	TrolleyControlConfig TrolleyTakeoff::controlConfig() const
	{
		// The low-speed feedback fade (TROLLEY_STR_VLO/VHI) smooths the accelerating ground roll, but the
		// alignment taxi deliberately runs below those speeds and must keep full steering authority to point
		// the trolley onto the path before the ramp. So the fade is only applied once the throttle ramp begins.
		const bool fade_low_speed_feedback = takeoff_state_ >= TrolleyTakeoffState::THROTTLE_RAMP;
		return {
			.wheelbase = param_trolley_wheelbase_.get(),
			.reference_offset = param_trolley_ref_x_.get(),
			.max_steering_angle = math::radians(param_trolley_str_max_.get()),
			.heading_gain = param_trolley_trk_gain_.get(),
			.cross_track_gain = param_trolley_xtk_gain_.get(),
			.max_lateral_acceleration = param_trolley_lat_acc_.get(),
			.feedback_speed_zero = fade_low_speed_feedback ? param_trolley_str_vlo_.get() : 0.f,
			.feedback_speed_full = fade_low_speed_feedback ? param_trolley_str_vhi_.get() : 0.f,
		};
	}

	float TrolleyTakeoff::openLoopWheelSteeringSetpoint(const float longitudinal_speed) const
	{
		if (isAborted() || takeoff_state_ >= TrolleyTakeoffState::CLIMBOUT)
		{
			return 0.f;
		}

		const int32_t path_type = param_trolley_path_.get();

		if (path_type == TrolleyPathType::PATH_STRAIGHT)
		{
			return 0.f;
		}

		if (path_type != TrolleyPathType::PATH_CONSTANT_RIGHT && path_type != TrolleyPathType::PATH_CONSTANT_LEFT)
		{
			return 0.f;
		}

		const float reference_radius = effectivePathRadius();
		const float direction = (path_type == TrolleyPathType::PATH_CONSTANT_RIGHT) ? 1.f : -1.f;
		const float curvature = direction / reference_radius;
		const float offset_curvature = param_trolley_ref_x_.get() * curvature;

		if (!PX4_ISFINITE(curvature) || fabsf(offset_curvature) >= 1.f)
		{
			return 0.f;
		}

		TrolleyPathState path_state{};
		path_state.heading = 0.f;
		path_state.error = 0.f;
		path_state.curvature = curvature;
		path_state.valid = true;

		const float desired_yaw = -asinf(offset_curvature);
		const float speed = PX4_ISFINITE(longitudinal_speed) ? fabsf(longitudinal_speed) : 0.f;
		const TrolleyControlOutput output = calculateTrolleyControl(path_state, desired_yaw, speed, controlConfig());

		return output.valid ? output.normalized_steering : 0.f;
	}

	TrolleyControlOutput TrolleyTakeoff::pathTrackingWheelSteeringSetpoint(const TrolleyPathState &path_state,
			const float yaw, const float longitudinal_speed) const
	{
		return calculateTrolleyControl(path_state, yaw, longitudinal_speed, controlConfig());
	}

	float TrolleyTakeoff::rotationAirspeedThreshold(const float takeoff_airspeed) const
	{
		return (param_trolley_rot_airspd_.get() > FLT_EPSILON) ?
		       math::min(param_trolley_rot_airspd_.get(), takeoff_airspeed) :
		       0.9f * takeoff_airspeed;
	}

	float TrolleyTakeoff::rotationGroundspeedThreshold(const float takeoff_airspeed) const
	{
		return (param_trolley_rot_gspd_.get() > FLT_EPSILON) ?
		       param_trolley_rot_gspd_.get() :
		       rotationAirspeedThreshold(takeoff_airspeed);
	}

	bool TrolleyTakeoff::releaseConditionReached(const hrt_abstime &time_now, const float takeoff_airspeed,
			const float calibrated_airspeed, const float estimated_ground_speed) const
	{
		switch (param_trolley_tk_cond_.get())
		{
			case TrolleyTakeoffCondition::CONDITION_GROUNDSPEED:
				return PX4_ISFINITE(estimated_ground_speed)
				       && estimated_ground_speed > rotationGroundspeedThreshold(takeoff_airspeed);

			case TrolleyTakeoffCondition::CONDITION_TIME:
				// Measured from the throttle ramp start so a preceding alignment phase does not consume
				// the configured test duration.
				return (time_now - ramp_start_time_) > (math::max(param_trolley_tk_time_.get(), 0.f) * 1_s);

			case TrolleyTakeoffCondition::CONDITION_AIRSPEED:
			default:
				return PX4_ISFINITE(calibrated_airspeed) && calibrated_airspeed > rotationAirspeedThreshold(takeoff_airspeed);
		}
	}

	void TrolleyTakeoff::updateLinkState(const bool trolley_link_required, const bool trolley_link_healthy)
	{
		trolley_link_healthy_ = !trolley_link_required || trolley_link_healthy;
	}

	float TrolleyTakeoff::getPitch() const
	{
		if (takeoff_state_ <= TrolleyTakeoffState::CLAMPED_TO_TROLLEY)
		{
			return math::radians(param_trolley_psp_.get());
		}

		return NAN;
	}

	float TrolleyTakeoff::getThrottle(const float idle_throttle) const
	{
		float throttle = idle_throttle;

		switch (takeoff_state_)
		{
			case TrolleyTakeoffState::ALIGN_TO_PATH:
				throttle = math::constrain(param_trolley_aln_thr_.get(), 0.f, param_trolley_max_thr_.get());
				break;

			case TrolleyTakeoffState::THROTTLE_RAMP:
				throttle = interpolateValuesOverAbsoluteTime(idle_throttle, param_trolley_max_thr_.get(), ramp_start_time_,
						param_trolley_ramp_time_.get());
				break;

			case TrolleyTakeoffState::CLAMPED_TO_TROLLEY:
			case TrolleyTakeoffState::CLIMBOUT:
				throttle = param_trolley_max_thr_.get();
				break;

			case TrolleyTakeoffState::FLYING:
				throttle = NAN;
				break;

			case TrolleyTakeoffState::ABORTED:
				throttle = idle_throttle;
				break;
		}

		return throttle;
	}

	float TrolleyTakeoff::getMinPitch(float min_pitch_in_climbout, float min_pitch) const
	{
		if (takeoff_state_ < TrolleyTakeoffState::CLIMBOUT)
		{
			return math::radians(param_trolley_psp_.get() - 0.01f);

		}

		else if (takeoff_state_ < TrolleyTakeoffState::FLYING)
		{
			const float trolley_pitch_min = math::radians(param_trolley_psp_.get() - 0.01f);
			return interpolateValuesOverAbsoluteTime(trolley_pitch_min, min_pitch_in_climbout, takeoff_time_,
					param_trolley_rot_time_.get());

		}

		else
		{
			return min_pitch;
		}
	}

	float TrolleyTakeoff::getMaxPitch(const float max_pitch) const
	{
		if (takeoff_state_ < TrolleyTakeoffState::CLIMBOUT)
		{
			return math::radians(param_trolley_psp_.get() + 0.01f);

		}

		else if (takeoff_state_ < TrolleyTakeoffState::FLYING)
		{
			const float trolley_pitch_max = math::radians(param_trolley_psp_.get() + 0.01f);
			return interpolateValuesOverAbsoluteTime(trolley_pitch_max, max_pitch, takeoff_time_,
					param_trolley_rot_time_.get());

		}

		else
		{
			return max_pitch;
		}
	}

	void TrolleyTakeoff::reset()
	{
		initialized_ = false;
		takeoff_state_ = TrolleyTakeoffState::THROTTLE_RAMP;
		takeoff_time_ = 0;
		ramp_start_time_ = 0;
		time_last_steering_update_ = 0;
		release_condition_met_since_ = 0;
		align_condition_met_since_ = 0;
		trolley_link_healthy_ = true;
		release_authorized_ = false;
		bad_disconnect_detected_ = false;
		good_disconnect_detected_ = false;
		wheel_steering_setpoint_ = 0.f;
	}

	float TrolleyTakeoff::interpolateValuesOverAbsoluteTime(const float start_value, const float end_value,
			const hrt_abstime &start_time, const float interpolation_time) const
	{
		if (interpolation_time <= FLT_EPSILON)
		{
			return end_value;
		}

		const float seconds_since_start = hrt_elapsed_time(&start_time) * 1.e-6f;
		const float interpolator = math::constrain(seconds_since_start / interpolation_time, 0.0f, 1.0f);

		return interpolator * end_value + (1.0f - interpolator) * start_value;
	}

} // namespace trolleytakeoff
