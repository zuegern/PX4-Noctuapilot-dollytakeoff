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

#include "TrolleyControl.h"

#include <float.h>
#include <math.h>

#include <mathlib/mathlib.h>

namespace trolleytakeoff
{

using matrix::Vector2f;

static bool vectorFinite(const Vector2f &vector)
{
	return PX4_ISFINITE(vector(0)) && PX4_ISFINITE(vector(1));
}

TrolleyPathState calculateStraightPathState(const Vector2f &start, const float bearing, const Vector2f &position)
{
	TrolleyPathState state{};

	if (!vectorFinite(start) || !vectorFinite(position) || !PX4_ISFINITE(bearing)) {
		return state;
	}

	state.tangent = Vector2f{cosf(bearing), sinf(bearing)};
	const Vector2f start_to_position = position - start;
	state.closest_point = start + state.tangent * start_to_position.dot(state.tangent);
	const Vector2f path_to_position = position - state.closest_point;
	state.heading = matrix::wrap_pi(bearing);
	state.error = state.tangent(0) * path_to_position(1) - state.tangent(1) * path_to_position(0);
	state.curvature = 0.f;
	state.valid = true;
	return state;
}

TrolleyPathState calculateCircularPathState(const Vector2f &start, const float bearing, const float radius,
		const bool right_turn, const Vector2f &position)
{
	TrolleyPathState state{};

	if (!vectorFinite(start) || !vectorFinite(position) || !PX4_ISFINITE(bearing)
	    || !PX4_ISFINITE(radius) || radius <= FLT_EPSILON) {
		return state;
	}

	const Vector2f tangent_at_start{cosf(bearing), sinf(bearing)};
	const Vector2f center_direction = right_turn ?
					  Vector2f{-tangent_at_start(1), tangent_at_start(0)} :
					  Vector2f{tangent_at_start(1), -tangent_at_start(0)};
	const Vector2f center = start + center_direction * radius;
	Vector2f radial = position - center;

	if (radial.norm() <= FLT_EPSILON) {
		// Every point on the circle is equally close, so no unique path tangent exists.
		return state;
	}

	radial.normalize();
	state.closest_point = center + radial * radius;
	state.tangent = right_turn ? Vector2f{-radial(1), radial(0)} : Vector2f{radial(1), -radial(0)};
	const Vector2f path_to_position = position - state.closest_point;
	state.heading = atan2f(state.tangent(1), state.tangent(0));
	state.error = state.tangent(0) * path_to_position(1) - state.tangent(1) * path_to_position(0);
	state.curvature = (right_turn ? 1.f : -1.f) / radius;
	state.valid = true;
	return state;
}

TrolleyControlOutput calculateTrolleyControl(const TrolleyPathState &path, const float yaw,
		const float longitudinal_speed,
		const TrolleyControlConfig &config)
{
	TrolleyControlOutput output{};

	if (!path.valid || !PX4_ISFINITE(path.heading) || !PX4_ISFINITE(path.error)
	    || !PX4_ISFINITE(path.curvature) || !PX4_ISFINITE(yaw) || !PX4_ISFINITE(longitudinal_speed)
	    || !PX4_ISFINITE(config.wheelbase) || config.wheelbase <= FLT_EPSILON
	    || !PX4_ISFINITE(config.reference_offset)
	    || !PX4_ISFINITE(config.max_steering_angle) || config.max_steering_angle <= FLT_EPSILON
	    || !PX4_ISFINITE(config.heading_gain) || config.heading_gain < 0.f
	    || !PX4_ISFINITE(config.cross_track_gain) || config.cross_track_gain < 0.f) {
		return output;
	}

	const float offset_curvature = config.reference_offset * path.curvature;

	if (fabsf(offset_curvature) >= 1.f) {
		return output;
	}

	// Sufficient negative-feedback stability condition from Qin and Li, Proposition 1.
	// Their proof assumes a forward-mounted reference point and constant path curvature.
	if (config.reference_offset >= 0.f) {
		const float steering_tangent = tanf(config.max_steering_angle);
		const float denominator = config.wheelbase
					  * sqrtf(config.wheelbase * config.wheelbase
						  + config.reference_offset * config.reference_offset
						  * steering_tangent * steering_tangent);
		output.minimum_cross_track_gain = config.reference_offset * steering_tangent * steering_tangent / denominator;
		output.stability_condition_met = config.heading_gain > FLT_EPSILON
						 && config.cross_track_gain > output.minimum_cross_track_gain;
	}

	output.steering_limit = config.max_steering_angle;

	if (PX4_ISFINITE(config.max_lateral_acceleration) && config.max_lateral_acceleration > FLT_EPSILON
	    && fabsf(longitudinal_speed) > 0.1f) {
		const float acceleration_limited_angle =
			atanf(config.max_lateral_acceleration * config.wheelbase
			      / (longitudinal_speed * longitudinal_speed));
		output.steering_limit = math::min(output.steering_limit, acceleration_limited_angle);
	}

	if (output.steering_limit <= FLT_EPSILON) {
		return output;
	}

	// Kinematic bicycle feedforward for a path reference point offset from the rear axle.
	const float radius_scale = sqrtf(math::max(1.f - offset_curvature * offset_curvature, FLT_EPSILON));
	output.feedforward_angle = atanf(config.wheelbase * path.curvature / radius_scale);
	output.desired_yaw_offset = -asinf(offset_curvature);
	output.heading_error = matrix::wrap_pi(matrix::wrap_pi(yaw - path.heading) - output.desired_yaw_offset);
	output.desired_yaw = matrix::wrap_pi(path.heading + output.desired_yaw_offset
					     - atanf(config.cross_track_gain * path.error));
	output.yaw_rate_feedforward = longitudinal_speed * path.curvature / radius_scale;

	// Negative heading/cross-track feedback with a smooth steering-angle saturation.
	const float feedback_input = -config.heading_gain
				     * (output.heading_error + atanf(config.cross_track_gain * path.error));
	output.feedback_angle = 2.f * output.steering_limit / M_PI_F
				* atanf(M_PI_F * feedback_input / (2.f * output.steering_limit));
	output.steering_angle = math::constrain(output.feedforward_angle + output.feedback_angle,
					       -output.steering_limit, output.steering_limit);
	output.normalized_steering = math::constrain(output.steering_angle / config.max_steering_angle, -1.f, 1.f);
	output.path_feasible = fabsf(output.feedforward_angle) <= output.steering_limit + 1.e-4f;
	output.valid = true;
	return output;
}

} // namespace trolleytakeoff
