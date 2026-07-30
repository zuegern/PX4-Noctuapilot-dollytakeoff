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

#pragma once

#include <math.h>
#include <matrix/math.hpp>

namespace trolleytakeoff
{

struct TrolleyPathState {
	matrix::Vector2f closest_point{NAN, NAN};
	matrix::Vector2f tangent{NAN, NAN};
	float heading{NAN};
	float error{NAN}; // positive to the right of the path in the local NED frame
	float curvature{NAN}; // positive for a right turn
	bool valid{false};
};

struct TrolleyControlConfig {
	float wheelbase{NAN};
	float reference_offset{NAN};
	float max_steering_angle{NAN};
	float heading_gain{NAN};
	float cross_track_gain{NAN};
	float max_lateral_acceleration{NAN};
	// Longitudinal speed [m/s] at or below which the closed-loop steering feedback is faded to zero, and
	// the speed at which it reaches full authority. Leave non-finite / non-positive to disable the fade.
	float feedback_speed_zero{NAN};
	float feedback_speed_full{NAN};
};

struct TrolleyControlOutput {
	float normalized_steering{NAN};
	float steering_angle{NAN};
	float feedforward_angle{NAN};
	float feedback_angle{NAN};
	float steering_limit{NAN};
	float heading_error{NAN};
	float desired_yaw_offset{NAN};
	float desired_yaw{NAN};
	float yaw_rate_feedforward{NAN};
	float minimum_cross_track_gain{NAN};
	float feedback_speed_scale{NAN}; // [0,1] speed-dependent scaling applied to the closed-loop feedback
	bool path_feasible{false};
	bool stability_condition_met{false};
	bool valid{false};
};

TrolleyPathState calculateStraightPathState(const matrix::Vector2f &start, float bearing,
		const matrix::Vector2f &position);

TrolleyPathState calculateCircularPathState(const matrix::Vector2f &start, float bearing, float radius,
		bool right_turn, const matrix::Vector2f &position);

TrolleyControlOutput calculateTrolleyControl(const TrolleyPathState &path, float yaw, float longitudinal_speed,
		const TrolleyControlConfig &config);

} // namespace trolleytakeoff
