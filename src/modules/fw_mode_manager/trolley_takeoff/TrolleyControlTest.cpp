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

#include <gtest/gtest.h>

#include <math.h>
#include <mathlib/mathlib.h>

#include "TrolleyControl.h"

using matrix::Vector2f;
using namespace trolleytakeoff;

static TrolleyControlConfig defaultConfig()
{
	return {
		.wheelbase = 0.94f,
		.reference_offset = 0.4f,
		.max_steering_angle = math::radians(20.f),
		.heading_gain = 1.f,
		.cross_track_gain = 0.5f,
		.max_lateral_acceleration = -1.f,
	};
}

TEST(TrolleyControl, StraightPathGeometryAndCorrection)
{
	const TrolleyPathState centered = calculateStraightPathState(Vector2f{0.f, 0.f}, 0.f, Vector2f{5.f, 0.f});
	ASSERT_TRUE(centered.valid);
	EXPECT_NEAR(centered.error, 0.f, 1.e-6f);
	EXPECT_NEAR(centered.heading, 0.f, 1.e-6f);

	const TrolleyControlOutput centered_output = calculateTrolleyControl(centered, 0.f, 5.f, defaultConfig());
	ASSERT_TRUE(centered_output.valid);
	EXPECT_TRUE(centered_output.stability_condition_met);
	EXPECT_NEAR(centered_output.normalized_steering, 0.f, 1.e-6f);

	const TrolleyPathState right_of_path =
		calculateStraightPathState(Vector2f{0.f, 0.f}, 0.f, Vector2f{5.f, 1.f});
	ASSERT_TRUE(right_of_path.valid);
	EXPECT_GT(right_of_path.error, 0.f);
	EXPECT_LT(calculateTrolleyControl(right_of_path, 0.f, 5.f, defaultConfig()).normalized_steering, 0.f);

	EXPECT_LT(calculateTrolleyControl(centered, 0.2f, 5.f, defaultConfig()).normalized_steering, 0.f);
}

TEST(TrolleyControl, CircularPathUsesReferenceOffset)
{
	const float radius = 10.f;
	const TrolleyPathState right_curve =
		calculateCircularPathState(Vector2f{0.f, 0.f}, 0.f, radius, true, Vector2f{0.f, 0.f});
	ASSERT_TRUE(right_curve.valid);
	EXPECT_NEAR(right_curve.heading, 0.f, 1.e-6f);
	EXPECT_NEAR(right_curve.error, 0.f, 1.e-6f);
	EXPECT_NEAR(right_curve.curvature, 1.f / radius, 1.e-6f);

	const TrolleyControlConfig config = defaultConfig();
	const float desired_yaw = -asinf(config.reference_offset * right_curve.curvature);
	const TrolleyControlOutput output = calculateTrolleyControl(right_curve, desired_yaw, 5.f, config);
	ASSERT_TRUE(output.valid);
	EXPECT_TRUE(output.path_feasible);
	EXPECT_NEAR(output.feedback_angle, 0.f, 1.e-6f);
	EXPECT_GT(output.feedforward_angle, 0.f);

	const float expected_feedforward =
		atanf(config.wheelbase * right_curve.curvature
		      / sqrtf(1.f - config.reference_offset * config.reference_offset
			      * right_curve.curvature * right_curve.curvature));
	EXPECT_NEAR(output.feedforward_angle, expected_feedforward, 1.e-6f);
}

TEST(TrolleyControl, CircularCrossTrackFeedbackHasCorrectSign)
{
	const TrolleyControlConfig config = defaultConfig();
	const TrolleyPathState inside_right_curve =
		calculateCircularPathState(Vector2f{0.f, 0.f}, 0.f, 10.f, true, Vector2f{0.f, 1.f});
	const TrolleyPathState outside_right_curve =
		calculateCircularPathState(Vector2f{0.f, 0.f}, 0.f, 10.f, true, Vector2f{0.f, -1.f});

	ASSERT_TRUE(inside_right_curve.valid);
	ASSERT_TRUE(outside_right_curve.valid);
	EXPECT_GT(inside_right_curve.error, 0.f);
	EXPECT_LT(outside_right_curve.error, 0.f);

	const float desired_yaw = -asinf(config.reference_offset * inside_right_curve.curvature);
	const TrolleyControlOutput centered =
		calculateTrolleyControl(
			calculateCircularPathState(Vector2f{0.f, 0.f}, 0.f, 10.f, true, Vector2f{0.f, 0.f}),
			desired_yaw, 5.f, config);
	const TrolleyControlOutput inside = calculateTrolleyControl(inside_right_curve, desired_yaw, 5.f, config);
	const TrolleyControlOutput outside = calculateTrolleyControl(outside_right_curve, desired_yaw, 5.f, config);

	ASSERT_TRUE(centered.valid);
	ASSERT_TRUE(inside.valid);
	ASSERT_TRUE(outside.valid);
	EXPECT_LT(inside.steering_angle, centered.steering_angle);
	EXPECT_GT(outside.steering_angle, centered.steering_angle);
}

TEST(TrolleyControl, LateralAccelerationLimitsSteering)
{
	TrolleyControlConfig config = defaultConfig();
	config.max_lateral_acceleration = 4.f;
	const TrolleyPathState right_curve =
		calculateCircularPathState(Vector2f{0.f, 0.f}, 0.f, 10.f, true, Vector2f{0.f, 0.f});
	const TrolleyControlOutput output = calculateTrolleyControl(right_curve, 0.f, 12.f, config);

	ASSERT_TRUE(output.valid);
	const float expected_limit = atanf(config.max_lateral_acceleration * config.wheelbase / (12.f * 12.f));
	EXPECT_NEAR(output.steering_limit, expected_limit, 1.e-6f);
	EXPECT_LE(fabsf(output.steering_angle), expected_limit + 1.e-6f);
	EXPECT_FALSE(output.path_feasible);
}

TEST(TrolleyControl, RejectsUndefinedCircleCenter)
{
	const TrolleyPathState state =
		calculateCircularPathState(Vector2f{0.f, 0.f}, 0.f, 10.f, true, Vector2f{0.f, 10.f});

	EXPECT_FALSE(state.valid);
}

TEST(TrolleyControl, ChecksSufficientStabilityCondition)
{
	const TrolleyPathState path = calculateStraightPathState(Vector2f{0.f, 0.f}, 0.f, Vector2f{0.f, 0.f});
	TrolleyControlConfig config = defaultConfig();
	const TrolleyControlOutput stable = calculateTrolleyControl(path, 0.f, 5.f, config);

	ASSERT_TRUE(stable.valid);
	EXPECT_GT(stable.minimum_cross_track_gain, 0.f);
	EXPECT_TRUE(stable.stability_condition_met);

	config.cross_track_gain = stable.minimum_cross_track_gain;
	EXPECT_FALSE(calculateTrolleyControl(path, 0.f, 5.f, config).stability_condition_met);

	config = defaultConfig();
	config.heading_gain = 0.f;
	EXPECT_FALSE(calculateTrolleyControl(path, 0.f, 5.f, config).stability_condition_met);

	config = defaultConfig();
	config.reference_offset = -0.1f;
	EXPECT_FALSE(calculateTrolleyControl(path, 0.f, 5.f, config).stability_condition_met);
}

TEST(TrolleyControl, RejectsImpossibleReferenceOffset)
{
	TrolleyControlConfig config = defaultConfig();
	config.reference_offset = 10.f;
	const TrolleyPathState curve =
		calculateCircularPathState(Vector2f{0.f, 0.f}, 0.f, 10.f, true, Vector2f{0.f, 0.f});

	EXPECT_FALSE(calculateTrolleyControl(curve, 0.f, 5.f, config).valid);
}
