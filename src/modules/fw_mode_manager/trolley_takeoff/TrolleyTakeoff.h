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
 * @file TrolleyTakeoff.h
 * Trolley-assisted takeoff handling for fixed-wing UAVs.
 */

#ifndef TROLLEYTAKEOFF_H
#define TROLLEYTAKEOFF_H

#include "TrolleyControl.h"

#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <px4_platform_common/module_params.h>

namespace trolleytakeoff
{

enum TrolleyTakeoffState {
	ALIGN_TO_PATH = 0, // taxi at low throttle and steer until the trolley points along the path
	THROTTLE_RAMP, // ramping up throttle while attached to the trolley
	CLAMPED_TO_TROLLEY, // constrained on trolley, steering with wheel control
	CLIMBOUT, // after release/liftoff, climb to clearance altitude
	FLYING, // navigate freely
	ABORTED // trolley connection was lost before release was authorized
};

enum TrolleyPathType {
	PATH_STRAIGHT = 0,
	PATH_CONSTANT_RIGHT = 1,
	PATH_CONSTANT_LEFT = 2
};

enum TrolleyTakeoffCondition {
	CONDITION_AIRSPEED = 0,
	CONDITION_GROUNDSPEED = 1,
	CONDITION_TIME = 2
};

enum TrolleySteeringMode {
	STEERING_HEADING = 0,
	STEERING_PATH_TRACKING = 1,
	STEERING_OPEN_LOOP = 2,
	STEERING_PATH_HEADING = 3
};

class __EXPORT TrolleyTakeoff : public ModuleParams
{
public:
	TrolleyTakeoff(ModuleParams *parent) : ModuleParams(parent) {}
	~TrolleyTakeoff() = default;

	void init(const hrt_abstime &time_now, const bool trolley_link_required, const bool trolley_link_healthy);

	void update(const hrt_abstime &time_now, const float takeoff_airspeed, const float calibrated_airspeed,
		    const float estimated_ground_speed, const float vehicle_altitude, const float clearance_altitude,
		    const float path_heading_error, const bool trolley_link_required, const bool trolley_link_healthy);

	TrolleyTakeoffState getState() const { return takeoff_state_; }

	bool isInitialized() const { return initialized_; }

	bool isAligningToPath() const { return takeoff_state_ == TrolleyTakeoffState::ALIGN_TO_PATH; }

	hrt_abstime initializedTime() const { return time_initialized_; }

	bool wheelNudgingEnabled() const { return param_trolley_nudge_.get() && takeoff_state_ < TrolleyTakeoffState::CLIMBOUT; }

	bool linkHealthy() const { return trolley_link_healthy_; }

	bool isReleased() const { return release_authorized_; }

	bool isAborted() const { return takeoff_state_ == TrolleyTakeoffState::ABORTED; }

	bool badDisconnectDetected() const { return bad_disconnect_detected_; }

	bool goodDisconnectDetected() const { return good_disconnect_detected_; }

	void abort();

	bool wheelSteeringEnabled() const;

	bool directWheelSteeringEnabled() const;

	bool headingSteeringEnabled() const;

	bool pathTrackingSteeringEnabled() const;

	bool openLoopSteeringEnabled() const;

	bool pathHeadingSteeringEnabled() const;

	bool wheelControllerSteeringEnabled() const;

	bool navigationSteeringEnabled() const;

	bool wheelControllerConfigured() const { return param_fw_w_en_.get(); }

	float wheelSteeringSetpoint() const { return wheel_steering_setpoint_; }

	void updateWheelSteeringSetpoint(const hrt_abstime &time_now, const float target_setpoint);

	float openLoopWheelSteeringSetpoint(const float longitudinal_speed) const;

	TrolleyControlOutput pathTrackingWheelSteeringSetpoint(const TrolleyPathState &path_state, const float yaw,
			const float longitudinal_speed) const;

	float maximumPathError() const { return param_trolley_xtk_max_.get(); }

	float maximumWheelYawRate() const { return math::radians(param_fw_w_rmax_.get()); }

	int32_t steeringMode() const { return param_trolley_str_mode_.get(); }

	int32_t pathType() const { return param_trolley_path_.get(); }

	float pathRadius() const { return param_trolley_radius_.get(); }

	// Minimum possible radius of the PX4 reference point on the trolley centerline.
	float minimumTurnRadius() const;

	// Radius used by path tracking for the PX4 reference point on the trolley centerline.
	float effectivePathRadius() const;

	// Converts reference-point path radius to rear-axle centerline radius for bicycle steering geometry.
	float rearAxleTurnRadius(const float reference_radius) const;

	TrolleyControlConfig controlConfig() const;

	float getPitch() const;

	float getThrottle(const float idle_throttle) const;

	float getMinPitch(const float min_pitch_in_climbout, const float min_pitch) const;

	float getMaxPitch(const float max_pitch) const;

	// NOTE: this is only to be used for mistaken mode transitions to takeoff while already in air.
	void forceSetFlyState()
	{
		takeoff_state_ = TrolleyTakeoffState::FLYING;
		release_authorized_ = true;
	}

	void reset();

	float interpolateValuesOverAbsoluteTime(const float start_value, const float end_value, const hrt_abstime &start_time,
						const float interpolation_time) const;

private:
	void updateLinkState(const bool trolley_link_required, const bool trolley_link_healthy);

	float rotationAirspeedThreshold(const float takeoff_airspeed) const;

	float rotationGroundspeedThreshold(const float takeoff_airspeed) const;

	bool releaseConditionReached(const hrt_abstime &time_now, const float takeoff_airspeed, const float calibrated_airspeed,
				     const float estimated_ground_speed) const;

	TrolleyTakeoffState takeoff_state_{THROTTLE_RAMP};

	bool initialized_{false};

	bool trolley_link_healthy_{true};

	bool release_authorized_{false};

	bool bad_disconnect_detected_{false};

	bool good_disconnect_detected_{false};

	float wheel_steering_setpoint_{0.f};

	hrt_abstime time_initialized_{0};

	hrt_abstime takeoff_time_{0};

	hrt_abstime ramp_start_time_{0};

	hrt_abstime time_last_steering_update_{0};

	hrt_abstime release_condition_met_since_{0};

	hrt_abstime align_condition_met_since_{0};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::TROLLEY_ALN_THR>) param_trolley_aln_thr_,
		(ParamFloat<px4::params::TROLLEY_ALN_ERR>) param_trolley_aln_err_,
		(ParamFloat<px4::params::TROLLEY_ALN_TO>) param_trolley_aln_to_,
		(ParamFloat<px4::params::TROLLEY_MAX_THR>) param_trolley_max_thr_,
		(ParamFloat<px4::params::TROLLEY_PSP>) param_trolley_psp_,
		(ParamFloat<px4::params::TROLLEY_RAMP_TM>) param_trolley_ramp_time_,
		(ParamBool<px4::params::TROLLEY_NUDGE>) param_trolley_nudge_,
		(ParamInt<px4::params::TROLLEY_TK_COND>) param_trolley_tk_cond_,
		(ParamFloat<px4::params::TROLLEY_ROT_ASPD>) param_trolley_rot_airspd_,
		(ParamFloat<px4::params::TROLLEY_ROT_GSPD>) param_trolley_rot_gspd_,
		(ParamFloat<px4::params::TROLLEY_RLS_HOLD>) param_trolley_rls_hold_,
		(ParamFloat<px4::params::TROLLEY_TK_TIME>) param_trolley_tk_time_,
		(ParamFloat<px4::params::TROLLEY_ROT_TIME>) param_trolley_rot_time_,
		(ParamInt<px4::params::TROLLEY_STR_MODE>) param_trolley_str_mode_,
		(ParamInt<px4::params::TROLLEY_PATH>) param_trolley_path_,
		(ParamFloat<px4::params::TROLLEY_RADIUS>) param_trolley_radius_,
		(ParamFloat<px4::params::TROLLEY_WB>) param_trolley_wheelbase_,
		(ParamFloat<px4::params::TROLLEY_REF_X>) param_trolley_ref_x_,
		(ParamFloat<px4::params::TROLLEY_STR_MAX>) param_trolley_str_max_,
		(ParamFloat<px4::params::TROLLEY_TRK_GAIN>) param_trolley_trk_gain_,
		(ParamFloat<px4::params::TROLLEY_XTK_GAIN>) param_trolley_xtk_gain_,
		(ParamFloat<px4::params::TROLLEY_LAT_ACC>) param_trolley_lat_acc_,
		(ParamFloat<px4::params::TROLLEY_XTK_MAX>) param_trolley_xtk_max_,
		(ParamFloat<px4::params::TROLLEY_STR_RATE>) param_trolley_str_rate_,
		(ParamFloat<px4::params::TROLLEY_STR_HOLD>) param_trolley_str_hold_,
		(ParamBool<px4::params::FW_W_EN>) param_fw_w_en_,
		(ParamFloat<px4::params::FW_W_RMAX>) param_fw_w_rmax_
	)
};

} // namespace trolleytakeoff

#endif // TROLLEYTAKEOFF_H
