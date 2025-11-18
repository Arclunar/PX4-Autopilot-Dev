/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
#include "disturbance_generator.hpp"

#include <lib/rate_control/rate_control.hpp>
#include <lib/rate_ndob/rate_ndob.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/systemlib/mavlink_log.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls_status.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/rate_ndob_outputs.h>

// Other topics for L1 adaptive
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/l1_adaptive_debug.h>
#include "L1AdaptiveControl.hpp"
#include <uORB/topics/debug_vect.h> // Ye


using namespace time_literals;

class MulticopterRateControl : public ModuleBase<MulticopterRateControl>, public ModuleParams, public px4::WorkItem
{
public:
	MulticopterRateControl(bool vtol = false);
	~MulticopterRateControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);


	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	int print_l1_param();

	bool init();

private:
	void Run() override;

	/**
	 * initialize some vectors/matrices from parameters
	 */
	void parameters_updated();



	void updateActuatorControlsStatus(const vehicle_torque_setpoint_s &vehicle_torque_setpoint, float dt);

	RateControl _rate_control; ///< class for rate control calculations

	StepDistGenerator _dist;

	RateNDOB _rate_ndob;

	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription _control_allocator_status_sub{ORB_ID(control_allocator_status)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _adaptive_K_sub{ORB_ID(debug_vect)}; // Ye
	uORB::Subscription _esc_status_sub{ORB_ID(esc_status)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	uORB::Publication<actuator_controls_status_s>	_actuator_controls_status_pub{ORB_ID(actuator_controls_status_0)};
	uORB::PublicationMulti<rate_ctrl_status_s>	_controller_status_pub{ORB_ID(rate_ctrl_status)};
	uORB::Publication<vehicle_rates_setpoint_s>	_vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s>	_vehicle_torque_setpoint_pub;
	uORB::Publication<vehicle_thrust_setpoint_s>	_vehicle_thrust_setpoint_pub;
	uORB::Publication<rate_ndob_outputs_s>		_rate_ndob_outputs_pub{ORB_ID(rate_ndob_outputs)};

	// subscribe to local position and vehicle_attitude
	uORB::SubscriptionCallbackWorkItem _local_pos_sub{this, ORB_ID(vehicle_local_position)};	/**< vehicle local position */
	uORB::SubscriptionCallbackWorkItem _local_pos_gt_sub{this, ORB_ID(vehicle_local_position_groundtruth)};
	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};
	// subscribe to hover thrust estimate
	uORB::Subscription _hover_thrust_estimate_sub{ORB_ID(hover_thrust_estimate)};
	
	// publicate L1 debug msg
	uORB::Publication<l1_adaptive_debug_s> _l1_adaptive_debug_pub{ORB_ID(l1_adaptive_debug)};

	matrix::Vector3f adaptive_K{1.0f, 1.0f, 1.0f};
	matrix::Vector3f unlimit_adaptive_K{1.0f, 1.0f, 1.0f};


	vehicle_control_mode_s	_vehicle_control_mode{};
	vehicle_status_s	_vehicle_status{};

	bool _landed{true};
	bool _maybe_landed{true};
	bool _dist_trigger{false};

	hrt_abstime _last_run{0};

	perf_counter_t	_loop_perf;			/**< loop duration performance counter */

	// keep setpoint values between updates
	matrix::Vector3f _acro_rate_max;		/**< max attitude rates in acro mode */
	matrix::Vector3f _rates_setpoint{};

	float _battery_status_scale{0.0f};
	matrix::Vector3f _thrust_setpoint{};

	float _energy_integration_time{0.0f};
	float _control_energy[4] {};

	// L1 adaptive controller
	int32_t _adaptive_controller_type{0};
	L1AdaptiveControl _l1_adaptive_control;
	NaiveL1AdaptiveControl _naive_l1_adaptive_control;
	bool _last_l1enabled{false};
	bool _l1_yaw_on{false};

	// to turn on l1 adaptive controller after system has started for a while
	hrt_abstime _rate_control_start_time{0};
	
	// parameters update for l1
	void l1_parameters_updated();
	bool _l1_print{false};
	float _l1_torque_ratio_x{1.0f};
	float _l1_torque_ratio_y{1.0f};

	// k adaptive parameters update
	void k_adaptive_parameters_updated();
	bool _k_adaptive_on{false};
	float _k_max_roll{1.0f};
	float _k_max_pitch{1.0f};
	float _k_max_yaw{1.0f};


	bool _l1_use_gt_pos{false};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MC_ROLLRATE_P>) _param_mc_rollrate_p,
		(ParamFloat<px4::params::MC_ROLLRATE_I>) _param_mc_rollrate_i,
		(ParamFloat<px4::params::MC_RR_INT_LIM>) _param_mc_rr_int_lim,
		(ParamFloat<px4::params::MC_ROLLRATE_D>) _param_mc_rollrate_d,
		(ParamFloat<px4::params::MC_ROLLRATE_FF>) _param_mc_rollrate_ff,
		(ParamFloat<px4::params::MC_ROLLRATE_K>) _param_mc_rollrate_k,

		(ParamFloat<px4::params::MC_PITCHRATE_P>) _param_mc_pitchrate_p,
		(ParamFloat<px4::params::MC_PITCHRATE_I>) _param_mc_pitchrate_i,
		(ParamFloat<px4::params::MC_PR_INT_LIM>) _param_mc_pr_int_lim,
		(ParamFloat<px4::params::MC_PITCHRATE_D>) _param_mc_pitchrate_d,
		(ParamFloat<px4::params::MC_PITCHRATE_FF>) _param_mc_pitchrate_ff,
		(ParamFloat<px4::params::MC_PITCHRATE_K>) _param_mc_pitchrate_k,

		(ParamFloat<px4::params::MC_YAWRATE_P>) _param_mc_yawrate_p,
		(ParamFloat<px4::params::MC_YAWRATE_I>) _param_mc_yawrate_i,
		(ParamFloat<px4::params::MC_YR_INT_LIM>) _param_mc_yr_int_lim,
		(ParamFloat<px4::params::MC_YAWRATE_D>) _param_mc_yawrate_d,
		(ParamFloat<px4::params::MC_YAWRATE_FF>) _param_mc_yawrate_ff,
		(ParamFloat<px4::params::MC_YAWRATE_K>) _param_mc_yawrate_k,

		(ParamFloat<px4::params::MC_ACRO_R_MAX>) _param_mc_acro_r_max,
		(ParamFloat<px4::params::MC_ACRO_P_MAX>) _param_mc_acro_p_max,
		(ParamFloat<px4::params::MC_ACRO_Y_MAX>) _param_mc_acro_y_max,
		(ParamFloat<px4::params::MC_ACRO_EXPO>) _param_mc_acro_expo,			/**< expo stick curve shape (roll & pitch) */
		(ParamFloat<px4::params::MC_ACRO_EXPO_Y>) _param_mc_acro_expo_y,				/**< expo stick curve shape (yaw) */
		(ParamFloat<px4::params::MC_ACRO_SUPEXPO>) _param_mc_acro_supexpo,		/**< superexpo stick curve shape (roll & pitch) */
		(ParamFloat<px4::params::MC_ACRO_SUPEXPOY>) _param_mc_acro_supexpoy,		/**< superexpo stick curve shape (yaw) */

		// L1 parameters
		(ParamFloat<px4::params::MC_L1_MASS>) _param_mc_l1_mass,
		(ParamFloat<px4::params::MC_L1_J_X>) _param_mc_l1_j_x,
		(ParamFloat<px4::params::MC_L1_J_Y>) _param_mc_l1_j_y,
		(ParamFloat<px4::params::MC_L1_J_Z>) _param_mc_l1_j_z,
		(ParamFloat<px4::params::MC_L1_AS_V>) _param_mc_l1_as_v,
		(ParamFloat<px4::params::MC_L1_AS_OMEGA>) _param_mc_l1_as_omega,
		(ParamFloat<px4::params::MC_L1_COFQ1_T>) _param_mc_l1_cofq1_t,
		(ParamFloat<px4::params::MC_L1_COFQ1_M>) _param_mc_l1_cofq1_m,
		(ParamFloat<px4::params::MC_L1_COFQ2_M>) _param_mc_l1_cofq2_m,
		(ParamFloat<px4::params::MC_L1_TOR_RATX>) _param_mc_l1_tor_ratx,
		(ParamFloat<px4::params::MC_L1_TOR_RATY>) _param_mc_l1_tor_raty,

		(ParamFloat<px4::params::MC_DOB_CUTOFF>) _param_mc_dob_cutoff,
		(ParamFloat<px4::params::MC_DOB_K>) _param_mc_dob_k,
		(ParamFloat<px4::params::MC_DIST_F>) _param_mc_dist_f,
		(ParamFloat<px4::params::MC_DIST_T>) _param_mc_dist_t,
		(ParamFloat<px4::params::MC_DIST_MAG>) _param_mc_dist_mag,

		(ParamBool<px4::params::MC_BAT_SCALE_EN>) _param_mc_bat_scale_en,
		(ParamInt<px4::params::MC_DIST_EN>) _param_mc_dist_en,
		(ParamInt<px4::params::MC_DOB_EN>) _param_mc_dob_en,
		// L1 parameters
		(ParamBool<px4::params::MC_L1_EN>) _param_mc_l1_en,
		(ParamBool<px4::params::MC_L1_CTRL_ON>) _param_mc_l1_ctrl_on,
		(ParamBool<px4::params::MC_L1_USE_GT_POS>) _param_mc_l1_use_gt_pos,
		(ParamBool<px4::params::MC_L1_PRINT>) _param_mc_l1_print,

		(ParamInt<px4::params::ADA_CONTROL_TYPE>) _param_ada_control_type,
		(ParamFloat<px4::params::MC_L1_EMAX>) _param_mc_l1_emax,
		(ParamFloat<px4::params::MC_L1_KAD>) _param_mc_l1_kad,
		(ParamInt<px4::params::MC_L1_YAW_ON>) _param_mc_l1_yaw_on,

		// Ye: K adaptive parameters
		(ParamBool<px4::params::MC_K_ADAPTIVE_ON>) _param_mc_k_adaptive_on,
		(ParamFloat<px4::params::MC_K_MAX_ROLL>) _param_mc_k_max_roll,
		(ParamFloat<px4::params::MC_K_MAX_PITCH>) _param_mc_k_max_pitch,
		(ParamFloat<px4::params::MC_K_MAX_YAW>) _param_mc_k_max_yaw

	)
};
