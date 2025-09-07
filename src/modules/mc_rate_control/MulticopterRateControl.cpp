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

#include "MulticopterRateControl.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/events.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

MulticopterRateControl::MulticopterRateControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_vehicle_torque_setpoint_pub(vtol ? ORB_ID(vehicle_torque_setpoint_virtual_mc) : ORB_ID(vehicle_torque_setpoint)),
	_vehicle_thrust_setpoint_pub(vtol ? ORB_ID(vehicle_thrust_setpoint_virtual_mc) : ORB_ID(vehicle_thrust_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	parameters_updated();
	_controller_status_pub.advertise();
}

MulticopterRateControl::~MulticopterRateControl()
{
	perf_free(_loop_perf);
}

bool
MulticopterRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	_rate_control_start_time = hrt_absolute_time();

	return true;
}

void
MulticopterRateControl::parameters_updated()
{
	// rate control parameters
	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	const Vector3f rate_k = Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());

	_rate_control.setPidGains(
		rate_k.emult(Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get())));

	_rate_control.setIntegratorLimit(
		Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get()));

	_rate_control.setFeedForwardGain(
		Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get()));


	// manual rate control acro mode rate limits
	_acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()), radians(_param_mc_acro_p_max.get()),
				  radians(_param_mc_acro_y_max.get()));
}

void MulticopterRateControl::l1_parameters_updated()
{
	_l1_adaptive_control.setTuneParameters(_param_mc_l1_as_v.get(),_param_mc_l1_as_omega.get());
	_l1_adaptive_control.setMassInertia(_param_mc_l1_mass.get(),_param_mc_l1_j_x.get(),_param_mc_l1_j_y.get(),_param_mc_l1_j_z.get());
	_l1_adaptive_control.setLowPassFilterParameters(_param_mc_l1_cofq1_t.get(),_param_mc_l1_cofq1_m.get(),_param_mc_l1_cofq2_m.get());
	_l1_adaptive_control.l1enable=_param_mc_l1_en.get();
	_l1_adaptive_control.l1_ctrl_on=_param_mc_l1_ctrl_on.get();
	_l1_use_gt_pos = _param_mc_l1_use_gt_pos.get();
	_l1_print = _param_mc_l1_print.get();
	_l1_torque_ratio_x = _param_mc_l1_tor_ratx.get();
	_l1_torque_ratio_y = _param_mc_l1_tor_raty.get();

}


void
MulticopterRateControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
		l1_parameters_updated();
	}

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		const hrt_abstime now = angular_velocity.timestamp_sample;

		// Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
		// ! L1 采样时间用这个dt？
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;

		const Vector3f rates{angular_velocity.xyz};
		const Vector3f angular_accel{angular_velocity.xyz_derivative};

		/* check for updates in other topics */
		_vehicle_control_mode_sub.update(&_vehicle_control_mode);

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
				_maybe_landed = vehicle_land_detected.maybe_landed;
			}
		}

		_vehicle_status_sub.update(&_vehicle_status);

		// use rates setpoint topic
		vehicle_rates_setpoint_s vehicle_rates_setpoint{};

		if (_vehicle_control_mode.flag_control_manual_enabled && !_vehicle_control_mode.flag_control_attitude_enabled) {
			// generate the rate setpoint from sticks
			manual_control_setpoint_s manual_control_setpoint;

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				// manual rates control - ACRO mode
				const Vector3f man_rate_sp{
					math::superexpo(manual_control_setpoint.roll, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(-manual_control_setpoint.pitch, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(manual_control_setpoint.yaw, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get())};

				_rates_setpoint = man_rate_sp.emult(_acro_rate_max);
				_thrust_setpoint(2) = -(manual_control_setpoint.throttle + 1.f) * .5f;
				_thrust_setpoint(0) = _thrust_setpoint(1) = 0.f;

				// publish rate setpoint
				vehicle_rates_setpoint.roll = _rates_setpoint(0);
				vehicle_rates_setpoint.pitch = _rates_setpoint(1);
				vehicle_rates_setpoint.yaw = _rates_setpoint(2);
				_thrust_setpoint.copyTo(vehicle_rates_setpoint.thrust_body);
				vehicle_rates_setpoint.timestamp = hrt_absolute_time();

				_vehicle_rates_setpoint_pub.publish(vehicle_rates_setpoint);
			}

		} else if (_vehicle_rates_setpoint_sub.update(&vehicle_rates_setpoint)) {
			if (_vehicle_rates_setpoint_sub.copy(&vehicle_rates_setpoint)) {
				_rates_setpoint(0) = PX4_ISFINITE(vehicle_rates_setpoint.roll)  ? vehicle_rates_setpoint.roll  : rates(0);
				_rates_setpoint(1) = PX4_ISFINITE(vehicle_rates_setpoint.pitch) ? vehicle_rates_setpoint.pitch : rates(1);
				_rates_setpoint(2) = PX4_ISFINITE(vehicle_rates_setpoint.yaw)   ? vehicle_rates_setpoint.yaw   : rates(2);
				_thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body);
			}
		}

		// run the rate controller
		if (_vehicle_control_mode.flag_control_rates_enabled) {

			// reset integral if disarmed
			if (!_vehicle_control_mode.flag_armed || _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				_rate_control.resetIntegral();
			}

			// update saturation status from control allocation feedback
			control_allocator_status_s control_allocator_status;

			if (_control_allocator_status_sub.update(&control_allocator_status)) {
				Vector<bool, 3> saturation_positive;
				Vector<bool, 3> saturation_negative;

				if (!control_allocator_status.torque_setpoint_achieved) {
					for (size_t i = 0; i < 3; i++) {
						if (control_allocator_status.unallocated_torque[i] > FLT_EPSILON) {
							saturation_positive(i) = true;

						} else if (control_allocator_status.unallocated_torque[i] < -FLT_EPSILON) {
							saturation_negative(i) = true;
						}
					}
				}

				// TODO: send the unallocated value directly for better anti-windup
				_rate_control.setSaturationStatus(saturation_positive, saturation_negative);
			}

			// run rate controller | for l1 , this is the base controller
			Vector3f att_control = _rate_control.update(rates, _rates_setpoint, angular_accel, dt, _maybe_landed || _landed);


			// **** L1 Adaptive Controller ****
			if(_last_l1enabled && !_l1_adaptive_control.l1enable)
			{
				// if l1 adaptive controller is disabled, we need to reset the controller
				_l1_adaptive_control._controller_init = false;
				PX4_INFO("L1 Adaptive Control: disabled!");
			}

			// don't run l1 adaptive controller at the first 5 seconds
			if(_l1_adaptive_control.l1enable && (hrt_absolute_time() - _rate_control_start_time) > 5 * 1e6)
			{

					// update sample time 
					_l1_adaptive_control.setSampleTime(dt);

					// bool should_turnoff_l1 = false;
					// bool should_turnoff_l1_ctrl = false;

					// update states including linear velocity and angular rates
					// vehicle_local_position_s vehicle_local_position;
					// if(_l1_use_gt_pos)
					// 	_local_pos_gt_sub.copy(&vehicle_local_position);  //! using groundtruth position and velocity no noise
					// else
					// 	_local_pos_sub.copy(&vehicle_local_position);

					// Vector3f velocities = Vector3f(vehicle_local_position.vx,vehicle_local_position.vy, vehicle_local_position.vz);

					// if(!PX4_ISFINITE(velocities(0)) || !PX4_ISFINITE(velocities(1)) || !PX4_ISFINITE(velocities(2))) // safety check
					// {
					// 	PX4_WARN("L1 Adaptive Control: velocity is INF! turn off L1 all");
					// 	// should_turnoff_l1 = true;
					// 	// should_turnoff_l1_ctrl = true;
					// 	_l1_adaptive_control.l1enable = false;
					// 	int32_t false_value = 0;
					// 	param_set(param_find("MC_L1_EN"), &false_value);
					// }

					vehicle_attitude_s vehicle_att;
					_vehicle_attitude_sub.copy(&vehicle_att);
					Quaternionf vehicle_att_q = Quaternionf(vehicle_att.q[0],vehicle_att.q[1],vehicle_att.q[2],vehicle_att.q[3]);

					// update base controller output
					Vector3f base_torque = att_control;

					// static bool hte_inited = false;
					float throttle2thrust_ratio = 1.0f;
					// hover_thrust_estimate_s hte;
					// if (_hover_thrust_estimate_sub.update(&hte)) {
					// 	if (hte.valid) {
					// 		throttle2thrust_ratio = _l1_adaptive_control.getThrottle2ThrustRatio(hte.hover_thrust);
					// 		hte_inited = true;
					// 	}
					// }
					// if (!hte_inited) {
					// 	if(_l1_adaptive_control.l1_ctrl_on)
					// 		PX4_WARN("L1 Adaptive Control: hover_thrust_estimate is not valid! Turn off L1 Control");
					// 	should_turnoff_l1_ctrl = true;
					// }

					// we only need the thrust norm
					//! WARNING : _thrust_setpoint.norm() this is not the exact thrust force ,it is throttle
					float thrust_norm = _thrust_setpoint.norm() * throttle2thrust_ratio; 	
					Vector4f u_b = Vector4f(thrust_norm,base_torque(0),base_torque(1),base_torque(2));
					Vector4f u_ad = Vector4f(0,0,0,0);

					// if(should_turnoff_l1_ctrl && _l1_adaptive_control.l1enable)
					// {
					// 	_l1_adaptive_control.l1_ctrl_on = false;
					// 	int32_t false_value = 0;
					// 	param_set(param_find("MC_L1_CTRL_ON"), &false_value);
					// }

					// if(should_turnoff_l1 && _l1_adaptive_control.l1_ctrl_on)
					// {
					// 	_l1_adaptive_control.l1enable = false;
					// 	int32_t false_value = 0;
					// 	param_set(param_find("MC_L1_EN"), &false_value);
					// }

					Vector3f velocities = Vector3f(0,0,0);
					if(_l1_adaptive_control.l1enable)
					{
						if(!_l1_adaptive_control._controller_init) // first time
						{
							_l1_adaptive_control.initialize(velocities,rates,vehicle_att_q,u_b);
							PX4_INFO("L1 Adaptive Control: controller initialized!");
						}
						else{
							_l1_adaptive_control.setState(velocities,rates);
							_l1_adaptive_control.setAttitude(vehicle_att_q);

							if(_l1_adaptive_control.update(u_b,u_ad))
							{
								if(!PX4_ISFINITE(u_ad(1)) || !PX4_ISFINITE(u_ad(2)) || !PX4_ISFINITE(u_ad(3)))
								{
									PX4_WARN("L1 Adaptive Control: u_ad is INF!");
								}
								else{
									// add u_ad(0) to _thrust_setpoint direction. Note that thrust transit to thrust force
									// ！ stvstv : don't use thrust compensation for now
									// _thrust_setpoint = _thrust_setpoint + _thrust_setpoint.normalized() * u_ad(0) / (throttle2thrust_ratio + FLT_EPSILON);
									// add u_ad(1 to 3) to att_control
									if(_l1_adaptive_control.l1_ctrl_on){
										att_control(0) += u_ad(1) * _l1_torque_ratio_x;
										att_control(1) += u_ad(2) * _l1_torque_ratio_y;
										att_control(2) += u_ad(3);
									}
								}

							}
							else{
								PX4_WARN("L1 Adaptive Control: controller update error!");
							}

							// for debug
							Vector3f v_hat = _l1_adaptive_control.getVhat();
							Vector3f omega_hat = _l1_adaptive_control.getOmegahat();
							Vector3f v_pred_error_now = _l1_adaptive_control.getVPredError();
							Vector3f omega_pred_error_now = _l1_adaptive_control.getOmegaPredError();
							Vector4f sigma_m_now = _l1_adaptive_control.getSigma_m_now();
							Vector2f sigma_um_now = _l1_adaptive_control.getSigma_um_now();

							static hrt_abstime last_print_time = 0;
							if(_l1_print)
							{
								if(hrt_absolute_time() - last_print_time > 1e6) // print per 1 seconds
								{
									PX4_INFO("v_hat: %f %f %f", (double)v_hat(0), (double)v_hat(1), (double)v_hat(2));
									PX4_INFO("omega_hat: %f %f %f", (double)omega_hat(0), (double)omega_hat(1), (double)omega_hat(2));
									PX4_INFO("v_pred_error_now: %f %f %f", (double)v_pred_error_now(0), (double)v_pred_error_now(1), (double)v_pred_error_now(2));
									PX4_INFO("omega_pred_error_now: %f %f %f", (double)omega_pred_error_now(0), (double)omega_pred_error_now(1), (double)omega_pred_error_now(2));
									PX4_INFO("sigma_m_now: %f %f %f %f", (double)sigma_m_now(0), (double)sigma_m_now(1), (double)sigma_m_now(2), (double)sigma_m_now(3));
									PX4_INFO("sigma_um_now: %f %f", (double)sigma_um_now(0), (double)sigma_um_now(1));
									PX4_INFO("u_ad: %f %f %f %f", (double)u_ad(0), (double)u_ad(1), (double)u_ad(2), (double)u_ad(3));
									PX4_INFO("u_b: %f %f %f %f", (double)u_b(0), (double)u_b(1), (double)u_b(2), (double)u_b(3));
									last_print_time = hrt_absolute_time();
								}

							}

							l1_adaptive_debug_s l1_adaptive_debug{};
							l1_adaptive_debug.timestamp = hrt_absolute_time();
							l1_adaptive_debug.v_pred_x = v_hat(0);
							l1_adaptive_debug.v_pred_y = v_hat(1);
							l1_adaptive_debug.v_pred_z = v_hat(2);
							l1_adaptive_debug.v_pred_error_x = v_pred_error_now(0);
							l1_adaptive_debug.v_pred_error_y = v_pred_error_now(1);
							l1_adaptive_debug.v_pred_error_z = v_pred_error_now(2);
							l1_adaptive_debug.omega_pred_error_x = omega_pred_error_now(0);
							l1_adaptive_debug.omega_pred_error_y = omega_pred_error_now(1);
							l1_adaptive_debug.omega_pred_error_z = omega_pred_error_now(2);
							l1_adaptive_debug.sigma_m_0 = sigma_m_now(0);
							l1_adaptive_debug.sigma_m_1 = sigma_m_now(1);
							l1_adaptive_debug.sigma_m_2 = sigma_m_now(2);
							l1_adaptive_debug.sigma_m_3 = sigma_m_now(3);
							l1_adaptive_debug.sigma_um_0 = sigma_um_now(0);
							l1_adaptive_debug.sigma_um_1 = sigma_um_now(1);
							l1_adaptive_debug.omega_pred_x = omega_hat(0);
							l1_adaptive_debug.omega_pred_y = omega_hat(1);
							l1_adaptive_debug.omega_pred_z = omega_hat(2);
							l1_adaptive_debug.u_ad_0 = u_ad(0);
							l1_adaptive_debug.u_ad_1 = u_ad(1);
							l1_adaptive_debug.u_ad_2 = u_ad(2);
							l1_adaptive_debug.u_ad_3 = u_ad(3);
							l1_adaptive_debug.u_b_0 = u_b(0);
							l1_adaptive_debug.u_b_1 = u_b(1);
							l1_adaptive_debug.u_b_2 = u_b(2);
							l1_adaptive_debug.u_b_3 = u_b(3);
							l1_adaptive_debug.dt = dt;
							l1_adaptive_debug.l1enable = _l1_adaptive_control.l1enable;
							l1_adaptive_debug.l1ctrl_enable = _l1_adaptive_control.l1_ctrl_on;
							_l1_adaptive_debug_pub.publish(l1_adaptive_debug);
						}
					}
			}

			_last_l1enabled = _l1_adaptive_control.l1enable;

			// ****    L1 Adaptive Control End    ****


			// publish rate controller status ｜ just rate integral value
			rate_ctrl_status_s rate_ctrl_status{};
			_rate_control.getRateControlStatus(rate_ctrl_status);
			rate_ctrl_status.timestamp = hrt_absolute_time();
			_controller_status_pub.publish(rate_ctrl_status);

			// publish thrust and torque setpoints
			vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
			vehicle_torque_setpoint_s vehicle_torque_setpoint{};

			_thrust_setpoint.copyTo(vehicle_thrust_setpoint.xyz);
			vehicle_torque_setpoint.xyz[0] = PX4_ISFINITE(att_control(0)) ? att_control(0) : 0.f;
			vehicle_torque_setpoint.xyz[1] = PX4_ISFINITE(att_control(1)) ? att_control(1) : 0.f;
			vehicle_torque_setpoint.xyz[2] = PX4_ISFINITE(att_control(2)) ? att_control(2) : 0.f;

			// scale setpoints by battery status if enabled
			if (_param_mc_bat_scale_en.get()) {
				if (_battery_status_sub.updated()) {
					battery_status_s battery_status;

					if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
						_battery_status_scale = battery_status.scale;
					}
				}

				if (_battery_status_scale > 0.f) {
					for (int i = 0; i < 3; i++) {
						vehicle_thrust_setpoint.xyz[i] = math::constrain(vehicle_thrust_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
						vehicle_torque_setpoint.xyz[i] = math::constrain(vehicle_torque_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
					}
				}
			}

			vehicle_thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
			_vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint);

			vehicle_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_torque_setpoint.timestamp = hrt_absolute_time();
			_vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);

			updateActuatorControlsStatus(vehicle_torque_setpoint, dt);

		}
	}

	perf_end(_loop_perf);
}

void MulticopterRateControl::updateActuatorControlsStatus(const vehicle_torque_setpoint_s &vehicle_torque_setpoint,
		float dt)
{
	for (int i = 0; i < 3; i++) {
		_control_energy[i] += vehicle_torque_setpoint.xyz[i] * vehicle_torque_setpoint.xyz[i] * dt;
	}

	_energy_integration_time += dt;

	if (_energy_integration_time > 500e-3f) {

		actuator_controls_status_s status;
		status.timestamp = vehicle_torque_setpoint.timestamp;

		for (int i = 0; i < 3; i++) {
			status.control_power[i] = _control_energy[i] / _energy_integration_time;
			_control_energy[i] = 0.f;
		}

		_actuator_controls_status_pub.publish(status);
		_energy_integration_time = 0.f;
	}
}

int MulticopterRateControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterRateControl *instance = new MulticopterRateControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MulticopterRateControl::custom_command(int argc, char *argv[])
{
	if (argc > 0 && strcmp(argv[0], "print_l1_param") == 0) {
		if (get_instance()) {
			return get_instance()->print_l1_param();
		} else {
			PX4_ERR("Module instance not running");
			return PX4_ERROR;
		}
	}
	return print_usage("unknown command");
}

int MulticopterRateControl::print_l1_param()
{
	// Print L1 adaptive controller parameters
	PX4_INFO("L1 Adaptive Controller Parameters:");
	PX4_INFO("L1 Enabled: %d", _l1_adaptive_control.l1enable);
	PX4_INFO("L1 Use ground truth locPos : %d", _l1_use_gt_pos);
	PX4_INFO("L1 Ctrl On : %d", _l1_adaptive_control.l1_ctrl_on);
	PX4_INFO("L1 Mass: %f", static_cast<double>(_l1_adaptive_control._m));
	PX4_INFO("L1 Mass Inverse : %f", static_cast<double>(_l1_adaptive_control._mInverse));
	PX4_INFO("L1 J_X: %f", static_cast<double>(_l1_adaptive_control._j(0,0)));
	PX4_INFO("L1 J_Y: %f", static_cast<double>(_l1_adaptive_control._j(1,1)));
	PX4_INFO("L1 J_Z: %f", static_cast<double>(_l1_adaptive_control._j(2,2)));
	PX4_INFO("L1 AS_V: %f", static_cast<double>(_l1_adaptive_control._As_v));
	PX4_INFO("L1 AS_OMEGA: %f", static_cast<double>(_l1_adaptive_control._As_omega));
	PX4_INFO("L1 COFQ1_T: %f", static_cast<double>(_l1_adaptive_control._lpf_cofq1_T));
	PX4_INFO("L1 COFQ1_M: %f", static_cast<double>(_l1_adaptive_control._lpf_cofq1_M));
	PX4_INFO("L1 COFQ2_M: %f", static_cast<double>(_l1_adaptive_control._lpf_cofq2_M));	

	PX4_INFO("L1 dt: %f", static_cast<double>(_l1_adaptive_control._dt));

	// omega hat
	Vector3f omega_hat = _l1_adaptive_control.getOmegahat();
	PX4_INFO("L1 omega_hat: %f %f %f", static_cast<double>(omega_hat(0)), static_cast<double>(omega_hat(1)), static_cast<double>(omega_hat(2)));

	

	return PX4_OK;
}



int MulticopterRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter rate controller. It takes rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has a PID loop for angular rate error.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("print_l1_param", "Print L1 adaptive controller parameters");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_rate_control_main(int argc, char *argv[])
{
	return MulticopterRateControl::main(argc, argv);
}
