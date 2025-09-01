/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
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
 * @file L1AdaptiveControl.hpp
 *
 * L1 adaptive controller for multicopter
 */

 #pragma once

 #include <lib/mathlib/mathlib.h>
 #include <matrix/matrix/math.hpp>
 #include <uORB/topics/vehicle_angular_velocity.h>
 #include <uORB/topics/vehicle_attitude.h>
 #include <uORB/topics/vehicle_attitude_setpoint.h>
 #include <uORB/topics/vehicle_local_position.h>


#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <lib/systemlib/mavlink_log.h>
#include <px4_platform_common/time.h> 


using namespace matrix;


class L1AdaptiveControl
{
public:
	L1AdaptiveControl() = default;
	~L1AdaptiveControl() = default;

	bool l1enable = false;
	bool l1_ctrl_on = false;

	// set sample time
	void setSampleTime(float dt)
	{
		_dt = dt;
	}

	// set parameters for tune
	void setTuneParameters(float As_v, float As_omega)
	{
		_As_v = As_v;
		_As_omega = As_omega;
	}

	// set low pass filter parameters
	void setLowPassFilterParameters(float lpf_cofq1_T, float lpf_cofq1_M, float lpf_cofq2_M)
	{
		_lpf_cofq1_T = lpf_cofq1_T;
		_lpf_cofq1_M = lpf_cofq1_M;
		_lpf_cofq2_M = lpf_cofq2_M;
	}

	// set mass and inertia of uav
	void setMassInertia(float m , float j_x,float j_y,float j_z)
	{
		_m = m;
		_mInverse = 1.0f / m;
		_j.setIdentity();
		_j(0, 0) = j_x;
		_j(1, 1) = j_y;
		_j(2, 2) = j_z;
		_jInverse = _j.I();
	}

	// set state e.g. velocity and angular velocity
	void setState(const Vector3f &v, const Vector3f &omega)
	{
		_v_now = v;
		_omega_now = omega;
	}

	// input quaternion , transforom to Rotation matrix
	void setAttitude(const Quaternionf &q)
	{
		_R_now = Dcmf(q);
	}


	/// @brief 初始化控制器状态
	void initialize(const Vector3f &v_now, const Vector3f &omega_now, const Quaternionf &q_now, const Vector4f &u_b)
	{
		// initial predictor output to zeros
		_v_hat_prev = Vector3f(0,0,0);
		_omega_hat_prev = Vector3f(0,0,0);

		// last step state
		_v_prev = v_now;
		_omega_prev = omega_now;
		_R_prev = Dcmf(q_now);
		_u_b_prev = u_b;

		// initial adaptive control output 、 uncentainty and low pass filter
		_u_ad_prev = Vector4f(0,0,0,0);
		_sigma_m_hat_prev = Vector4f(0,0,0,0);
		_sigma_um_hat_prev = Vector2f(0,0);
		_lpf1_prev = Vector4f(0,0,0,0);

		_controller_init = true;
	}

	/// @brief main function of L1 adaptive controller
	/// @param u_b  baseline controller output as input , u_ad the L1 adaptive controller output
	/// @return true if u_ad is computed
	bool update(const Vector4f u_b,Vector4f &u_ad_out)
	{
		// // PX4_INFO per 1 second
		// static uint32_t info_counter = 0; // we check the first 10 loops
		// if (info_counter++ < 10) {
		// 	PX4_INFO("v_hat_prev: %f %f %f", static_cast<double>(_v_hat_prev(0)), static_cast<double>(_v_hat_prev(1)), static_cast<double>(_v_hat_prev(2)));
		// }else
		// {
		// 	hrt_abstime current_time = hrt_absolute_time();
		// 	static hrt_abstime last_info_time;
		// 	if (current_time - last_info_time > 1e6) {
		// 		PX4_INFO("v_hat_prev: %f %f %f", static_cast<double>(_v_hat_prev(0)), static_cast<double>(_v_hat_prev(1)), static_cast<double>(_v_hat_prev(2)));
		// 		last_info_time = current_time;
		// 	}
		// }

		// check if initialized
		if(!_controller_init || _dt < 1e-5f) {
			return false;
		}

		Vector3f v_hat, omega_hat; // state predictor value for translational speed and rotational speed

		// update state predictor of this step
		v_hat = _v_hat_prev +
			(e3 * GRAVITY_MAGNITUDE
			- _R_prev.col(2) * (_u_b_prev(0) + _u_ad_prev(0) + _sigma_m_hat_prev(0)) * _mInverse
			+ _R_prev.col(0) * _sigma_um_hat_prev(0) * _mInverse
			+ _R_prev.col(1) * _sigma_um_hat_prev(1) * _mInverse
			+ _v_pred_error_prev * _As_v) * _dt;


		Vector3f tempVec = {_u_b_prev(1) + _u_ad_prev(1) + _sigma_m_hat_prev(1),
			_u_b_prev(2) + _u_ad_prev(2) + _sigma_m_hat_prev(2),
			_u_b_prev(3) + _u_ad_prev(3) + _sigma_m_hat_prev(3)};

		omega_hat = _omega_hat_prev
		+ (-_jInverse * (_omega_prev % (_j * _omega_prev))
		+ _jInverse * tempVec + _omega_pred_error_prev * _As_omega) * _dt;

		// update predictor output storage
		_v_hat_prev = v_hat;
		_omega_hat_prev = omega_hat;

		// compute prediction error for this step
		_v_pred_error_now = v_hat - _v_now;
		_omega_pred_error_now = omega_hat - _omega_now;

		// exponential coefficients for As
		float exp_As_v_dt = expf(_As_v * _dt);
		float exp_As_omega_dt = expf(_As_omega * _dt);

		// compute uncertainty h(t) piece constant
		Vector3f PhiInvmu_v = _v_pred_error_now / (exp_As_v_dt - 1) * _As_v * exp_As_v_dt;
		Vector3f PhiInvmu_omega = _omega_pred_error_now / (exp_As_omega_dt - 1) * _As_omega * exp_As_omega_dt;

		// obtain the matched and unmatched uncertainty
		_sigma_m_hat_now(0) = Vector3f(_R_now.col(2)).dot(PhiInvmu_v) * _m;
		_sigma_m_hat_now_2to4 = -_j * PhiInvmu_omega;
		_sigma_m_hat_now(1) = _sigma_m_hat_now_2to4(0);
		_sigma_m_hat_now(2) = _sigma_m_hat_now_2to4(1);
		_sigma_m_hat_now(3) = _sigma_m_hat_now_2to4(2);

		_sigma_um_hat_now(0) = - Vector3f(_R_now.col(0)).dot(PhiInvmu_v) * _m;
		_sigma_um_hat_now(1) = - Vector3f(_R_now.col(1)).dot(PhiInvmu_v) * _m;

		// store uncertainty estimations
		_sigma_m_hat_prev = _sigma_m_hat_now;
		_sigma_um_hat_prev = _sigma_um_hat_now;

		if(l1_ctrl_on)
		{
			// compute lpf1 coefficients
			float lpf1_coefficientThrust1 = expf(-_lpf_cofq1_T * _dt);
			float lpf1_coefficientThrust2 = 1.0f - lpf1_coefficientThrust1;

			float lpf1_coefficientMoment1 = expf(-_lpf_cofq1_M * _dt);
			float lpf1_coefficientMoment2 = 1.0f - lpf1_coefficientMoment1;

			// update the adaptive control
			Vector4f u_ad_int;
			Vector4f u_ad;

			// low-pass filter 1 (negation is added to u_ad_prev to filter the correct signal)
			u_ad_int(0) = lpf1_coefficientThrust1 * (_lpf1_prev(0)) + lpf1_coefficientThrust2 * _sigma_m_hat_now(0);
			u_ad_int(1) = lpf1_coefficientMoment1 * (_lpf1_prev(1)) + lpf1_coefficientMoment2 * _sigma_m_hat_now(1);
			u_ad_int(2) = lpf1_coefficientMoment1 * (_lpf1_prev(2)) + lpf1_coefficientMoment2 * _sigma_m_hat_now(2);
			u_ad_int(3) = lpf1_coefficientMoment1 * (_lpf1_prev(3)) + lpf1_coefficientMoment2 * _sigma_m_hat_now(3);

			_lpf1_prev = u_ad_int; // store the current state

			// negate
			u_ad_out = -u_ad_int ;
		}
		else
		{
			_lpf1_prev = Vector4f{0, 0, 0, 0};
			u_ad_out = Vector4f{0, 0, 0, 0};
		}

		// store the current state
		_v_pred_error_prev = _v_pred_error_now;
		_omega_pred_error_prev = _omega_pred_error_now;
		_u_ad_prev = u_ad_out;
		_v_prev = _v_now;
		_omega_prev = _omega_now;
		_R_prev = _R_now;
		_u_b_prev = u_b;

		return true;
	}

	bool _controller_init = false;

	Vector3f getVhat()
	{
		return _v_hat_prev;
	}
	Vector3f getOmegahat()
	{
		return _omega_hat_prev;
	}

	Vector3f getVPredError()
	{
	    return _v_pred_error_now;
	}
	Vector3f getOmegaPredError()
	{
	    return _omega_pred_error_now;
	}

	Vector4f getSigma_m_now()
	{
	    return _sigma_m_hat_now;
	}

	Vector2f getSigma_um_now()
	{
	    return _sigma_um_hat_now;
	}

	// calculate the throttle2thrust ratio
	float getThrottle2ThrustRatio(float hover_throttle)
	{
		return _m * GRAVITY_MAGNITUDE / hover_throttle;
	}


	// parameters for tune
	float _As_v;
	float _As_omega;
	float _dt; //sample duration

	// parameters for low pass filter
	float _lpf_cofq1_T;
	float _lpf_cofq1_M, _lpf_cofq2_M;

	// uav parameter
	float _m, _mInverse; // mass and its inverse
	Matrix3f _j,_jInverse; // inertia matrix and its inverse
	const float GRAVITY_MAGNITUDE = 9.8; // gravitational acceleration
	


	
	private:

	// state
	// now
	Vector3f _v_now; // translational speed
	Vector3f _omega_now; // rotational speed
	// last step
	Vector3f _v_prev; // translational speed
	Vector3f _omega_prev;

	// other state
	Matrix3f _R_prev, _R_now;

	// state predictor
	// now
	Vector3f _v_hat;          // state predictor value of translational speed
	Vector3f _omega_hat;      // state predictor value of rotational speed
	Vector3f e3 = {0, 0, 1}; // unit vector

	// last step
	Vector3f _v_hat_prev;
	Vector3f _omega_hat_prev;

	///this step predict error
	Vector3f _v_pred_error_now;
	Vector3f _omega_pred_error_now;

	/// previous step predict error
	Vector3f _v_pred_error_prev;
	Vector3f _omega_pred_error_prev;


	// base control input
	Vector4f _u_b_prev; // storage of previous step u_baseline
	// adaptive control input
	Vector4f _u_ad_prev; // previously stored adaptive control

	// matched uncertainty ( body_z force , torque in body_x , body_y , body_z )
	Vector4f _sigma_m_hat_prev;
	// unmatched uncertainty ( body_y force , body_z force)
	Vector2f _sigma_um_hat_prev;

	Vector4f _sigma_m_hat_now; 		// estimated matched uncertainty
	Vector3f _sigma_m_hat_now_2to4;     // second to fourth element of the estimated matched uncertainty ( uncentained torque )
	Vector2f _sigma_um_hat_now;         // estimated unmatched uncertainty


	// lpf state
	Vector4f _lpf1_prev; // storage of previous step u_adaptive



}; // L1AdaptiveControl

