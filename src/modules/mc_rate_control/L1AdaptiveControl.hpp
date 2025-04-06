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


class L1AdaptiveControl
{
public:
	L1AdaptiveControl() = default;
	~L1AdaptiveControl() = default;

	// set sample time
	void setSampleTime(double dt)
	{
		_dt = dt;
	}

	// set parameters for tune
	void setParameters(double As_v, double As_omega)
	{
		_As_v = As_v;
		_As_omega = As_omega;
	}

	// set state e.g. velocity and angular velocity
	void setState(const matrix::Vector3f &v, const matrix::Vector3f &omega)
	{
		_v = v;
		_omega = omega;
	}


private:

	// parameters for tune
	double _As_v;
	double _As_omega;
	double _dt; //sample duration

	// state
	matrix::Vector3f _v; // translational speed
	matrix::Vector3f _omega; // rotational speed



	// state predictor
	matrix::Vector3f _v_hat;          // state predictor value of translational speed
	matrix::Vector3f _omega_hat;      // state predictor value of rotational speed
	matrix::Vector3f e3 = {0, 0, 1}; // unit vector

} // L1AdaptiveControl
