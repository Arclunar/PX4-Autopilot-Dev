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

/**
 * @file mc_l1_params.c
 *
 * Parameters for multicopter l1 adaptive controller
 */

/**
 * Enable L1 adaptive controller for mc
 *
 * enable L1 adaptive controller
 *
 * @boolean
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(MC_L1_EN, 0);

/**
 * Enable L1 adaptive controller join control for mc
 *
 * enable L1 adaptive controller add compensate to control output
 *
 * @boolean
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(MC_L1_CTRL_ON, 0);

/**
 * Set L1 adaptive use ground truth local position in sitl
 *
 * Set L1 adaptive use ground truth local position in sitl : /vehicle_local_position_groudtruth
 *
 * @boolean
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(MC_L1_USE_GT_POS, 0);

/**
 * As_v
 *
 * For L1 adaptvie controller, user selected diagonal Hurwitz matrix diagonal element for velocity
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_L1_AS_V, 0.0f);

/**
 * As_omega
 *
 * For L1 adaptvie controller, user selected diagonal Hurwitz matrix diagonal element for angular velocity
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_L1_AS_OMEGA, 0.0f);

/**
 * mass
 *
 * For L1 adaptvie controller, uav mass
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_L1_MASS, 0.0f);

/**
 *
 * inertia x element
 *
 * For L1 adaptvie controller, uav inertia x element
 *
 * @min 0.0
 * @decimal 5
 * @increment 0.0001
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_L1_J_X, 0.0f);

/**
 *
 * inertia y element
 *
 * For L1 adaptvie controller, uav inertia y element
 *
 * @min 0.0
 * @decimal 5
 * @increment 0.0001
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_L1_J_Y, 0.0f);

/**
 *
 * inertia z element
 *
 * For L1 adaptvie controller, uav inertia z element
 *
 * @min 0.0
 * @decimal 5
 * @increment 0.0001
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_L1_J_Z, 0.0f);



/**
 *
 * Low pass filter 1 cutoff frequency for thrust
 *
 * For L1 adaptvie controller, the first low pass filter cutoff frequency for thrust
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_L1_COFQ1_T, 0.0f);


/**
 *
 * Low pass filter 1 cutoff frequency for moment
 *
 * For L1 adaptvie controller, the first low pass filter cutoff frequency for moment
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_L1_COFQ1_M, 0.0f);

/**
 *
 * Low pass filter 2 cutoff frequency for moment
 *
 * For L1 adaptvie controller, the second low pass filter cutoff frequency for moment
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_L1_COFQ2_M, 0.0f);


/**
 * Enable L1 adaptive controller print info for debug
 *
 * enable L1 adaptive controller print debug information
 *
 * @boolean
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(MC_L1_PRINT, 0);

/**
 *
 * att_control to real torque ratio for x axis
 *
 * mc rate controller pid output to real torque ratio
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_L1_TOR_RATX, 1.0f);

/**
 *
 * att_control to real torque ratio for y axis
 *
 * mc rate controller pid output to real torque ratio
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_L1_TOR_RATY, 1.0f);