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
PARAM_DEFINE_FLOAT(MC_L1_AS_V, 0.5f);

/**
 * As_omega
 *
 * For L1 adaptvie controller, user selected diagonal Hurwitz matrix diagonal element for angular velocity
 *
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_L1_AS_OMEGA, -10.0f);

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
PARAM_DEFINE_FLOAT(MC_L1_MASS, 1.5f);

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
PARAM_DEFINE_FLOAT(MC_L1_J_X, 0.00736f);

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
PARAM_DEFINE_FLOAT(MC_L1_J_Y, 0.00840f);

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
PARAM_DEFINE_FLOAT(MC_L1_J_Z, 0.01176f);



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
PARAM_DEFINE_FLOAT(MC_L1_COFQ1_T, 1.0f);


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
PARAM_DEFINE_FLOAT(MC_L1_COFQ1_M, 10.0f);

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
PARAM_DEFINE_FLOAT(MC_L1_COFQ2_M, 2.0f);


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
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_L1_TOR_RATX, 17.274f);

/**
 *
 * att_control to real torque ratio for y axis
 *
 * mc rate controller pid output to real torque ratio
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_L1_TOR_RATY, 23.791f);

/**
* adaptive controller type
*
* 0 for L1 adaptive controller; 1 for naive L1 adaptive controller
*
* @min 0
* @max 1
* @value 0 L1 Adaptive Control
* @value 1 Naive L1 Adaptive Control
* @reboot_required true
* @group Multicopter Rate Control
*/
PARAM_DEFINE_INT32(ADA_CONTROL_TYPE, 0);

/**
 *
 * kad for naive l1 adaptive controller
 *
 * l1 adaptive controller gain for naive l1 adaptive controller
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_L1_KAD, 1.0f);


/**
 *
 * emax for naive l1 adaptive controller
 *
 * l1 adaptive controller error emax for naive l1 adaptive controller
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_L1_EMAX, 1.0f);

/**
 * Enable L1 adaptive controller add yaw compensate
 *
 * enable L1 adaptive controller add yaw compensate
 *
 * @boolean
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(MC_L1_YAW_ON, 0);

// Ye: K adaptive controller

/**
 * Enable K adaptive controller
 *
 * enable K adaptive controller
 *
 * @boolean
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(MC_K_ADAPTIVE_ON, 0);

/**
 *
 * K limitation for roll
 *
 * max K for roll
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_K_MAX_ROLL, 2.7f);

/**
 *
 * K limitation for pitch
 *
 * max K for pitch
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_K_MAX_PITCH, 2.7f);

/**
 *
 * K limitation for yaw
 *
 * max K for yaw
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_K_MAX_YAW, 1.2f);