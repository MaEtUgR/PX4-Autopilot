/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: @author Thomas Gubler <thomasgubler@gmail.com>
 *   Author: @author Tobias Naegeli <naegelit@student.ethz.ch>
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
 * @file mc_att_control_hcf_params.c
 * Multicopter attitude controller parameters specific to the HCF project.
 *
 *  @author Thomas Gubler <thomasgubler@gmail.com>
 *  @author Tobias Naegeli <naegelit@student.ethz.ch>
 */

/*
 * Input scaler X and Y
 * The stick input is scaled with this number to calculate the force setpoint
 */
PARAM_DEFINE_FLOAT(HA_INP_SXY, 0.5f);

/*
 * Input scaler Z
 * The stick input is scaled with this number to calculate the force setpoint
 */
PARAM_DEFINE_FLOAT(HA_INP_SZ, 0.5f);

/*
 * Roll P Control Gain
 */
PARAM_DEFINE_FLOAT(HA_ATT_P, 1.0f);

/*
 * Roll I Control Gain
 */
PARAM_DEFINE_FLOAT(HA_ATT_I, 0.0f);

/*
 *  Roll D Control Gain
 */
PARAM_DEFINE_FLOAT(HA_ATT_D, 1.0f);

/*
 * Pitch P Control Gain
 */
//PARAM_DEFINE_FLOAT(HA_A_PP, 1.0f);
//
///*
// * Pitch I Control Gain
// */
//PARAM_DEFINE_FLOAT(HA_A_PI, 0.0f);
//
///*
// *  Pitch D Control Gain
// */
//PARAM_DEFINE_FLOAT(HA_A_PD, 1.0f);
//
///*
// * Yaw P Control Gain
// */
PARAM_DEFINE_FLOAT(HA_ATT_YP, 1.0f);

/*
 * Yaw I Control Gain
 */
PARAM_DEFINE_FLOAT(HA_ATT_YI, 0.0f);

/*
 *  Yaw D Control Gain
 */
PARAM_DEFINE_FLOAT(HA_ATT_YD, 1.0f);


/**
 * Roll rate P gain
 *
 * Roll rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @group Multicopter Attitude Control
 */
//PARAM_DEFINE_FLOAT(HA_Rate_RP, 0.1f);
//
///**
// * Roll rate D gain
// *
// * Roll rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
// *
// * @min 0.0
// * @group Multicopter Attitude Control
// */
//
//PARAM_DEFINE_FLOAT(HA_Rate_RD, 0.002f);
///**
// * Pitch rate P gain
// *
// * Pitch rate proportional gain, i.e. control output for angular speed error 1 rad/s.
// *
// * @min 0.0
// * @group Multicopter Attitude Control
// */
PARAM_DEFINE_FLOAT(HA_RATE_P, 0.03f);

/**
 * Pitch rate D gain
 *
 * Pitch rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HA_RATE_D, 0.002f);

/**
 * Yaw rate P gain
 *
 * Yaw rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HA_RATE_YP, 0.15f);

/**
 * Yaw rate D gain
 *
 * Yaw rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(HA_RATE_YD, 0.0f);
