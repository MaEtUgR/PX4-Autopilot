/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: @author Anton Babushkin <anton.babushkin@me.com>
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
 * @file inputloader.cpp
 * Small class that loads the attiude setpoint
 *
 *  @author Thomas Gubler <thomasgubler@gmail.com>
 *  @author Anton Babushkin <anton.babushkin@me.com>
 *
 */

#include "inputloader.h"

#include <px4_config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <systemlib/err.h>
#include <arch/board/board.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>

InputLoader::InputLoader(SuperBlock *parent, const char *name) :
	SuperBlock(parent, name),
	/* Subscriptions */
	//XXX adjust update times
	_att_sp(ORB_ID(vehicle_attitude_setpoint), 15, 0, &getSubscriptions()),
	_att(ORB_ID(vehicle_attitude), 15, 0, &getSubscriptions()),
	_manual_control_sp(ORB_ID(manual_control_setpoint), 15, 0, &getSubscriptions()),
	_control_mode(ORB_ID(vehicle_control_mode), 15, 0, &getSubscriptions()),
	_force_sp(ORB_ID(vehicle_force_setpoint), 15, 0, &getSubscriptions()),
	/* Publications */
	_att_sp_pub(ORB_ID(vehicle_attitude_setpoint), -1, &getPublications()),
	/* Params */
	_stickScaleXY(this, "SXY"),
	_stickScaleZ(this, "SZ"),
	_forceSpNED(),
	_forceSpBodyYaw(),
	_yawSp(0),
	_yawRateSp(0)
{
}

InputLoader::~InputLoader() {};

void InputLoader::update()
{

	//warnx("mc %u of %u", _control_mode.flag_control_manual_enabled, _control_mode.flag_control_offboard_enabled);
	if (_control_mode.get().flag_control_manual_enabled && !_control_mode.get().flag_control_position_enabled) {
		/* manual input, set or modify attitude setpoint */

		//XXX implement sth similar to the commented out code
		//if (!_control_mode.flag_control_climb_rate_enabled) {
			//[> pass throttle directly if not in altitude stabilized mode <]
			//_att_sp_pub.thrust = _manual_control_sp.z;
		//}

		/* In manual mode convert stick to force vector */
		_forceSpBodyYaw(0) = _manual_control_sp.get().x * _stickScaleXY.get();
		_forceSpBodyYaw(1) = _manual_control_sp.get().y * _stickScaleXY.get();
		_forceSpBodyYaw(2) = -_manual_control_sp.get().z * _stickScaleZ.get(); //z down

		/* rotate around yaw */
		math::Matrix<3, 3> R_IB_yaw;
		R_IB_yaw.from_euler(0.0f, 0.0f, _att.get().yaw);
		_forceSpNED = R_IB_yaw * _forceSpBodyYaw;

		/* publish attitude sp */
		//warnx("x_b %.3f y_b %.3f yaw %.3f", _forceSpBodyYaw(0), _forceSpBodyYaw(1), _att.yaw);
		//warnx("x_n %.3f y_n %.3f", _forceSpNED(0), _forceSpNED(1));
	} else if (_control_mode.get().flag_control_offboard_enabled) {
		/* XXX: implement auto selection of correct setpoint in offboard mode
		 * using attitude setpoint if available (attitude offboard control),
		* else use force setpoint (position offboard control)*/
		if (_control_mode.get().flag_control_attitude_enabled) {
			if(_att_sp.get().q_d_valid) {
				/* use quaternion setpoint if it is available */
				quaternion_to_force(_att_sp.get().q_d, _forceSpNED.data);
				/* scale with thrust */
				_forceSpNED *= _att_sp.get().thrust;
				//warnx("offb quat fsp %.3f %.3f %.3f", (double)_forceSpNED(0), (double)_forceSpNED(1), (double)_forceSpNED(2));
			//XXX yaw handle yaw
			} else {
				//if (_force_sp.frame == FORCE_SETPOINT_FRAME_BODY_NED) {
					_forceSpBodyYaw(0)=_force_sp.get().x; //-_force_sp.y (Thomas implementation)
					_forceSpBodyYaw(1)=_force_sp.get().y; //_force_sp.x (Thomas implementation)
					_forceSpBodyYaw(2)=_force_sp.get().z;

					/* offboard force control */
					/* rotate around yaw */
					math::Matrix<3, 3> R_IB_yaw;
					R_IB_yaw.from_euler(0.0f, 0.0f, _att.get().yaw);
					_forceSpNED = R_IB_yaw * _forceSpBodyYaw;
				/*} else if (_force_sp.frame == FORCE_SETPOINT_FRAME_LOCAL_NED) {
					_forceSpNED(0) = _force_sp.x;
					_forceSpNED(1) = _force_sp.y;
					_forceSpNED(2) = _force_sp.z;
				}*/// TODO: make consistent with code (where are these macros used)
				_yawSp=_force_sp.get().yaw;
				_yawRateSp=_force_sp.get().yaw_rate;

			}
		} else if (_control_mode.get().flag_control_force_enabled) {

			//if (_force_sp.frame == FORCE_SETPOINT_FRAME_BODY_NED) {
				_forceSpBodyYaw(0)=_force_sp.get().x; //-_force_sp.y (Thomas implementation)
				_forceSpBodyYaw(1)=_force_sp.get().y; //_force_sp.x (Thomas implementation)
				_forceSpBodyYaw(2)=_force_sp.get().z;

				/* offboard force control */
				/* rotate around yaw */
				math::Matrix<3, 3> R_IB_yaw;
				R_IB_yaw.from_euler(0.0f, 0.0f, _att.get().yaw);
				_forceSpNED = R_IB_yaw * _forceSpBodyYaw;
			/*} else if (_force_sp.frame == FORCE_SETPOINT_FRAME_LOCAL_NED) {
				_forceSpNED(0) = _force_sp.x;
				_forceSpNED(1) = _force_sp.y;
				_forceSpNED(2) = _force_sp.z;
			}*/// TODO: make consistent with code (where are these macros used)

			// copy the yaw and yawrate
			_yawSp=_force_sp.get().yaw;
			_yawRateSp=_force_sp.get().yaw_rate;
			//warnx("offb fsp %.3f %.3f %.3f", (double)_force_sp.x, (double)_force_sp.y, (double)_force_sp.z);
		}
	} else {
		/* In auto
		 * if force sp available just copy the force setpoint from uorb to _forceSp
		 * else: generate force sp from desired attitude */
		//XXX: check that force setpoint is available
		_forceSpNED(0) = _force_sp.get().x;
		_forceSpNED(1) = _force_sp.get().y;
		_forceSpNED(2) = _force_sp.get().z;
		//warnx("fsp %.3f %.3f %.3f", (double)_forceSpNED(0), (double)_forceSpNED(1), (double)_forceSpNED(2));
	}
}

void InputLoader::quaternion_to_force(const float quaternion[4], float force[3])
{
    double a = quaternion[0];
    double b = quaternion[1];
    double c = quaternion[2];
    double d = quaternion[3];
    double aSq = a * a;
    double bSq = b * b;
    double cSq = c * c;
    double dSq = d * d;
    force[0] = -2.0 * (a * c + b * d);
    force[1] = -2.0 * (c * d - a * b);
    force[2] = -(aSq - bSq - cSq + dSq);
}

