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
 * @file inputloader.h
 * Small class that loads the attiude setpoint
 *
 *  @author Thomas Gubler <thomasgubler@gmail.com>
 *  @author Anton Babushkin <anton.babushkin@me.com>
 *
 */

#include <mathlib/mathlib.h>
#include <systemlib/param/param.h>
#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_force_setpoint.h>
#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>
#include <drivers/drv_hrt.h>
#include <systemlib/perf_counter.h>

#include "codegen/attitude_controller.h"

class InputLoader : public control::SuperBlock
{
public:
	/**
	 * Constructor
	 */
	InputLoader(SuperBlock *parent, const char *name);

	/**
	 * Destructor
	 */
	~InputLoader();

	/**
	 * Main task to get input setpoint
	 */
	void update();

	/* Accessors */
	math::Vector<3> getForceSetpointNED() { return _forceSpNED; }
	math::Vector<3> getForceSetpointBodyYaw() { return _forceSpBodyYaw; }
	float getSetpointYaw() { return _yawSp; }
	float getSetpointYawRate() { return _yawRateSp; }

private:
	/* Subscriptions */
	uORB::Subscription<vehicle_attitude_setpoint_s> _att_sp;	/**< vehicle attitude sp */
	uORB::Subscription<vehicle_attitude_s>		_att;	/**< vehicle attitude */
	uORB::Subscription<manual_control_setpoint_s> 	_manual_control_sp;	/**< manual control sp */
	uORB::Subscription<vehicle_control_mode_s>	_control_mode;	/**< vehicle control mode */
	uORB::Subscription<vehicle_force_setpoint_s>	_force_sp;	/**< force setpoint */

	/* Publications */
	uORB::Publication<vehicle_attitude_setpoint_s> _att_sp_pub;	/**< vehicle attitude sp pub */
	/* Params */
	control::BlockParamFloat _stickScaleXY; /**< x and y stick inputs are scaled with this number to caclulate the force
					       setpoint vector */
	control::BlockParamFloat _stickScaleZ; /**< z stick inputs are scaled with this number to caclulate the force
					       setpoint vector */
	math::Vector<3> _forceSpNED;    /** force setpoint in NED frame */
	math::Vector<3> _forceSpBodyYaw; /** force setpoint in frame yawed with current yaw */
	float _yawSp;
	float _yawRateSp;
	/* Convert mavlink quaternion to force vector, matches the mavlink quaternion to dcmimplementation */
	 void quaternion_to_force(const float quaternion[4], float force[3]);
};
