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
 * @file attitude_controller.h
 * Small class that performs attitude control using a matlab generated controller
 *
 *  @author Thomas Gubler <thomasgubler@gmail.com>
 *  @author Anton Babushkin <anton.babushkin@me.com>
 *
 */

#include "codegen/attitude_controller.h"

#include <mathlib/mathlib.h>
#include <systemlib/param/param.h>
#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>
#include <drivers/drv_hrt.h>
#include <systemlib/perf_counter.h>
#include <uORB/topics/vehicle_rates_setpoint.h>


class AttitudeController : public control::SuperBlock
{
public:
	/**
	 * Constructor
	 */
	AttitudeController(SuperBlock *parent, const char *name);

	/**
	 * Destructor
	 */
	~AttitudeController();

	/**
	 * Perform controller caclulations
	 */
	void update(const math::Vector<3> forceSetpointNED,float yawSp,float yawRateSp, const struct vehicle_attitude_s &att, bool intReset);

	/* Accessors */
	math::Vector<3> getRatesSp() { return math::Vector<3>(_attCtlArgs.out.rates_des); }
	float getThrustSp() { return _attCtlArgs.out.thrust; }

private:
	/* Subscriptions */

	/* Publications */
	uORB::Publication<vehicle_rates_setpoint_s> _vehicle_rates_sp_pub;	/**< rates sp pub */

	/* Params */
	control::BlockParamFloat _Kp;   /*< Roll P control gain */
	control::BlockParamFloat _Ki;   /*< Roll I control gain */
	control::BlockParamFloat _Kd;   /*< Roll D control gain */
//	control::BlockParamFloat _KpPitch;  /*< Pitch P control gain */
//	control::BlockParamFloat _KiPitch;  /*< Pitch I control gain */
//	control::BlockParamFloat _KdPitch;  /*< Pitch D control gain */
	control::BlockParamFloat _KpYaw;    /*< Yaw P control gain */
	control::BlockParamFloat _KiYaw;    /*< Yaw I control gain */
	control::BlockParamFloat _KdYaw;    /*< Yaw D control gain */

	/* Params as vectors (copied in updateParams) */
	math::Vector<3> _rate_p;				/**< P gain for angular rate error */
	math::Vector<3> _rate_d;				/**< D gain for angular rate error */

	/* Variables needed for the call to the matlab generated pos control function */
	struct AttituteControllerArguments {
		struct inputs {
			float R_IB[9];
			float yaw;
			float yawOffset;
			float F_des[3];
			float Kp[3];
			float Kd[3];
			float Ki[3];
			float e_Rd_v[3];
			boolean_T intFreeze;
			boolean_T intReset;
		} in;
		struct outputs {
			float rates_des[3];
			float thrust;
		} out;
	} _attCtlArgs;

	math::Vector<3> _rates_prev; /** system body angular rates of previous call */

};
