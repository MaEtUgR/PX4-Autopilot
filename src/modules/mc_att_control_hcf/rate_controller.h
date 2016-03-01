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
 * @file rate_controller.h
 * Small class that performs attitude rate control
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
#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>
#include <drivers/drv_hrt.h>
#include <systemlib/perf_counter.h>
#include <uORB/topics/actuator_controls.h>

class RateController : public control::SuperBlock
{
public:
	/**
	 * Constructor
	 */
	RateController(SuperBlock *parent, const char *name);

	/**
	 * Destructor
	 */
	~RateController();

	/**
	 * Main task to get actuator output
	 */
	void update(math::Vector<3> rates_sp, const float thrust_sp, math::Vector<3> rates);

	/* Custom updateParams function to copy params into vector form */
	virtual void updateParams();

private:
	/* Subscriptions */

	/* Publications */
	uORB::Publication<actuator_controls_s> _actuator_controls_pub;		/**< actuator controls pub */

	/* Params */
	control::BlockParamFloat _Kprate;
	control::BlockParamFloat _Kdrate;
//	control::BlockParamFloat _KpPitchrate;
//	control::BlockParamFloat _KdPitchrate;
	control::BlockParamFloat _KpYawrate;
	control::BlockParamFloat _KdYawrate;

	/* Params as vectors (copied in updateParams) */
	math::Vector<3> _rate_p;				/**< P gain for angular rate error */
	math::Vector<3> _rate_d;				/**< D gain for angular rate error */

	math::Vector<3> _rates_prev; /** system body angular rates of previous call */
	math::Vector<3> _control_output; /** actuator ouput, should be published to actuator controls topic together with thrust */

};
