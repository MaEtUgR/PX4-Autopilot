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
 * @file mc_att_control_hcf.h
 * Multicopter attitude controller for the HCF project.
 *
 *  @author Thomas Gubler <thomasgubler@gmail.com>
 *  @author Tobias Naegeli <naegelit@student.ethz.ch>
 */

#include <mathlib/mathlib.h>
#include <systemlib/param/param.h>
#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>
#include <drivers/drv_hrt.h>
#include <systemlib/perf_counter.h>

#include "codegen/attitude_controller.h"
#include "attitude_controller.h"
#include "inputloader.h"
#include "rate_controller.h"

/**
 * Multicopter position control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mc_att_control_hcf_main(int argc, char *argv[]);

class MulticopterAttitudeControlHcf : public control::SuperBlock
{
public:
	/**
	 * Constructor
	 */
	MulticopterAttitudeControlHcf(SuperBlock *parent, const char *name);

	/**
	 * Destructor, also kills task.
	 */
	~MulticopterAttitudeControlHcf();

	/**
	 * Main sensor collection task.
	 */
	void update();

private:

	int		_mavlink_fd;			/**< mavlink fd */
	bool		_global_pos_mode_enabled;	/**< if set to true the reprojection of the estimation is
							  published as global pos */
	/* Publications */

	/* Subscriptions */
	uORB::Subscription<vehicle_attitude_s> 			_att;			/**< vehicle attitude */
	uORB::Subscription<actuator_armed_s>			_arming;		/**< actuator arming status */
	uORB::Subscription<vehicle_control_mode_s>		_control_mode;		/**< vehicle control mode */

	/* Params */
	hrt_abstime t_prev;		/**< Timestamp of last iteration */

	/* Flags */
	bool _was_armed;
	bool _was_attcontrol;
	bool _initialized;
	bool _debug;			/**< Set true to enable debug output */

	/* Performance counters */
	perf_counter_t _perfCounterDt;
	perf_counter_t _perfCounterDuration;

	/* Other Sub blocks */
	InputLoader _inputLoader; /**< maps the stick setpoint to a force */
	AttitudeController _attitudeController; /**< handles the attitude control calculation (outer loop) */
	RateController _rateController; /**< handles the rate control calculation (inner loop) */

	/**
	 * Initialize state of the filter if not yet initialized
	 */
	int initStateIfNecessary();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	static void debug_print(const char *fmt, va_list args);
	void	    debug(const char *fmt, ...);
};
