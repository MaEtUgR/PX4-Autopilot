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
 * @file rate_controller.cpp
 * Small class that performs attitude rate control
 *
 *  @author Thomas Gubler <thomasgubler@gmail.com>
 *  @author Anton Babushkin <anton.babushkin@me.com>
 *
 */

#include "rate_controller.h"

#include <px4_config.h>
#include <px4_defines.h>
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

RateController::RateController(SuperBlock *parent, const char *name) :
	SuperBlock(parent, name),
	/* Publications*/
	_actuator_controls_pub(ORB_ID(actuator_controls_0), -1, &getPublications()),
	/* Params */
	_Kprate(this, "P"),
	_Kdrate(this, "D"),
//	_KpPitchrate(this, "PP"),
//	_KdPitchrate(this, "PD"),
	_KpYawrate(this, "YP"),
	_KdYawrate(this, "YD"),
	_rate_p(),
	_rate_d(),
	_rates_prev(),
	_control_output()
{
}

RateController::~RateController() {};

void RateController::update(math::Vector<3> rates_sp, const float thrust_sp, math::Vector<3> rates)
{
	/* angular rates error */
	//rates_sp(2)=0.0f;
	math::Vector<3> rates_err = rates_sp - rates;
	_control_output = _rate_p.emult(rates_err) + _rate_d.emult(_rates_prev - rates) / getDt();
	_rates_prev = rates;


//					static int counter = 0;
//					if (counter % 200 == 0&& counter>1000) {
//						//warnx("kx %.3f ky %.3f kz %.3f ", (double)_KpRollrate.get(), (double) _KpPitchrate.get(),(double)_KpYawrate.get());
//						//warnx("kx %.3f ky %.3f kz %.3f ", (double)_control_output(0), (double) _control_output(1),(double)_control_output(2));
//						//if (isnan(rates_sp(2))==0) {
//						//warnx("sp2 %.3f ", (double)(rates_sp(2)));
//						//warnx("sp1 %.3f ", (double)(rates_sp(1)));
//						//}
//					}
//					counter++;


	/* Publish */
	for (int i = 0; i < 3; i++) {
		_actuator_controls_pub.get().control[i] = PX4_ISFINITE(_control_output(i)) ? _control_output(i) : 0.0f;
	}
	_actuator_controls_pub.get().control[3] = PX4_ISFINITE(thrust_sp) ? thrust_sp : 0.0f;
	_actuator_controls_pub.update();
}

void RateController::updateParams()
{
	SuperBlock::updateParams();
	/* copy parameters into vector */
	_rate_p(0) = _Kprate.get();
	_rate_p(1) = _Kprate.get();
	_rate_p(2) = _KpYawrate.get();
	_rate_d(0) = _Kdrate.get();
	_rate_d(1) = _Kdrate.get();
	_rate_d(2) = _KdYawrate.get();

}
