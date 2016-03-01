/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file mc_att_control_hcf_main.cpp
 * Multicopter attitude controller for the HCF project.
 *
 *  @author Thomas Gubler <thomasgubler@gmail.com>
 *  @author Tobias Naegeli <naegelit@student.ethz.ch>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>

#include "mc_att_control_hcf.h"

static bool thread_should_exit = false;  	/**< Deamon exit flag */
static bool thread_running = false;      	/**< Deamon status flag */
static int daemon_task;			/**< Handle of deamon task / thread */

/**
 * Deamon management function.
 */
extern "C" __EXPORT int mc_att_control_hcf_main(int argc, char *argv[]);

/**
 * Mainloop of deamon.
 */
int mc_att_control_hcf_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	warnx("usage: mc_att_control_hcf {start|stop|status}");
}

/**
 * The deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int mc_att_control_hcf_main(int argc, char *argv[])
{
	//redirect stdout to usb console
	int fd = open("/dev/ttyACM0", O_RDWR);
	(void)dup2(fd, 0);
	(void)dup2(fd, 1);
	(void)dup2(fd, 2);
	close(fd);

	if (argc < 2) {
		usage(NULL);
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("already running");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		warnx("task spawn");
		daemon_task = px4_task_spawn_cmd("mc_att_control_hcf",
					     SCHED_DEFAULT,
					     SCHED_PRIORITY_MAX - 30,
					     3000,
					     mc_att_control_hcf_thread_main,
						 nullptr);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("is running\n");
			return 0;

		} else {
			warnx("not started\n");
			return 1;
		}

	}

	usage("unrecognized command");
	return 1;
}

int mc_att_control_hcf_thread_main(int argc, char *argv[])
{
	warnx("starting");

	thread_running = true;

	MulticopterAttitudeControlHcf multicopterAttitudeControlHcf(NULL, "HA"); //_H_uman Centric Flight _A_ttitude

	while (!thread_should_exit) {
		multicopterAttitudeControlHcf.update();
	}

	warnx("exiting.");

	thread_running = false;

	return 0;
}
