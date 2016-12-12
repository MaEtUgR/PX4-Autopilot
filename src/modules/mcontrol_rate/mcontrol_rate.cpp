/**
 * @file mc_rate_control.cpp
 * @author Matthias Grob <maetugr@gmail.com>
 *
 * Low Latency Rate Controller for Multicopter (started December 2016)
 */

#include <px4.h>
#include "BlockMControlRate.hpp"

extern "C" __EXPORT int mcontrol_rate_main(int argc, char *argv[]);
int mcontrol_rate_thread_main(int argc, char *argv[]);

static volatile bool thread_should_exit = false;
static volatile bool thread_running = false;
static int deamon_task_handle;

int mcontrol_rate_main(int argc, char *argv[]) {
	if (argc < 2) {
		warnx("usage: mcontrol_rate {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		if (thread_running) {
			warnx("already running");
			return 0; // this is not an error
		}
		thread_should_exit = false;
		deamon_task_handle = px4_task_spawn_cmd("mcontrol_rate", SCHED_DEFAULT,
				SCHED_PRIORITY_MAX - 5, 10240,
				mcontrol_rate_thread_main,
				(argv && argc > 2) ? (char * const *) &argv[2] : (char * const *) NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (thread_running) {
			warnx("stop");
			thread_should_exit = true;
		} else {
			warnx("not started");
		}
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("is running");
		} else {
			warnx("not started");
		}
		return 0;
	}

	warnx("unrecognized command");
	return 1;
}

int mcontrol_rate_thread_main(int argc, char *argv[]) {
	warnx("starting rate controller");

	BlockMControlRate controller_rate;
	thread_running = true;

	while (!thread_should_exit) {
		controller_rate.update();
	}

	warnx("exiting rate controller");
	thread_running = false;
	return 0;
}
