/**
 * @file mcontrol_main.cpp
 * @author Matthias Grob <maetugr@gmail.com>
 *
 * Nonlinear Quadrotor Controller (master thesis)
 */

#include "BlockMControl.hpp"

extern "C" __EXPORT int mcontrol_main(int argc, char *argv[]);	// deamon management function
int mcontrol_thread_main(int argc, char *argv[]);				// main loop of deamon

static volatile bool thread_should_exit = false;				// deamon exit flag
static volatile bool thread_running = false;					// deamon status flag
static int deamon_task;											// handle of deamon task / thread

int mcontrol_main(int argc, char *argv[]) {
	if (argc < 2) {
		warnx("usage: mcontrol {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		if (thread_running) {
			warnx("already running");
			return 0; // this is not an error
		}
		thread_should_exit = false;
		deamon_task = px4_task_spawn_cmd("mcontrol", SCHED_DEFAULT,
				SCHED_PRIORITY_MAX - 5, 4096,
				mcontrol_thread_main,
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

int mcontrol_thread_main(int argc, char *argv[]) {
	warnx("starting");

	bool simulation = false;
	if (argv != NULL && !strcmp(argv[0], "sim"))
		simulation = true;
	BlockMControl Controller(simulation);

	thread_running = true;

	while (!thread_should_exit)
		Controller.update();

	warnx("exiting");
	thread_running = false;
	return 0;
}
