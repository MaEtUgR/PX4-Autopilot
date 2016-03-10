/**
 * @file mcontrol_main.cpp
 * @author Matthias Grob <maetugr@gmail.com>
 *
 * Nonlinear Quadrotor Controller (master thesis)
 */

#include "BlockMControl.hpp"

extern "C" __EXPORT int mcontrol_main(int argc, char *argv[]);

int mcontrol_main(int argc, char *argv[]) {
	printf("APP UP AND RUNNING!!!\n");
	BlockMControl controller;
	return 0;
}
