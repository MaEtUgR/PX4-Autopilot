/**
 * @file BlockMControl.cpp
 * @author Matthias Grob <maetugr@gmail.com>
 *
 * Nonlinear Quadrotor Controller (master thesis)
 */

#include "BlockMControl.hpp"

BlockMControl::BlockMControl() : BlockUorbEnabledAutopilot(NULL, "MCONTROL") {
	printf("CLASS UP AND RUNNING!!!\n");
}
