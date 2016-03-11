/**
 * @file BlockMControl.cpp
 * @author Matthias Grob <maetugr@gmail.com>
 *
 * Nonlinear Quadrotor Controller (master thesis)
 */

#include "BlockMControl.hpp"

BlockMControl::BlockMControl() :
		SuperBlock(NULL, "MCONTROL"),
		_ctrl_state_sub(ORB_ID(control_state), 0, 0, &getSubscriptions()),
		_timeStamp(0)
{
	_ctrl_state_Poll.fd = _ctrl_state_sub.getHandle();
	_ctrl_state_Poll.events = POLLIN;
}

void BlockMControl::update() {
	if (px4_poll(&_ctrl_state_Poll, 1, 100) < 0) {				// poll on attitude topic to synchronize the loop to new data
		warn("mcontrol: poll error");
		return;
	}
	updateSubscriptions();

	uint64_t newTimeStamp = hrt_absolute_time();
	float dt = (newTimeStamp - _timeStamp) / 1e6f;
	_timeStamp = newTimeStamp;

	printf("dt: % .1f Gyro: % .1f % .1f % .1f:\n",(double)dt*1e3, (double)_ctrl_state_sub.get().roll_rate, (double)_ctrl_state_sub.get().pitch_rate, (double)_ctrl_state_sub.get().yaw_rate);
}
