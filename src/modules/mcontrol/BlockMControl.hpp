/**
 * @file BlockMControl.hpp
 * @author Matthias Grob <maetugr@gmail.com>
 *
 * Nonlinear Quadrotor Controller (master thesis)
 */

#pragma once
#include <px4_posix.h>
#include <controllib/uorb/blocks.hpp>

class BlockMControl : public control::SuperBlock {
public:
	BlockMControl();
	void update();
private:
	uORB::Subscription<control_state_s> _ctrl_state_sub;

	px4_pollfd_struct_t _ctrl_state_Poll;						// struct to feed the system call poll
	uint64_t _timeStamp;										// last time the loop ran to calculate dt
};
