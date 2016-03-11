/**
 * @file BlockMControl.hpp
 * @author Matthias Grob <maetugr@gmail.com>
 *
 * Nonlinear Quadrotor Controller (master thesis)
 */

#pragma once
#include <px4_posix.h>
#include <mathlib/mathlib.h>
#include <controllib/uorb/blocks.hpp>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_force_setpoint.h>

class BlockMControl : public control::SuperBlock {
public:
	BlockMControl();
	void update();
private:
	uORB::Subscription<control_state_s>				_sub_control_state;
	uORB::Subscription<vehicle_force_setpoint_s>	_sub_force_setpoint;
	uORB::Publication<actuator_controls_s>			_pub_actuator_controls;

	px4_pollfd_struct_t _control_state_Poll;						// file descriptors struct to feed the system call poll
	uint64_t _timeStamp;											// last time the loop ran to calculate dt
	math::Vector<3> _rates_prev;
};
