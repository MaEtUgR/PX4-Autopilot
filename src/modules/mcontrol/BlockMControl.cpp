/**
 * @file BlockMControl.cpp
 * @author Matthias Grob <maetugr@gmail.com>
 *
 * Nonlinear Quadrotor Controller (master thesis)
 */

#include "BlockMControl.hpp"

BlockMControl::BlockMControl() :
		SuperBlock(NULL, "MCONTROL"),
		_sub_control_state(ORB_ID(control_state), 0, 0, &getSubscriptions()),
		_sub_vehicle_attitude(ORB_ID(vehicle_attitude), 0, 0, &getSubscriptions()),
		_sub_force_setpoint(ORB_ID(vehicle_force_setpoint), 0, 0, &getSubscriptions()),
		_pub_actuator_controls(ORB_ID(actuator_controls_0), -1, &getPublications()),
		_dt(0),
		_dt_timeStamp(0)
{
	_control_state_Poll.fd = _sub_control_state.getHandle();
	_control_state_Poll.events = POLLIN;
}

void BlockMControl::update() {
	if(!poll_control_state()) return;		// poll on attitude topic to synchronize the loop to new data
	updateSubscriptions();
	calculate_dt();

	//printf("dt: % .1fms Gyro: % .1f % .1f % .1f:\n",(double)dt*1e3, (double)_sub_control_state.get().roll_rate, (double)_sub_control_state.get().pitch_rate, (double)_sub_control_state.get().yaw_rate);
	//printf("Joystick: % .1f % .1f % .1f % .1f\n", (double)_sub_force_setpoint.get().x, (double)_sub_force_setpoint.get().y, (double)_sub_force_setpoint.get().z, (double)_sub_force_setpoint.get().yaw_rate);

	/*printf("q: ");
	for(int i = 0; i < 4; i++) {
		printf("%f ", (double)_sub_control_state.get().q[i]);
	}
	printf("\n");

	printf("R:\n");
	for(int i = 0; i < 9; i++) {
		printf("%f ", (double)_sub_vehicle_attitude.get().R[i]);
		if(i % 3 == 2)
			printf("\n");
	}*/

	Controller();
	//rateController_original();
}

bool BlockMControl::poll_control_state() {
	if (px4_poll(&_control_state_Poll, 1, 100) < 0) {
		warn("mcontrol: poll error");
		return false;
	}
	return true;
}

void BlockMControl::calculate_dt() {
	uint64_t newTimeStamp = hrt_absolute_time();
	_dt = (newTimeStamp - _dt_timeStamp) / 1e6f;
	_dt_timeStamp = newTimeStamp;
}

void BlockMControl::Controller() {
	matrix::Vector3f rates(_sub_control_state.get().roll_rate, _sub_control_state.get().pitch_rate, _sub_control_state.get().yaw_rate);
	matrix::Vector3f rates_desired(_sub_force_setpoint.get().x, _sub_force_setpoint.get().y, _sub_force_setpoint.get().z);
	matrix::Vector3f rates_error = rates - rates_desired;

	matrix::Matrix3f attitude(_sub_vehicle_attitude.get().R);
	matrix::Matrix3f attitude_desired = matrix::eye<float, 3>();
	matrix::Matrix3f attitude_error_M = attitude_desired.T()*attitude - attitude.T()*attitude_desired;
	matrix::Vector3f attitude_error = 1/2.0f * ((matrix::Matrix3f)(attitude_desired.T()*attitude - attitude.T()*attitude_desired)).UnHat();

	//printf("Attitude:\n"); attitude.print();
	//printf("Attitude^T:\n"); attitude.T().print();
	//printf("Attitude_err_M:\n"); attitude_error_M.print();
	printf("Attitude_err:\n"); attitude_error.print();

	matrix::Vector3<float> momentum = -0.48f * rates_error -3.0f * attitude_error;

	for(int i = 0; i < 3; i++) {
		_pub_actuator_controls.get().control[i] = PX4_ISFINITE(momentum(i)) ? momentum(i) : 0.0f;
	}
	float thrust_desired = _sub_force_setpoint.get().yaw_rate*1.4f-0.4f;
	_pub_actuator_controls.get().control[3] = PX4_ISFINITE(thrust_desired) && thrust_desired > 0.1f ? thrust_desired : 0.1f;
	_pub_actuator_controls.update();
}

void BlockMControl::rateController_original() {
	math::Vector<3> rates = {_sub_control_state.get().roll_rate, _sub_control_state.get().pitch_rate, _sub_control_state.get().yaw_rate};
	math::Vector<3> rates_sp = {_sub_force_setpoint.get().x, _sub_force_setpoint.get().y, _sub_force_setpoint.get().z};
	float thrust_sp = _sub_force_setpoint.get().yaw_rate*1.4f-0.4f;

	math::Vector<3> _rate_p = {0.48f, 0.48f, 0.15f};
	math::Vector<3> _rate_d = {0.002f, 0.002f, 0.0f};

	math::Vector<3> rates_err = rates_sp*2 - rates;
	math::Vector<3> _control_output = _rate_p.emult(rates_err) + _rate_d.emult(_rates_prev - rates) / _dt;
	_rates_prev = rates;

	for(int i = 0; i < 3; i++) {
		_pub_actuator_controls.get().control[i] = PX4_ISFINITE(_control_output(i)) ? _control_output(i) : 0.0f;
	}
	_pub_actuator_controls.get().control[3] = PX4_ISFINITE(thrust_sp) && thrust_sp > 0.1f ? thrust_sp : 0.1f;
	_pub_actuator_controls.update();
}
