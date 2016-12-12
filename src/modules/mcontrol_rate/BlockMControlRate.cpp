/**
 * @file BlockMControl.cpp
 * @author Matthias Grob <maetugr@gmail.com>
 */

#include "BlockMControlRate.hpp"

using namespace matrix;

BlockMControlRate::BlockMControlRate() :
		Block(NULL, "RATECONTROL"),
		_sub_sensor_gyro(ORB_ID(sensor_gyro), 0, 0, &getSubscriptions()),
		_sub_vehicle_rates_setpoint(ORB_ID(vehicle_rates_setpoint), 0, 0, &getSubscriptions()),
		_pub_actuator_controls(ORB_ID(actuator_controls_0), -1, &getPublications()),
		_dt_last_timestamp(hrt_absolute_time())
{
	_sensor_gyro_poll = {_sub_sensor_gyro.getHandle(), POLLIN, 0};	// setup of the poll struct for sensor_gyro topic
}

BlockMControlRate::~BlockMControlRate() {};

void BlockMControlRate::update() {
	if(!pollGyro()) return;	// poll on gyroscope topic to synchronize the loop to new sensor data
	updateSubscriptions();
	measureDt();

	controlRate();
}

bool BlockMControlRate::pollGyro() {
	if(px4_poll(&_sensor_gyro_poll, 1, 10000) <= 0) {
		warnx("sensor_gyro: poll error");
		return false;
	}
	return true;
}

void BlockMControlRate::measureDt() {
	uint64_t dt_new_timestamp = hrt_absolute_time();
	setDt((dt_new_timestamp - _dt_last_timestamp) / 1e6f);
	_dt_last_timestamp = dt_new_timestamp;
}

void BlockMControlRate::controlRate() {
	Vector3f rate(&_sub_sensor_gyro.get().x);
	Vector3f rate_setpoint(&_sub_vehicle_rates_setpoint.get().roll);
	Vector3f rate_error = rate - rate_setpoint;
	Vector3f moment = -1.f * rate_error;
	Vector<float, 4> moment_thrust;
	moment_thrust.set(moment,0,0);
	moment_thrust(3) = _sub_vehicle_rates_setpoint.get().thrust;
	moment_thrust.print();
	publishMomentThrust(moment_thrust);
}

void BlockMControlRate::publishMomentThrust(Vector<float, 4> moment_thrust) {
	_pub_actuator_controls.get().timestamp = hrt_absolute_time();
	_pub_actuator_controls.get().timestamp_sample = _sub_sensor_gyro.get().timestamp;
	for(int i = 0; i < 4; i++) {
		_pub_actuator_controls.get().control[i] = PX4_ISFINITE(moment_thrust(i)) ? moment_thrust(i) : 0;
	}
	_pub_actuator_controls.update();
}
