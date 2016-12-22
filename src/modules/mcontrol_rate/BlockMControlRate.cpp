/**
 * @file BlockMControl.cpp
 * @author Matthias Grob <maetugr@gmail.com>
 */

#include "BlockMControlRate.hpp"

using namespace matrix;

BlockMControlRate::BlockMControlRate() :
		Block(NULL, "RATECONTROL"),
		_sub_sensor_gyro(ORB_ID(sensor_gyro), 0, 0, &getSubscriptions()),
		_sub_manual_control_setpoint(ORB_ID(manual_control_setpoint), 0, 0, &getSubscriptions()),
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
	//Eulerf eul(0,0,M_PI*0.5);
	//Quatf q(eul);
	//Dcmf R(q);

	//eul.print();
	//q.print();
	//R.print();
	(Quatf(1,2,3,4)*Quatf(5,6,7,8)).print();


	Vector3f rate(&_sub_sensor_gyro.get().x);
	//Vector3f rate_setpoint(&_sub_vehicle_rates_setpoint.get().roll);
	Vector<float, 4> manual_setpoint;
	manual_setpoint(0) = _sub_manual_control_setpoint.get().y;
	manual_setpoint(1) = -_sub_manual_control_setpoint.get().x;
	manual_setpoint(2) = _sub_manual_control_setpoint.get().r;
	manual_setpoint(3) = _sub_manual_control_setpoint.get().z;
	Vector3f rate_error = rate - 4.f*manual_setpoint.slice<3,1>(0,0);
	Vector3f rate_gains_p(0.1f,0.1f,1.0f);
	Vector3f moment = -rate_error.emult(rate_gains_p);
	Vector<float, 4> moment_thrust;
	moment_thrust.set(moment,0,0);
	//moment_thrust(3) = _sub_vehicle_rates_setpoint.get().thrust;
	moment_thrust(3) = manual_setpoint(3);
	//moment_thrust.print();
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
