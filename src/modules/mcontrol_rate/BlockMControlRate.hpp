/**
 * @file BlockMControlRate.hpp
 * @author Matthias Grob <maetugr@gmail.com>
 *
 * High Frequency Rate Controller for Multicopter
 */

#pragma once
#include <px4_posix.h>
#include <drivers/drv_hrt.h>
#include <matrix/math.hpp>
#include <controllib/block/Block.hpp>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/actuator_outputs.h>

class BlockMControlRate : public control::Block {
public:
	BlockMControlRate();
	~BlockMControlRate();
	void update();
private:
	uORB::Subscription<sensor_gyro_s> _sub_sensor_gyro;
	uORB::Subscription<vehicle_rates_setpoint_s> _sub_vehicle_rates_setpoint;
	uORB::Publication<actuator_controls_s> _pub_actuator_controls;

	bool pollGyro();
	px4_pollfd_struct_t _sensor_gyro_poll;	// file descriptor struct to feed the system call poll

	void measureDt();
	uint64_t _dt_last_timestamp;	// last time the loop ran to calculate dt

	void controlRate();

	void publishMomentThrust(matrix::Vector<float,4> moment_thrust);
};
