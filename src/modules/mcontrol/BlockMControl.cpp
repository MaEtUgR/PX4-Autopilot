/**
 * @file BlockMControl.cpp
 * @author Matthias Grob <maetugr@gmail.com>
 *
 * Nonlinear Quadrotor Controller (master thesis)
 */

#include "BlockMControl.hpp"

using namespace matrix;

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
	_Rd_prev.setZero();
	_Od_prev.setZero();
}

void BlockMControl::update() {
	if(!poll_control_state()) return;		// poll on attitude topic to synchronize the loop to new data
	updateSubscriptions();
	calculate_dt();

	//printf("Joystick: % .1f % .1f % .1f % .1f\n", (double)_sub_force_setpoint.get().x, (double)_sub_force_setpoint.get().y, (double)_sub_force_setpoint.get().z, (double)_sub_force_setpoint.get().yaw_rate);

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
	Eulerf euler(_sub_force_setpoint.get().x/2, _sub_force_setpoint.get().y/2, _sub_force_setpoint.get().z/2);	// joystick euler attitude set point TODO: better translation through vector
	Dcmf dcm(euler);

	Matrix3f R(_sub_vehicle_attitude.get().R); 											// R   : attitude			= estimated attitude from uORB topic
	Matrix3f Rd = dcm;																	// Rd  : desired attitude
	Vector3f e_R = 1/2.f * ((matrix::Matrix3f)(Rd.T() * R - R.T() * Rd)).V();			// e_R : attitude error		= 1/2 * (Rd' R - R' * Rd)^V

	Matrix3f Rd_d = (Rd - _Rd_prev) / _dt;												// Rd_d: derivative of desired attitude
	_Rd_prev = Rd;

	Vector3f O(&_sub_control_state.get().roll_rate);									// O   : rate				= gyroscope measurement from uORB topic
	Vector3f Od = 1/2.0f * ((matrix::Matrix3f)(Rd.T() * Rd_d - Rd_d.T() * Rd)).V();		// Od  : desired rate		= 1/2 * (Rd' Rd_d - Rd_d' * Rd)^V
	Vector3f e_O = O - R.T() * Rd * Od;													// e_O : rate error

	Matrix3f J = diag(Vector3f({0.0347563, 0.0458929, 0.0977}));
	Vector3f e_C = O % (J * O);															// e_C : coriolis error because O being in body frame TODO: centripetal important?? http://sal.aalto.fi/publications/pdf-files/eluu11_public.pdf p. 5

	Vector3f Od_d = (Od - _Od_prev) / _dt;												// Od_d: derivative of desired rate
	_Od_prev = Od;

	Vector3f e_M = J * (O.Hat()*R.T()*Rd*Od - R.T()*Rd*Od_d);							// e_M : Model feed forward	= J * (O^^*R'*Rd*Od - R'*Rd*Od_d)

	printf("e_R\n"); e_R.print();
	printf("e_O\n"); e_O.print();
	printf("e_C\n"); e_C.print();
	printf("e_M\n"); e_M.print();

	Vector3f m = -0.60f * e_O -5.0f * e_R + e_C; //- e_M;									// m   : angular moment to apply to quad

	float thrust_desired = _sub_force_setpoint.get().yaw_rate*1.4f-0.4f;
	publishMoment(thrust_desired, m);
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

	publishMoment(thrust_sp, Vector3f(_control_output.data));
}

void BlockMControl::publishMoment(float thrust, matrix::Vector3f moment) {
	for(int i = 0; i < 3; i++) {
		_pub_actuator_controls.get().control[i] = PX4_ISFINITE(moment(i)) ? moment(i) : 0.0f;
	}
	_pub_actuator_controls.get().control[3] = PX4_ISFINITE(thrust) && thrust > 0.2f ? thrust : 0.2f;
	_pub_actuator_controls.update();
}
