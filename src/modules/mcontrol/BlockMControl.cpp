/**
 * @file BlockMControl.cpp
 * @author Matthias Grob <maetugr@gmail.com>
 *
 * Nonlinear Quadrotor Controller (master thesis)
 */

#include "BlockMControl.hpp"

using namespace matrix;

BlockMControl::BlockMControl(bool simulation) :
		SuperBlock(NULL, "MCONTROL"),
		_sub_control_state(ORB_ID(control_state), 0, 0, &getSubscriptions()),
		_sub_vehicle_attitude(ORB_ID(vehicle_attitude), 0, 0, &getSubscriptions()),
		_sub_force_setpoint(ORB_ID(vehicle_force_setpoint), 0, 0, &getSubscriptions()),
		_sub_manual_control_setpoint(ORB_ID(manual_control_setpoint), 0, 0, &getSubscriptions()),
		_pub_actuator_controls(ORB_ID(actuator_controls_0), -1, &getPublications()),
		_dt(0),
		_dt_timeStamp(0),
		_joystick{0,0,0,0},
		maxeO(0),
		maxeC(0)
{
	_control_state_Poll.fd = _sub_control_state.getHandle();
	_control_state_Poll.events = POLLIN;
	_Rd_prev.setZero();
	_Od_prev.setZero();

	_simulation = simulation;				// TODO: distinction between simulation and reality should not be needed, inputs have to get the same and parameters have to be set differently
}

void BlockMControl::update() {
	if(!poll_control_state()) return;		// poll on attitude topic to synchronize the loop to new data
	updateSubscriptions();
	calculate_dt();
	get_joystick_data();

	//printf("Joystick: % .1f % .1f % .1f % .1f\n", (double)_joystick[0], (double)_joystick[1], (double)_joystick[2], (double)_joystick[3]);
	//printf("Kill: %d\n", _sub_manual_control_setpoint.get().kill_switch);

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

void BlockMControl::get_joystick_data() {
	if(_simulation) {											// remote control from ros node with xbox controller (simulation)
		_joystick[0] = _sub_force_setpoint.get().x;
		_joystick[1] = _sub_force_setpoint.get().y;
		_joystick[2] = _sub_force_setpoint.get().z;
		_joystick[3] = _sub_force_setpoint.get().yaw_rate;
	} else {													// remote control from dedicated pixhawk RC (real QC)
		_joystick[0] =  _sub_manual_control_setpoint.get().y;	// roll
		_joystick[1] = -_sub_manual_control_setpoint.get().x;	// pitch
		_joystick[2] =  _sub_manual_control_setpoint.get().r;	// yaw
		_joystick[3] =  _sub_manual_control_setpoint.get().z;	// throttle
		if(_joystick[3] < 0.1f)					// make arming/disarming not produce yaw input
			_joystick[2] = 0;
	}
}

void BlockMControl::Controller() {
	Eulerf euler(_joystick[0]/1.5f, _joystick[1]/1.5f, 0);				// joystick euler attitude set point TODO: better translation through vector
	Dcmf dcm(euler);

	Matrix3f R(_sub_vehicle_attitude.get().R); 											// R   : attitude			= estimated attitude from uORB topic
	float yaw = atan2(R(1,0),R(0,0));

	Matrix3f Rd = Dcmf(Eulerf(0,0,yaw)) * dcm;																	// Rd  : desired attitude
	Vector3f R_z(R(0, 2), R(1, 2), R(2, 2));
	Vector3f Rd_z(Rd(0, 2), Rd(1, 2), Rd(2, 2));
	Vector3f e_R = R.T() * (Rd_z % R_z);
	//Vector3f e_R = 1/2.f * ((matrix::Matrix3f)(Rd.T() * R - R.T() * Rd)).V();			// e_R : attitude error		= 1/2 * (Rd' R - R' * Rd)^V

	Matrix3f Rd_d = (Rd - _Rd_prev) / _dt;												// Rd_d: derivative of desired attitude
	_Rd_prev = Rd;

	Vector3f O(&_sub_control_state.get().roll_rate);									// O   : rate				= gyroscope measurement from uORB topic
	Vector3f Od = 1/2.0f * ((matrix::Matrix3f)(Rd.T() * Rd_d - Rd_d.T() * Rd)).V();		// Od  : desired rate		= 1/2 * (Rd' Rd_d - Rd_d' * Rd)^V
	Vector3f e_O = O - Vector3f(0,0,_joystick[2]*2);// - R.T() * Rd * Od;													// e_O : rate error

	Matrix3f J = diag(Vector3f({0.0347563, 0.0458929, 0.0977}));
	Vector3f e_C = O % (J * O);															// e_C : coriolis error because O being in body frame TODO: centripetal important?? http://sal.aalto.fi/publications/pdf-files/eluu11_public.pdf p. 5

	Vector3f Od_d = (Od - _Od_prev) / _dt;												// Od_d: derivative of desired rate
	_Od_prev = Od;

	Vector3f O_d = (O - _O_prev) / _dt;													// O_d: derivative of the rate TODO: estimate angular acceleration
	_O_prev = O;

	Vector3f e_M = J * (O.Hat()*R.T()*Rd*Od /*- R.T()*Rd*Od_d*/);							// e_M : Model feed forward	= J * (O^^*R'*Rd*Od - R'*Rd*Od_d)

	//usleep(10000);
	/*printf("R valid: %d\n",_sub_vehicle_attitude.get().R_valid);
	printf("q\n"); matrix::Quaternion<float>(_sub_vehicle_attitude.get().q[0], _sub_vehicle_attitude.get().q[1], _sub_vehicle_attitude.get().q[2], _sub_vehicle_attitude.get().q[3]).print();
	printf("R\n"); R.print();
	printf("R2\n"); R2.print();
	printf("e_O\n"); e_O.print();
	printf("e_R\n"); (e_R*100).print();*/
	/*float eOnorm = e_O.norm(); // e_C significance test
	float eCnorm = e_C.norm();
	maxeO = eOnorm > maxeO ? eOnorm : maxeO;
	maxeC = eCnorm > maxeC ? eCnorm : maxeC;
	printf("maxeO %f\n", (double)maxeO);
	printf("maxeC %f\n", (double)maxeC);*/
	/*printf("e_O\n"); e_O.print();
	printf("e_C\n"); e_C.print();*/
	//printf("e_M\n"); e_M.print();

	Vector3f m;
	if(_simulation)
		m = -0.60f * e_O -2.0f * e_R /*+ e_C*/ /*- e_M*/;									// m   : angular moment to apply to quad
	else {
		m = -0.06f * e_O -0.2f * e_R /*-0.004f * O_d*/;
		m = m.emult(Vector3f(1,1,1));															// draft for using different gains depending on roll, pitch or yaw
	}

	float thrust_desired = _joystick[3]*1.4f-0.4f;
	thrust_desired = thrust_desired > 0 ? thrust_desired : 0;
	//printf("thrust_desired: %f\n",(double)thrust_desired);
	publishMoment(thrust_desired, m);
}

void BlockMControl::rateController_original() {
	math::Vector<3> rates = {_sub_control_state.get().roll_rate, _sub_control_state.get().pitch_rate, _sub_control_state.get().yaw_rate};
	math::Vector<3> rates_sp = {_joystick[0], _joystick[1], _joystick[2]};
	float thrust_sp = _joystick[3]*1.4f-0.4f;

	math::Vector<3> _rate_p;
	if(_simulation)
		_rate_p = {0.48f, 0.48f, 0.15f};
	else
		_rate_p = {0.1f, 0.1f, 0.1f};

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
	_pub_actuator_controls.get().control[3] = PX4_ISFINITE(thrust) && thrust > 0.0f ? thrust : 0.0f;
	_pub_actuator_controls.update();
}
