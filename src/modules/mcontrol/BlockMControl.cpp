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
		_sub_vehicle_attitude_setpoint(ORB_ID(vehicle_attitude_setpoint), 0, 0, &getSubscriptions()),
		_pub_actuator_controls(ORB_ID(actuator_controls_0), -1, &getPublications()),
		_dt_timeStamp(0),
		_joystick{0,0,0,0}
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

	//Controller();
	ControllerQ();
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
	setDt((newTimeStamp - _dt_timeStamp) / 1e6f);
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
	Dcmf Rjoy(euler);

	Matrix3f R(_sub_vehicle_attitude.get().R); 											// R   : attitude			= estimated attitude from uORB topic

	//float yaw = atan2(R(1,0),R(0,0));				// direct desired attitude hack
	Matrix3f Rd(_sub_vehicle_attitude_setpoint.get().R_body);							// Rd  : desired attitude	= desired attitude from uORB topic
	Rd = /*Dcmf(Eulerf(0,0,yaw)) **/ Rjoy;

	float r[] = {1,0,0, 0,-1,0, 0,0,-1};
	if(_joystick[3] < 0)
		Rd = Matrix3f(r);

	Vector3f R_z(R(0, 2), R(1, 2), R(2, 2));		// reduced attitude control
	Vector3f Rd_z(Rd(0, 2), Rd(1, 2), Rd(2, 2));
	Vector3f e_R = R.T() * (Rd_z % R_z);
	//e_R = 1/2.f * ((matrix::Matrix3f)(Rd.T() * R - R.T() * Rd)).vee();			// e_R : attitude error		= 1/2 * (Rd' R - R' * Rd)^V

	Matrix3f Rd_d = (Rd - _Rd_prev) / getDt();												// Rd_d: derivative of desired attitude
	_Rd_prev = Rd;

	Vector3f O(&_sub_control_state.get().roll_rate);									// O   : rate				= gyroscope measurement from uORB topic
	Vector3f Od = 1/2.0f * ((matrix::Matrix3f)(Rd.T() * Rd_d - Rd_d.T() * Rd)).vee();	// Od  : desired rate		= 1/2 * (Rd' Rd_d - Rd_d' * Rd)^V
	Vector3f e_O = O - Vector3f(0,0,_joystick[2]*2);// - R.T() * Rd * Od;													// e_O : rate error

	Matrix3f J = diag(Vector3f({0.0347563, 0.0458929, 0.0977}));
	Vector3f e_C = O % (J * O);															// e_C : coriolis error because O being in body frame TODO: centripetal important?? http://sal.aalto.fi/publications/pdf-files/eluu11_public.pdf p. 5

	Vector3f Od_d = (Od - _Od_prev) / getDt();												// Od_d: derivative of desired rate
	_Od_prev = Od;

	Vector3f O_d = (O - _O_prev) / getDt();													// O_d: derivative of the rate TODO: estimate angular acceleration
	_O_prev = O;

	Vector3f e_M = J * (O.hat()*R.T()*Rd*Od /*- R.T()*Rd*Od_d*/);							// e_M : Model feed forward	= J * (O^^*R'*Rd*Od - R'*Rd*Od_d)

	Vector3f m;
	if(_simulation)
		m = -0.60f * e_O -2.0f * e_R /*+ e_C*/ /*- e_M*/;									// m   : angular moment to apply to quad
	else {
		m = -0.06f * e_O -0.2f * e_R /*-0.004f * O_d*/;
		m = m.emult(Vector3f(1,1,1));															// draft for using different gains depending on roll, pitch or yaw
	}

	publishMoment(m);
}

void BlockMControl::ControllerQ() {
	// ATTENTION: quaternions are defined differently than in the paper they are just complex conjugates/reverse rotations compared to the paper
	Eulerf euler(_joystick[0]/1.5f, _joystick[1]/1.5f, 0);				// joystick euler attitude set point TODO: better translation through vector
	Quatf qjoy(euler);

	Quatf q(_sub_vehicle_attitude.get().q); 							// q   : attitude					= estimated attitude from uORB topic
	Quatf qd(_sub_vehicle_attitude_setpoint.get().q_d);					// qd  : desired attitude			= desired attitude from uORB topic
	qd = qjoy;

	if(_joystick[3] < 0)
		qd = qd * Quatf(0,1,0,0);

	Dcmf R(q);															// reduced attitude control (only roll and pitch because they are much faster)
	Dcmf Rd(qd);														// get only the z unit vectors by converting to rotation matrices and taking the last column
	Vector3f Rz(R(0, 2), R(1, 2), R(2, 2));
	Vector3f Rdz(Rd(0, 2), Rd(1, 2), Rd(2, 2));
	float alpha = acosf(Rz.dot(Rdz));									// get the angle between them
	Vector3f axis = Rz % Rdz;											// and the axis to rotate from one to the other
	axis.normalize();
	Vector3f qered13 = sinf(alpha/2) * axis;
	Quatf qered(cos(alpha/2), qered13(0), qered13(1), qered13(2));		// build up the quaternion that does this rotation
	Quatf qdred = q * qered;											// qdred: reduced desired attitude	= desired attitude from uORB topic

	float p = 0.4f;
	Quatf qmix = qdred.inversed() * qd;									// mixing reduced and full attitude control
	qmix *= sign(qmix(0));												// take care of the ambiguity of a quaternion
	float alphahalf = asinf(qmix(3));
	qmix(0) = cosf(p*alphahalf);
	qmix(1) = qmix(2) = 0;
	qmix(3) = sinf(p*alphahalf);
	Quatf qcmd = qdred * qmix;

	Quatf qe = q * qcmd.inversed();										// qe : attitude error		= qd^-1 * q
	Vector3f e_R = 2.f * sign(qe(0)) * Vector3f(qe(1),qe(2),qe(3));		// take care of the ambiguity of a quaternion

	Vector3f O(&_sub_control_state.get().roll_rate);					// O   : rate				= gyroscope measurement from uORB topic
	Vector3f e_O = O - Vector3f(0,0,_joystick[2]*2);					// e_O : rate error

	Vector3f O_d = (O - _O_prev) / getDt();								// O_d: derivative of the rate TODO: estimate angular acceleration
	_O_prev = O;

	Vector3f m;
	if(_simulation)
		m = -0.60f * e_O -2.0f * e_R;									// m   : angular moment to apply to quad
	else {
		m = -0.06f * e_O -0.2f * e_R /*-0.004f * O_d*/;
		m = m.emult(Vector3f(1,1,1));									// draft for using different gains depending on roll, pitch or yaw
	}

	publishMoment(m);
}

void BlockMControl::rateController_original() {
	math::Vector<3> rates = {_sub_control_state.get().roll_rate, _sub_control_state.get().pitch_rate, _sub_control_state.get().yaw_rate};
	math::Vector<3> rates_sp = {_joystick[0], _joystick[1], _joystick[2]};

	math::Vector<3> _rate_p;
	if(_simulation)
		_rate_p = {0.48f, 0.48f, 0.15f};
	else
		_rate_p = {0.1f, 0.1f, 0.1f};

	math::Vector<3> _rate_d = {0.002f, 0.002f, 0.0f};

	math::Vector<3> rates_err = rates_sp*2 - rates;
	math::Vector<3> _control_output = _rate_p.emult(rates_err) + _rate_d.emult(_rates_prev - rates) / getDt();
	_rates_prev = rates;

	publishMoment(Vector3f(_control_output.data));
}

void BlockMControl::publishMoment(matrix::Vector3f moment) {
	float thrust = _sub_vehicle_attitude_setpoint.get().thrust;
	thrust = _joystick[3]*1.4f-0.4f;

	for(int i = 0; i < 3; i++)
		_pub_actuator_controls.get().control[i] = PX4_ISFINITE(moment(i)) ? moment(i) : 0;
	_pub_actuator_controls.get().control[3] = PX4_ISFINITE(thrust) && thrust > 0 ? thrust : 0;
	_pub_actuator_controls.update();
}
