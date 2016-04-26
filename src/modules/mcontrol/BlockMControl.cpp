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
	Eulerf euler(_joystick[0]/1.5f, _joystick[1]/1.5f, 0/*_joystick[2]*1.5f*/);				// joystick euler attitude set point TODO: better translation through vector
	if(_joystick[3] < 0) {
		_qd = /*Quatf(euler) **/ Quatf(0,1,0,0);	// flip
		float r[] = {1,0,0, 0,-1,0, 0,0,-1};
		_Rd = Matrix3f(r);
	} else {
		_qd = Quatf(euler);
		_Rd = Dcmf(euler);
	}

	Vector3f F(&_sub_force_setpoint.get().x);
	/*_Rd = FtoR(F, _sub_force_setpoint.get().yaw_rate);
	_qd = Quatf(_Rd);*/
	_qd = FtoQ(F, _sub_force_setpoint.get().yaw_rate);
	_qd.print();

	Vector3f e_R;
	if(1)
		e_R = ControllerQ();
	else
		e_R = ControllerR();

	Vector3f O(&_sub_control_state.get().roll_rate);									// O   : rate				= gyroscope measurement from uORB topic
	Vector3f e_O = O - Vector3f(0,0,0/*_joystick[2]*2*/);									// e_O : rate error

	Vector3f O_d = (O - _O_prev) / getDt();												// O_d: derivative of the rate TODO: estimate angular acceleration
	_O_prev = O;

	Vector3f m;
	if(_simulation)
		m = -0.60f * e_O -2.0f * e_R -0.004f * O_d;										// m   : angular moment to apply to quad
	else {
		m = -0.06f * e_O -0.2f * e_R -0.004f * O_d;
		m = m.emult(Vector3f(1,1,1));															// draft for using different gains depending on roll, pitch or yaw
	}

	float thrust = _sub_vehicle_attitude_setpoint.get().thrust;
	thrust = _joystick[3]*1.4f-0.4f;
	thrust = F.norm()*1.4f-0.4f;	// TODO: throttle should be smaller if we are not aligned with attitude yet
	publishMoment(m, thrust);
}

Vector3f BlockMControl::ControllerR() {
	Matrix3f R(_sub_vehicle_attitude.get().R); 											// R   : attitude			= estimated attitude from uORB topic
	Matrix3f Rd(_sub_vehicle_attitude_setpoint.get().R_body);							// Rd  : desired attitude	= desired attitude from uORB topic
	Rd = _Rd;	// TODO: set point override have to be taken out!

	Vector3f R_z(R(0, 2), R(1, 2), R(2, 2));		// reduced attitude control
	Vector3f Rd_z(Rd(0, 2), Rd(1, 2), Rd(2, 2));
	return R.T() * (Rd_z % R_z);
	//return 1/2.f * ((Dcmf)(Rd.T() * R - R.T() * Rd)).vee();				// e_R : attitude error		= 1/2 * (Rd' R - R' * Rd)^V
}

Vector3f BlockMControl::ControllerQ() {									// ATTENTION: quaternions are defined differently than in the paper they are complex conjugates/reverse rotations compared to the paper
	Quatf q(_sub_vehicle_attitude.get().q); 							// q   : attitude					= estimated attitude from uORB topic
	Quatf qd(_sub_vehicle_attitude_setpoint.get().q_d);					// qd  : desired attitude			= desired attitude from uORB topic
	qd = _qd;	// TODO: set point override have to be taken out!

	Quatf qe = q * qd.inversed();										// full quaternion attitude control
	if(qe(0) < 0) {														// qe : attitude error				= rotation from the actual attitude to the desired attitude
		qe *= -1;														// take care of the ambiguity of a quaternion
		qd = qe.inversed() * q;											// this step is needed to have a corrected unambiguous qd for the mix of reduced and full attitude later
	}

	Dcmf R(q);															// reduced attitude control part (only roll and pitch because they are much faster)
	Dcmf Rd(qd);														// get only the z unit vectors by converting to rotation matrices and taking the last column
	Vector3f Rz(R(0, 2), R(1, 2), R(2, 2));
	Vector3f Rdz(Rd(0, 2), Rd(1, 2), Rd(2, 2));
	float alpha = acosf(Rz.dot(Rdz));									// get the angle between them
	Vector3f axis = Rz % Rdz;											// and the axis to rotate from one to the other
	axis.normalize();
	Vector3f qered13 = sinf(alpha/2.f) * axis;
	Quatf qered(cos(alpha/2.f), qered13(0), qered13(1), qered13(2));	// build up the quaternion that does this rotation
	Quatf qdred = q * qered;											// qdred: reduced desired attitude	= rotation that's needed to align the z axis but seen from the world frame

	float p = 0.4f;		// TODO: parameter!								// mixing reduced and full attitude control
	Quatf qmix = qdred.inversed()*qd;									// qmix	: the roatation from full attitude to reduced attitude correction -> only body yaw roatation -> qmix = [cos(alpha_mix/2) 0 0 sin(alpha_mix/2)]
	float alphahalf = asinf(qmix(3));									// calculate the angle alpha/2 that the full attitude controller would do more than the reduced in body yaw direction
	qmix(0) = cosf(p*alphahalf);										// reconstruct a quaternion that scales the body yaw rotation angle error by p = K_yaw / K_roll,pitch
	qmix(3) = sinf(p*alphahalf);
	Quatf qcmd = qdred * qmix;											// qcmd	: final mixed desired attitude for the controller

	qe = q * qcmd.inversed();
	return 2.f * sign(qe(0)) * Vector3f(qe(1),qe(2),qe(3));				// using the sin(alpha/2) scaled rotation axis as correction term plus taking care of the ambiguity of a quaternion
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

	publishMoment(Vector3f(_control_output.data), _joystick[3]*1.4f-0.4f);
}

Quatf BlockMControl::FtoQ(Vector3f F, float yaw) {
	if(F.norm() < 1e-4f)
		return Quatf();	// if no desired force stay level TODO: we can just stay how we are if no force desired
	F.normalize();

	Vector3f z(0,0,1);
	float alpha = acosf(z.dot(F));
	Vector3f axis = z % F;
	if(axis.norm() < 1e-6f)
		axis = Vector3f(1,0,0); // we want to be exactly level then the angle is anyways very small or exactly 180 flipped then the angle is pi
	else
		axis.normalize();
	Quatf qred;
	qred.from_axis_angle(axis, alpha);

	Quatf qyaw;
	qyaw.from_axis_angle(z, yaw);

	return qyaw * qred;
}

Dcmf BlockMControl::FtoR(Vector3f F, float yaw) {
	F.normalize();
	Vector3f x_C_des(cos(yaw), sin(yaw), 0);
	Vector3f y_B_des = F % x_C_des;
	y_B_des.normalize();
	Vector3f x_B_des = y_B_des % F;
	Dcmf R1;
	R1.set(x_B_des,0,0);
	R1.set(y_B_des,0,1);
	R1.set(F,0,2);
	return R1;
}

void BlockMControl::publishMoment(matrix::Vector3f moment, float thrust) {
	for(int i = 0; i < 3; i++)
		_pub_actuator_controls.get().control[i] = PX4_ISFINITE(moment(i)) ? moment(i) : 0;
	_pub_actuator_controls.get().control[3] = PX4_ISFINITE(thrust) && thrust > 0 ? thrust : 0;
	_pub_actuator_controls.update();
}
