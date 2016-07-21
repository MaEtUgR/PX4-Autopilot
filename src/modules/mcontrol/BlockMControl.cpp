/**
 * @file BlockMControl.cpp
 * @author Matthias Grob <maetugr@gmail.com>
 *
 * Nonlinear Quadrotor Attitude Control and Estimation (master thesis)
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
		_sub_actuator_armed(ORB_ID(actuator_armed), 0, 0, &getSubscriptions()),
		_sub_actuator_outputs(ORB_ID(actuator_outputs), 0, 0, &getSubscriptions()),
		_pub_actuator_outputs(ORB_ID(actuator_outputs), -1, &getPublications()),
		_pub_actuator_controls(ORB_ID(actuator_controls_0), -1, &getPublications()),
		_sub_sensor_combined(ORB_ID(sensor_combined), 0, 0, &getSubscriptions()),
		_dt_timeStamp(hrt_absolute_time()),
		_joystick{0,0,0,0}
{
	_control_state_Poll.fd = _sub_control_state.getHandle();
	_control_state_Poll.events = POLLIN;
	_yaw = 0;
	_estimator_inited = false;

	const char *dev = PWM_OUTPUT0_DEVICE_PATH;	// file descriptor to command PWM set points
	_pwm_fd = open(dev, 0);
	if (_pwm_fd < 0) err(1, "can't open %s", dev);

	_simulation = simulation;					// TODO: distinction between simulation and reality should not be needed, inputs have to get the same and parameters have to be set differently
}

BlockMControl::~BlockMControl() {
	if (_pwm_fd != -1) px4_close(_pwm_fd);		// close file descriptor again
	publishMoment(Vector3f(0,0,0),0);			// send a last actuator control output to the px4io otherwise it goes to failsafe mode and the motors start spinning!
}

void BlockMControl::update() {
	if(!poll_control_state()) return;		// poll on attitude topic to synchronize the loop to new data
	updateSubscriptions();
	calculate_dt();
	get_joystick_data();

	Estimator();
	Controller();
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

bool BlockMControl::EstimatorInit(Vector3f A, Vector3f M) {
	// Quaternion initialisation
	if(!(A.norm() > 0) || !(M.norm() > 0)) return false;
	A.normalize();		// we need a rotation matrix with determinant 1 so we can already normalize the input vectors
	M.normalize();

	// Rotation matrix can be easily constructed from acceleration and mag field vectors
	Vector3f k = -A;	// 'k' is Earth Z axis (Down) unit vector in body frame
	Vector3f j = k % M;	// 'j' is Earth Y axis (East) unit vector in body frame, orthogonal with 'k' and 'M'
	j.normalize();		// A and M are not orthogonal -> we still need to normalize here
	Vector3f i = j % k;	// 'i' is Earth X axis (North) unit vector in body frame, orthogonal with 'j' and 'k'

	Dcmf R;				// Fill rotation matrix
	R.setCol(0, i);
	R.setCol(1, j);
	R.setCol(2, k);

	_q = Quatf(R.T());	// Convert to quaternion

	// QEKF additional initialisation
	_P = eye<float, 6>();

	return true;
}

void BlockMControl::Estimator() {
	Vector3f O(_sub_sensor_combined.get().gyro_rad_s);
	//Vector3f O2(&_sub_control_state.get().roll_rate); // why is this different?
	Vector3f A(_sub_sensor_combined.get().accelerometer_m_s2);
	Vector3f M(_sub_sensor_combined.get().magnetometer_ga);

	if(!_estimator_inited) _estimator_inited = EstimatorInit(A,M);

	//Vector3f M_ref(0.1250, 0, 0.3980);
	Vector3f M_ref(0.3162, 0, 0.9485);			// reference magnetic vector in world frame (AIT lab)
	M_ref.normalize();
	Vector3f M_ref_body = _q.apply(M_ref);		// reference magnetic vector rotated to body frame TODO: q.apply is against matrix library conventions?!
	if(M.norm() > 1e-6f && M.norm() < .6f)		// if the mesurement exceeds normal magnetic field amplitudes it has to be a disturbance and we neglect it
		M.normalize();
	else
		M.setZero();
	M_ref_body.normalize();
	Vector3f wM = M % M_ref_body;				// residuum/error state measurement of magnetometer

	Vector3f A_ref(0, 0, -9.81); 				// reference acceleration vector in world frame when NOT moving! (AIT lab)
	A_ref.normalize();
	Vector3f A_ref_body = _q.apply(A_ref);		// reference acceleration vector rotated to body frame
	if(A.norm() > 1e-6f) A.normalize();
	A_ref_body.normalize();
	Vector3f wA = A % A_ref_body;				// residuum/error state measurement of accelerometer

	if(1) {	// the QEKF (QuaternionEKF) approach
		Quatf qrot;
		qrot.from_axis_angle((O - _b) * getDt()); // gyro measurement in dynamic model replacement mode
		_q = qrot * _q;							// state prediction (note that the gyro bias is predicted to stay the same)

		SquareMatrix<float,6> A;				// continuous system matrix
		A.set(-O.hat(),0,0);
		A.set(-eye<float,3>(),0,3);
		A = eye<float,6>() + A * getDt();		// from here on A is the first order approximated discrete system transition matrix

		float Q_diag[] = {1,1,1,1e-4,1e-4,1e-4};
		SquareMatrix<float,6> Q = diag(Vector<float,6>(Q_diag));
		_P = A*_P*A.T() + Q;

		SquareMatrix<float,6> H;				// measurement sensitivity matrix
		H.set(eye<float,3>(),0,0);
		H.set(eye<float,3>(),3,0);
		Vector<float,6> r;						// residual/error state measurement vector
		r.set(wM,0,0);							// first three measurements error from magnetometer vector
		r.set(wA,3,0);							// second three measurements error from accelerometer vector

		float R_diag[] = {10000,10000,10000,10000,10000,10000};			// error state measurement update
		SquareMatrix<float,6> R = diag(Vector<float,6>(R_diag));
		if(_sub_actuator_armed.get().armed) {
			R(3,3) = R(4,4) = R(5,5) = 100000;
		}
		SquareMatrix<float,6> S = H*_P*H.T() + R;
		SquareMatrix<float,6> K = _P*H.T()*S.I();
		Vector<float,6> xe = K*r;				// xe: the error state itself

		qrot.from_axis_angle(xe.slice<3,1>(0,0)); // update the full state with the error state
		_q = qrot * _q;
		_b += xe.slice<3, 1>(3,0);

		_P = ( (eye<float,6>() - K*H) * _P * (eye<float,6>() - K*H).T() )     +      K*R*K.T(); // update the covariance matrix

	} else { // a conplementary filter approach for the estimator
		Quatf qrot;
		qrot.from_axis_angle((O+wM+wA) * getDt()); // mixing the gyro prediction and the corrections with the fixed vectors
		_q = qrot * _q;
	}
	_q.normalize();

	for(int i = 0; i < 4; i++)	// log estimated quaternion to compare with vicon
		_pub_actuator_outputs.get().output[4+i] = _q(i);

	Quatf q(_sub_vehicle_attitude.get().q);	// TODO: for debugging output purpouse only
	Dcmf R(_sub_vehicle_attitude.get().R);

	static int counter = 0;
	counter++;
	if (0 && counter % 40 == 0) {
		printf("%.3f,%.3f,%.3f,%.3f,", (double)_q(0), (double)_q(1),(double)_q(2),(double)_q(3));
		printf("%.3f,%.3f,%.3f,%.3f,", (double)q(0), (double)q(1),(double)q(2),(double)q(3));
		//printf("%.3f,%.3f,%.3f,", (double)O(0), (double)O(1),(double)O(2));
		//printf("%.3f,%.3f,%.3f\n", (double)M_d(0), (double)M_d(1),(double)M_d(2));
		printf("%.3f,%.3f,%.3f,", (double)M_ref_body(0), (double)M_ref_body(1),(double)M_ref_body(2));
		printf("%.3f,%.3f,%.3f\n", (double)M(0), (double)M(1),(double)M(2));
		//printf("%.3f,%.3f,%.3f\n", (double)M(0), (double)M(1),(double)M(2));
		//printf("%.3f,%.3f,%.3f,%.3f,", (double)q(0), (double)q(1),(double)q(2),(double)q(3));
	}
}

void BlockMControl::Controller() {
	Eulerf euler(_joystick[0]/1.5f, _joystick[1]/1.5f, 0/*_joystick[2]*1.5f*/);				// joystick euler attitude set point TODO: better translation through vector
	if(_simulation && _joystick[3] < 0) {
		_qd = Quatf(0,1,0,0);	// flip
	} else {
		_qd = Quatf(euler);
	}

	Vector3f F(&_sub_force_setpoint.get().x);
	if(_simulation) {
		_yaw += _sub_force_setpoint.get().yaw_rate * getDt();
		_qd = FtoQ(F, _yaw);
	}

	Quatf q(_sub_vehicle_attitude.get().q); 							// q   : attitude					= estimated attitude from uORB topic
	Quatf qd(_sub_vehicle_attitude_setpoint.get().q_d);					// qd  : desired attitude			= desired attitude from uORB topic
	q = _q;				// our own attitude from the estimator this app
	if (_simulation)
		qd = _qd;
	Vector3f e_Q = ControllerQ(q, qd);

	Vector3f O(&_sub_control_state.get().roll_rate);									// O   : rate				= gyroscope measurement from uORB topic
	//Vector3f e_O = O - Vector3f(_joystick[0],_joystick[1],_joystick[2])*2;	// for rate mode
	Vector3f e_O = O - Vector3f(0,0,0);													// e_O : rate error

	Vector3f O_d = (O - _O_prev) / getDt();												// O_d: derivative of the rate TODO: estimate angular acceleration
	_O_prev = O;

	Vector3f m;
	if(_simulation)
		m = -0.60f * e_O -3.0f * e_Q -0.004f * O_d;										// m   : angular moment to apply to quad
	else {
		m = -0.097f * e_O -0.2f * e_Q -0.002f * O_d;
	}

	float thrust = _sub_vehicle_attitude_setpoint.get().thrust;
	thrust = _joystick[3];
	if(_simulation) {
		thrust = _joystick[3]*1.4f-0.4f;
		thrust = F.norm()*1.4f-0.4f;	// TODO: throttle should be smaller if we are not aligned with attitude yet
	}

	publishMoment(m, thrust);
}

Vector3f BlockMControl::ControllerQ(Quatf q, Quatf qd) {				// q : known attitude qd : desired attitude
	if(q.norm() > 1e-6f) q.normalize();									// this is very important because if the quaternions are not exactly normalized we could take acosf(1.00001) and that is NaN
	if(qd.norm() > 1e-6f) qd.normalize();
	// ATTENTION: quaternions are defined differently than in the paper they are complex conjugates/reverse rotations compared to the paper
	Quatf qe = q * qd.inversed();										// full quaternion attitude control
	if(qe(0) < 0) {														// qe : attitude error				= rotation from the actual attitude to the desired attitude
		qe *= -1;														// take care of the ambiguity of a quaternion
		qd = qe.inversed() * q;											// this step is needed to have a corrected unambiguous qd for the mix of reduced and full attitude later
	}

	Dcmf R(q);															// reduced attitude control part (only roll and pitch because they are much faster)
	Dcmf Rd(qd);														// get only the z unit vectors by converting to rotation matrices and taking the last column
	Vector3f Rz(R(0, 2), R(1, 2), R(2, 2));
	Vector3f Rdz(Rd(0, 2), Rd(1, 2), Rd(2, 2));
	float alpha = acosf(Rz.dot(Rdz));									// get the angle between them (beware acos(>1 or < -1)!, that's why we normalize the quaternions in the beginning)
	Vector3f axis = Rz % Rdz;											// and the axis to rotate from one to the other
	if(axis.norm() > 1e-6f) axis.normalize();
	Vector3f qered13 = sinf(alpha/2.f) * axis;
	Quatf qered(cos(alpha/2.f), qered13(0), qered13(1), qered13(2));	// build up the quaternion that does this rotation
	Quatf qdred = q * qered;											// qdred: reduced desired attitude	= rotation that's needed to align the z axis but seen from the world frame

	float p = 1.f;		// TODO: parameter for this!					// mixing reduced and full attitude control
	Quatf qmix = qdred.inversed()*qd;									// qmix	: the roatation from full attitude to reduced attitude correction -> only body yaw roatation -> qmix = [cos(alpha_mix/2) 0 0 sin(alpha_mix/2)]
	float alphahalf = asinf(qmix(3));									// calculate the angle alpha/2 that the full attitude controller would do more than the reduced in body yaw direction
	qmix(0) = cosf(p*alphahalf);										// reconstruct a quaternion that scales the body yaw rotation angle error by p = K_yaw / K_roll,pitch
	qmix(3) = sinf(p*alphahalf);
	Quatf qcmd = qdred * qmix;											// qcmd	: final mixed desired attitude for the controller

	qe = q * qcmd.inversed();
	return 2.f * sign(qe(0)) * Vector3f(qe(1),qe(2),qe(3));				// using the sin(alpha/2) scaled rotation axis as correction term plus taking care of the ambiguity of a quaternion
}

Quatf BlockMControl::FtoQ(Vector3f F, float yaw) {
	if(F.norm() < 1e-4f)
		return _q;						// stay in attitude we already have if no force desired
	F.normalize();

	Vector3f z(0,0,1);
	float alpha = acosf(z.dot(F));
	Vector3f axis = z % F;
	if(axis.norm() < 1e-6f)
		axis = Vector3f(1,0,0);			// we want to be exactly level then the angle is anyways very small or exactly 180 flipped then the angle is pi and we can choose the rotation axis (we roll here)
	else
		axis.normalize();
	Quatf qred;
	qred.from_axis_angle(axis, alpha);	// smallest rotation to get to the force direction

	Quatf qyaw;
	qyaw.from_axis_angle(z, yaw);		// rotate yaw radians around the z axis

	return qyaw * qred;
}

void BlockMControl::publishMoment(Vector3f moment, float thrust) {
	if(_simulation) {	// normal output with pixhawk mixer
		for(int i = 0; i < 3; i++)
			_pub_actuator_controls.get().control[i] = PX4_ISFINITE(moment(i)) ? moment(i) : 0;
		_pub_actuator_controls.get().control[3] = PX4_ISFINITE(thrust) && thrust > 0 ? thrust : 0;
		_pub_actuator_controls.update();
	} else {	// my own mixer with direct access to the motors
		Vector<float,4> moment_thrust;
		moment_thrust(0) = moment(0);	// filling the vector that contains all commands including thrust
		moment_thrust(1) = moment(1);
		moment_thrust(2) = moment(2);
		moment_thrust(3) = thrust;
		Mixer(moment_thrust);
	}

}

void BlockMControl::Mixer(Vector<float,4> moment_thrust) {
	float dh = 195, dv = 155, dd = 249;
	float Mixa[16] ={-dv/dh,+1,+dv/dd,+1,
					 +dv/dh,-1,+dv/dd,+1,
					 +dv/dh,+1,-dv/dd,+1,
					 -dv/dh,-1,-dv/dd,+1};
	SquareMatrix<float,4> Mix(Mixa);
	if(_sub_actuator_armed.get().armed) {	// check for system to be armed for safety reasons
		_motors = Mix*moment_thrust;
		//for(int i = 0; i < 4; i++)
			//_motors(i) = sqrtf(_motors(i));	// TODO: keep in mind the square root (control gains didn't really change with this)
	} else {
		for(int i = 0; i < 4; i++)
			_motors(i) = 0;
	}
	PWM();		// get the mixed values out to the motors
}

void BlockMControl::PWM() {
	for(int i = 0; i < 4; i++)
		setMotorPWM(i, _motors(i));
	_pub_actuator_outputs.update();
}

void BlockMControl::setMotorPWM(int channel, float w) {
	if(w < 0 || !PX4_ISFINITE(w)) w = 0;											// constrain the output to 0-100%
	if(w > 1.0f) w = 1;
	int min = 1000, max = 2000;
	int value = min + (w * (max-min));
	int ret = ioctl(_pwm_fd, PWM_SERVO_SET(channel), value);
	if (ret != OK) printf("Error: PWM_SERVO_SET(%d)\n", channel);
	_pub_actuator_outputs.get().output[channel] = (float)value;
}
