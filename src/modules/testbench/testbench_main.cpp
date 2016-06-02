/**
 * @file testbench_main.cpp
 * @author Matthias Grob <maetugr@gmail.com>
 *
 * a test bench that alows to throttle up motors for thrust testing
 */

#include <px4_posix.h>
#include "drivers/drv_pwm_output.h"

extern "C" __EXPORT int testbench_main(int argc, char *argv[]);	// deamon management function
void setMotorPWM(int channel, float power);

int _pwm_fd = -1;

void setMotorPWM(int channel, float power) {
	if(power < 0) power = 0;
	if(power > 1.0f) power = 1;
	int min = 1000, max = 2000;
	int value = min + (power * (max-min));
	int ret = ioctl(_pwm_fd, PWM_SERVO_SET(channel), value);
	if (ret != OK) printf("Error: PWM_SERVO_SET(%d)\n", channel);
}

int testbench_main(int argc, char *argv[]) {
	printf("Welcome to the motor test bench!\n");

	const char *dev = PWM_OUTPUT0_DEVICE_PATH;	// file descriptor to command PWM set points
	_pwm_fd = open(dev, 0);
	if (_pwm_fd < 0) err(1, "can't open %s", dev);

	pollfd fds = {0, 0, POLLIN, 0, 0}; 			// first member fd = 0 -> stdin

	float thrust[] = {0,0,0,0};
	float step = 1.0f / 20;
	char command = 0;

	while(command != ' ') {
		int ret = poll(&fds, 1, 0);
		if (ret > 0) {							// a new button was pressed
			read(0, &command, 1);
			switch(command) {
				case 'g':
					thrust[0] += step;
					break;
				case 'v':
					thrust[0] -= step;
					break;
				case 'h':
					thrust[1] += step;
					break;
				case 'b':
					thrust[1] -= step;
					break;
				case 'j':
					thrust[2] += step;
					break;
				case 'n':
					thrust[2] -= step;
					break;
				case 'k':
					thrust[3] += step;
					break;
				case 'm':
					thrust[3] -= step;
					break;
				default:
					command = ' ';
			}
		}
		if(command == ' ')
			for(int i = 0; i < 4; i++)
				thrust[i] = 0;
		printf("Command: %c Values: %.3f, %.3f, %.3f, %.3f\n", command, (double)thrust[0], (double)thrust[1], (double)thrust[2], (double)thrust[3]);
		for(int i = 0; i < 4; i++)
			setMotorPWM(i, thrust[i]);
		usleep(20000);
	}
	return 0;
}
