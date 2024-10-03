#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <cmath>
#include <wiringPi.h>
#include <wiringSerial.h>

#include "pid.h"
#include "rpi_hoverserial.h"
#include "imuquat.h"

// PID variables
double input, output;
double setpoint = 4;
const int PID_OUTPUT_LIMIT = 255;
const int MOTOR_BACKWARD_PIN = 6;
const int MOTOR_FORWARD_PIN = 5;


//Best Stand Still
double Kp = 6.5, Ki = 0.04, Kd = 0.3; // You'll need to tune these values
//double Kp = 5.5, Ki = 0.0, Kd = 0.1; // You'll need to tune these values
//double Kp = 7.0, Ki = 0.10, Kd = 0.3; // You'll need to tune these values
//double Kp = 4, Ki = 0.9, Kd = 0.35; // You'll need to tune these values
//double Kp = 5, Ki = 0.1, Kd = 0.05; // You'll need to tune these values

PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, PID::ProportionalMode::P_ON_E,PID::Direction::DIRECT);
RPiHoverSerial HSerial;
void setupPID() {
  myPID.setMode(PID::Mode::AUTOMATIC);
  myPID.setOutputLimits(-PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
  
  // Create a duration of 10 milliseconds
  std::chrono::milliseconds sampleTime(10);
  myPID.setSampleTime(sampleTime); // 10ms sample time, matching your current loop delay
}

void TalkToHoverCPU(float pitch) {
  int mappedPitch = map(pitch, PID_OUTPUT_LIMIT, -PID_OUTPUT_LIMIT, -1000, 1000);
  printf("Mapped Pitch: %04d\n" ,mappedPitch);
  HSerial.send(0, mappedPitch);
}
void adjustBalancePID(float pitch) {
	input = pitch;
	myPID.compute();

	// Map PID output to motor speed
	int motorSpeed = abs(output);
	if (output > 0) {
		// Leaning forward, adjust backwards
		analogWrite(MOTOR_BACKWARD_PIN, motorSpeed);
		analogWrite(MOTOR_FORWARD_PIN, 0);
	} else if (output < 0) {
		// Leaning backward, adjust forwards
		analogWrite(MOTOR_FORWARD_PIN, motorSpeed);
		analogWrite(MOTOR_BACKWARD_PIN, 0);
	} else {
		// Balanced, stop motors
		analogWrite(MOTOR_FORWARD_PIN, 0);
		analogWrite(MOTOR_BACKWARD_PIN, 0);
	}

	// Call your existing function to communicate with the Hover CPU
	TalkToHoverCPU(output);
} 

void adjust_balance_pid(float pitch) {
    input = pitch;
    myPID.compute();

    int motorSpeed = abs(output);
    
    if (output > 0) {
        pwmWrite(MOTOR_BACKWARD_PIN, motorSpeed);
        pwmWrite(MOTOR_FORWARD_PIN, 0);
    } else if (output < 0) {
        pwmWrite(MOTOR_FORWARD_PIN, motorSpeed);
        pwmWrite(MOTOR_BACKWARD_PIN, 0);
    } else {
        pwmWrite(MOTOR_FORWARD_PIN, 0);
        pwmWrite(MOTOR_BACKWARD_PIN, 0);
    }

    TalkToHoverCPU(output);
}


int main() {
	if (!imuSetup()) return 1;

	if (!HSerial.begin(115200,"dev/ttyAMA0")) return 1;

	setupPID();
	float pitch;
	while(1) {
	if (GetPitch(&pitch))
		adjust_balance_pid(pitch);	
	}
}
