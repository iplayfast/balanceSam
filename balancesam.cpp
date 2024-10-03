#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringSerial.h>
//#include "madgwick.h"

// You'll need to implement or find C++ equivalents for these
#include "pid.h"

#define DEBUG_HOVER
#include "rpi_hoverserial.h"
#include "imuquat.h"
#define TIME_SEND           5
#define SPEED_MAX_TEST      200

#define USART_BAUD_RATE 19200
#define IMU_ADDRESS 0x68
#define PERFORM_CALIBRATION

const int MOTOR_FORWARD_PIN = 5;
const int MOTOR_BACKWARD_PIN = 6;

int serialFd;
MPU6500 IMU;
calData calib = { 0 };
AccelData IMUAccel;
GyroData IMUGyro;
MagData IMUMag;
Madgwick filter;

float qw, qx, qy, qz;

double setpoint, input, output;
double Kp = 5, Ki = 0.8, Kd = 0.05;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

const int PID_OUTPUT_LIMIT = 255;

void setupPID() {
    setpoint = 2;
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
    myPID.SetSampleTime(10);
}

float quaternionToPitch(float qw, float qx, float qy, float qz) {
    return atan2(2 * (qw * qy - qz * qx), 1 - 2 * (qx * qx + qy * qy)) * 180.0 / M_PI;
}

void TalkToHoverCPU(float pitch) {
    int mappedPitch = map(pitch, PID_OUTPUT_LIMIT, -PID_OUTPUT_LIMIT, -1000, 1000);
    std::cout << "Mapped Pitch: " << mappedPitch << std::endl;
    Send(serialFd, 0, mappedPitch);
}

void adjustBalancePID(float pitch) {
    input = pitch;
    myPID.Compute();

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

void setup() {
    wiringPiSetup();
    
    serialFd = serialOpen("/dev/ttyAMA0", USART_BAUD_RATE);
    if (serialFd < 0) {
        std::cerr << "Unable to open serial device: " << strerror(errno) << std::endl;
        return;
    }

    pinMode(MOTOR_FORWARD_PIN, PWM_OUTPUT);
    pinMode(MOTOR_BACKWARD_PIN, PWM_OUTPUT);

    Send(serialFd, 0, 0);

    int err = IMU.init(calib, IMU_ADDRESS);
    if (err != 0) {
        std::cerr << "Error initializing IMU: " << err << std::endl;
        return;
    }

#ifdef PERFORM_CALIBRATION
    // ... (keep your existing calibration code)
#endif

    filter.begin(0.2f);
    setupPID();

    std::cout << "Setup complete" << std::endl;
    std::cout << "Enter PID values and setpoint in the format: P,I,D,S" << std::endl;
    std::cout << "For example: 5.0,0.8,0.05,2.0" << std::endl;
}

void processSerialInput() {
    if (serialDataAvail(serialFd)) {
        std::string input;
        char c;
        while ((c = serialGetchar(serialFd)) != '\n') {
            input += c;
        }
        
        size_t firstComma = input.find(',');
        size_t secondComma = input.find(',', firstComma + 1);
        size_t thirdComma = input.find(',', secondComma + 1);
        
        if (firstComma != std::string::npos && secondComma != std::string::npos && thirdComma != std::string::npos) {
            float newKp = std::stof(input.substr(0, firstComma));
            float newKi = std::stof(input.substr(firstComma + 1, secondComma - firstComma - 1));
            float newKd = std::stof(input.substr(secondComma + 1, thirdComma - secondComma - 1));
            float newSetpoint = std::stof(input.substr(thirdComma + 1));
            
            Kp = newKp;
            Ki = newKi;
            Kd = newKd;
            setpoint = newSetpoint;
            
            myPID.SetTunings(Kp, Ki, Kd);
            
            std::cout << "New PID values and setpoint set:" << std::endl;
            std::cout << "Kp: " << Kp << std::endl;
            std::cout << "Ki: " << Ki << std::endl;
            std::cout << "Kd: " << Kd << std::endl;
            std::cout << "Setpoint: " << setpoint << std::endl;
        } else {
            std::cout << "Invalid input format. Use P,I,D,S" << std::endl;
        }
    }
}

int main() {
    setup();

    auto lastLoopTime = std::chrono::steady_clock::now();
    const auto loopInterval = std::chrono::milliseconds(10);

    while (true) {
        auto now = std::chrono::steady_clock::now();
        if (now - lastLoopTime >= loopInterval) {
            lastLoopTime = now;

            IMU.update();
            IMU.getAccel(&IMUAccel);
            IMU.getGyro(&IMUGyro);

            if (IMU.hasMagnetometer()) {
                IMU.getMag(&IMUMag);
                filter.update(IMUGyro.gyroX, IMUGyro.gyroY, IMUGyro.gyroZ, 
                              IMUAccel.accelX, IMUAccel.accelY, IMUAccel.accelZ, 
                              IMUMag.magX, IMUMag.magY, IMUMag.magZ);
            } else {
                filter.updateIMU(IMUGyro.gyroX, IMUGyro.gyroY, IMUGyro.gyroZ, 
                                 IMUAccel.accelX, IMUAccel.accelY, IMUAccel.accelZ);
            }

            qw = filter.getQuatW();
            qx = filter.getQuatX();
            qy = filter.getQuatY();
            qz = filter.getQuatZ();

            float pitch = quaternionToPitch(qw, qx, qy, qz);

            adjustBalancePID(pitch);

            std::cout << "Pitch: " << pitch << " ";
        }

        processSerialInput();
    }

    return 0;
}
