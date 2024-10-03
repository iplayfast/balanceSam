/*
In a balance bot, PID values control how the bot stays upright:

    P (Proportional): This part makes the bot correct itself based on how far it’s tilted. If it’s tilting a lot, the motors work harder to bring it back up.
    I (Integral): This helps fix tiny errors over time. If the bot keeps leaning slowly, it adjusts to stop that slow drift.
    D (Derivative): This part reacts to how fast the bot is tilting, helping to prevent overcorrecting or wobbling.
    
    To adjust the PID for a balance bot, follow these steps:

    Set I and D to zero: Start with only the P term.
    Tune P (Proportional): Gradually increase Kp until the bot starts to balance but oscillates.
    Tune D (Derivative): Add Kd to dampen the oscillations. Adjust until it stabilizes without much wobble.
    Tune I (Integral): Slowly increase Ki to correct for steady-state errors (like drifting or leaning over time).
    
    */

#include <Arduino.h>
#include <Wire.h>
#include "FastIMU.h"
#include "Madgwick.h"
#include <PID_v1.h>
#include <SoftwareSerial.h>
#include "hoverserial.h"

#define DEBUG_HOVER   // will serial.print the feedback struct

// Define pins for USART Communication
#define USART_RX 9
#define USART_TX 8
#define TIME_SEND           5         // [ms] Sending time interval

// Maximum Hoverboard Speed (0-1000)
#define SPEED_MAX      500       // [-] Maximum speed for testing

SoftwareSerial oSerialHover(USART_RX, USART_TX); // USART communication
SerialFeedback oHoverFeedback;

// Define pins for motor control
const int MOTOR_FORWARD_PIN = 5;  // PWM pin for forward motion
const int MOTOR_BACKWARD_PIN = 6; // PWM pin for backward motion

#define IMU_ADDRESS 0x68    // Change to the address of the IMU
#define PERFORM_CALIBRATION // Comment to disable startup calibration
MPU6500 IMU;                // Change to the name of your specific IMU if different
calData calib = { 0 };      // Calibration data
AccelData IMUAccel;         // Sensor data
GyroData IMUGyro;
MagData IMUMag;
Madgwick filter;

// Variables to store quaternion data
float qw, qx, qy, qz;

// PID variables
double input, output;
double setpoint = 4;

//Best Stand Still
double Kp = 6.5, Ki = 0.04, Kd = 0.3; // You'll need to tune these values
//double Kp = 5.5, Ki = 0.0, Kd = 0.1; // You'll need to tune these values
//double Kp = 7.0, Ki = 0.10, Kd = 0.3; // You'll need to tune these values
//double Kp = 4, Ki = 0.9, Kd = 0.35; // You'll need to tune these values
//double Kp = 5, Ki = 0.1, Kd = 0.05; // You'll need to tune these values

PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Define the maximum output for the PID controller
const int PID_OUTPUT_LIMIT = 255;

void setupPID() {
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
  myPID.SetSampleTime(10); // 10ms sample time, matching your current loop delay
}

// Function to convert quaternion to pitch angle
float quaternionToPitch(float qw, float qx, float qy, float qz) {
  return atan2(2 * (qw * qy - qz * qx), 1 - 2 * (qx * qx + qy * qy)) * RAD_TO_DEG;
}

void processSerialInput() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    char command = input.charAt(0);
    float value = input.substring(2).toFloat();

    switch (command) {
      case 'P':
      case 'p':
        Kp = value;
        myPID.SetTunings(Kp, Ki, Kd);
        Serial.println("Updated Kp: " + String(Kp));
        break;
      case 'I':
      case 'i':
        Ki = value;
        myPID.SetTunings(Kp, Ki, Kd);
        Serial.println("Updated Ki: " + String(Ki));
        break;
      case 'D':
      case 'd':
        Kd = value;
        myPID.SetTunings(Kp, Ki, Kd);
        Serial.println("Updated Kd: " + String(Kd));
        break;
      case 'S':
      case 's':
        setpoint = value;
        Serial.println("Updated Setpoint: " + String(setpoint));
        break;
      default:
        Serial.println("Invalid command. Use P, I, D, or S followed by a value.");
    }
  }
}

void TalkToHoverCPU(float pitch) {
  int mappedPitch = map(pitch, PID_OUTPUT_LIMIT, -PID_OUTPUT_LIMIT, -1000, 1000);
  Serial.print("Mapped Pitch: ");
  Serial.println(mappedPitch);
  Send(oSerialHover, 0, mappedPitch);
}

void adjustBalancePID(float pitch) {
  input = pitch;
  myPID.Compute();

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

void setup() {
  Wire.begin();
  Wire.setClock(400000); // 400khz clock

  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  SetupHoverArduino(oSerialHover, 19200);  // 8 Mhz Arduino Mini too slow for 115200 !!!

  pinMode(MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(MOTOR_BACKWARD_PIN, OUTPUT);

  Send(oSerialHover, 0, 0);

  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }

#ifdef PERFORM_CALIBRATION
  // TODO
#endif

  filter.begin(0.2f);
  setupPID();
  while(Serial.available())
    Serial.read();
  Serial.println("Setup complete");
}

void loop() {
  processSerialInput();  // Add this line to check for serial input each loop

  static unsigned long lastLoopTime = 0;
  const unsigned long loopInterval = 2; // 10ms loop interval for 100Hz update rate

  if (micros() - lastLoopTime >= loopInterval * 1000) {
    lastLoopTime = micros();

    // Update IMU data
    IMU.update();
    IMU.getAccel(&IMUAccel);
    IMU.getGyro(&IMUGyro);

    // Update Madgwick filter
    if (IMU.hasMagnetometer()) {
      IMU.getMag(&IMUMag);
      filter.update(IMUGyro.gyroX, IMUGyro.gyroY, IMUGyro.gyroZ, 
                    IMUAccel.accelX, IMUAccel.accelY, IMUAccel.accelZ, 
                    IMUMag.magX, IMUMag.magY, IMUMag.magZ);
    } else {
      filter.updateIMU(IMUGyro.gyroX, IMUGyro.gyroY, IMUGyro.gyroZ, 
                       IMUAccel.accelX, IMUAccel.accelY, IMUAccel.accelZ);
    }

    // Get quaternion data
    qw = filter.getQuatW();
    qx = filter.getQuatX();
    qy = filter.getQuatY();
    qz = filter.getQuatZ();

    // Calculate pitch
    float pitch = quaternionToPitch(qw, qx, qy, qz);

    // Apply PID control for balance
    adjustBalancePID(pitch);

    // Optional: Print debug information
    Serial.print("Kp: "); Serial.print(Kp,4);
    Serial.print(" Ki: "); Serial.print(Ki,4);
    Serial.print(" Kd: "); Serial.print(Kd,4);
    Serial.print(" Setpoint: "); Serial.print(setpoint,4);
    Serial.print(" Pitch: "); Serial.print(pitch); Serial.print(" ");
  }
}
