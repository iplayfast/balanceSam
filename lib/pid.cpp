#include "pid.h"
#include <algorithm>
#include <chrono>
PID::PID(double *input, double *output, double *setpoint,
         double kp, double ki, double kd,
         ProportionalMode pMode, Direction direction)
    : input(input)
    , output(output)
    , setpoint(setpoint)
    , dispKp(kp)
    , dispKi(ki)
    , dispKd(kd)
    , kp(kp)
    , ki(ki)
    , kd(kd)
    , mode(Mode::MANUAL)
    , direction(direction)
    , pMode(pMode)
    , lastTime(std::chrono::steady_clock::now() - std::chrono::milliseconds(100))
    , outputSum(0)
    , lastInput(0)
    , sampleTime(std::chrono::milliseconds(100))  // default 100ms sample time
    , outMin(0)
    , outMax(255)  // default output limits
    , inAuto(false)
    , pOnE(pMode == ProportionalMode::P_ON_E)
{
    setOutputLimits(0, 255);  // default output limits
    setControllerDirection(direction);
    setTunings(kp, ki, kd, pMode);
}

bool PID::compute() {
    if (!inAuto) return false;

    auto now = std::chrono::steady_clock::now();
    auto timeChange = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTime);

    if (timeChange >= sampleTime) {
        // Compute all the working error variables
        double error = *setpoint - *input;
        double dInput = *input - lastInput;
        outputSum += ki * error;

        // Add Proportional on Measurement, if P_ON_M is specified
        if (!pOnE) outputSum -= kp * dInput;

        outputSum = std::clamp(outputSum, outMin, outMax);

        // Add Proportional on Error, if P_ON_E is specified
        double output_1 = pOnE ? kp * error : 0;

        // Compute Rest of PID Output
        output_1 += outputSum - kd * dInput;

        output_1 = std::clamp(output_1, outMin, outMax);
        *output = output_1;

        // Remember some variables for next time
        lastInput = *input;
        lastTime = now;
        return true;
    }
    return false;
}

void PID::setTunings(double kp, double ki, double kd, ProportionalMode pMode) {
    if (kp < 0 || ki < 0 || kd < 0) return;

    this->pMode = pMode;
    pOnE = pMode == ProportionalMode::P_ON_E;

    dispKp = kp; dispKi = ki; dispKd = kd;

    double sampleTimeInSec = std::chrono::duration<double>(sampleTime).count();
    this->kp = kp;
    this->ki = ki * sampleTimeInSec;
    this->kd = kd / sampleTimeInSec;

    if (direction == Direction::REVERSE) {
        this->kp = -this->kp;
        this->ki = -this->ki;
        this->kd = -this->kd;
    }
}

void PID::setTunings(double kp, double ki, double kd) {
    setTunings(kp, ki, kd, pMode);
}

void PID::setSampleTime(std::chrono::milliseconds newSampleTime) {
    if (newSampleTime > std::chrono::milliseconds(0)) {
        double ratio = static_cast<double>(newSampleTime.count()) / sampleTime.count();
        ki *= ratio;
        kd /= ratio;
        sampleTime = newSampleTime;
    }
}

void PID::setOutputLimits(double min, double max) {
    if (min >= max) return;
    outMin = min;
    outMax = max;

    if (inAuto) {
        *output = std::clamp(*output, outMin, outMax);
        outputSum = std::clamp(outputSum, outMin, outMax);
    }
}

void PID::setMode(Mode mode) {
    bool newAuto = (mode == Mode::AUTOMATIC);
    if (newAuto && !inAuto) {
        initialize();
    }
    inAuto = newAuto;
}

void PID::initialize() {
    outputSum = *output;
    lastInput = *input;
    outputSum = std::clamp(outputSum, outMin, outMax);
}

void PID::setControllerDirection(Direction direction) {
    if (inAuto && direction != this->direction) {
        kp = -kp;
        ki = -ki;
        kd = -kd;
    }
    this->direction = direction;
}
