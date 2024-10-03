#ifndef PID_H
#define PID_H

#include <chrono>
#include <cstdint>

class PID {
public:

    enum class Mode : uint8_t {
        MANUAL = 0,
        AUTOMATIC = 1
    };

    enum class Direction : uint8_t {
        DIRECT = 0,
        REVERSE = 1
    };

    enum class ProportionalMode : uint8_t {
        P_ON_M = 0,
        P_ON_E = 1
    };

    // Constructor
    PID(double *input, double *output, double *setpoint,
        double kp, double ki, double kd,
        ProportionalMode pMode = ProportionalMode::P_ON_E,
        Direction direction = Direction::DIRECT);

    // Core functions
    void setMode(Mode mode);
    bool compute();
    void setOutputLimits(double min, double max);

    // Tuning functions
    void setTunings(double kp, double ki, double kd);
    void setTunings(double kp, double ki, double kd, ProportionalMode pMode);

    // Configuration functions
    void setControllerDirection(Direction direction);
    void setSampleTime(std::chrono::milliseconds sampleTime);

    // Status functions
    double getKp() const { return dispKp; }
    double getKi() const { return dispKi; }
    double getKd() const { return dispKd; }
    Mode getMode() const { return mode; }
    Direction getDirection() const { return direction; }

private:
    void initialize();

    // Tuning parameters
    double dispKp, dispKi, dispKd;
    double kp, ki, kd;

    // Configuration
    Direction direction;
    ProportionalMode pMode;

    // Working variables
    double* input;
    double* output;
    double* setpoint;
    std::chrono::steady_clock::time_point lastTime;
    double outputSum, lastInput;
    std::chrono::milliseconds sampleTime;
    double outMin, outMax;
    bool inAuto;
    bool pOnE;
    Mode mode;
};

#endif // PID_H
