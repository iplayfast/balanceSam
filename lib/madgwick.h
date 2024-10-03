#ifndef __Madgwick_h__
#define __Madgwick_h__

#include <cmath>
#include <chrono>
#include <cstdint>

class Madgwick {
private:
    std::chrono::steady_clock::time_point now;
    std::chrono::steady_clock::time_point last_update;
    static float invSqrt(float x);
    float beta;             // algorithm gain
    float q0, q1, q2, q3;   // quaternion of sensor frame relative to auxiliary frame

public:
    float delta_t = 0; // Used to control display output rate

    Madgwick(void);
    void changeBeta(float newBeta) { beta = newBeta; }
    void begin(float confBeta) { beta = confBeta; }
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);

    float getQuatW() const { return q0; }
    float getQuatX() const { return q1; }
    float getQuatY() const { return q2; }
    float getQuatZ() const { return q3; }
};

#endif // __Madgwick_h__
