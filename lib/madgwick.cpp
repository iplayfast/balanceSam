#include "madgwick.h"
void Madgwick::update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    now = std::chrono::steady_clock::now();
    delta_t = std::chrono::duration<float>(now - last_update).count();
    last_update = now;

    // Rest of the update function...
}
