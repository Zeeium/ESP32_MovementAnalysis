#ifndef FILTERS_H
#define FILTERS_H

#include "../Sensors/MPUInterface.h"

namespace Processing {

class ComplementaryFilter {
public:
    ComplementaryFilter(float alpha = 0.96f);
    
    // Reset orientation state
    void reset(float initialRoll = 0.0f, float initialPitch = 0.0f);
    
    // Update orientation with a new sample and time step
    void update(const SensorSample& sample, float dt);

    float getRoll() const { return _roll; }
    float getPitch() const { return _pitch; }
    float getRelativeYaw() const { return _relativeYaw; }
    float getYawRate() const { return _yawRate; }

private:
    float _alpha;
    float _roll;      // degrees
    float _pitch;     // degrees
    float _relativeYaw; // degrees (drifts over time)
    float _yawRate;   // rad/s (raw z gyro)
};

} // namespace Processing

#endif // FILTERS_H
