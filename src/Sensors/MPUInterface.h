#ifndef MPU_INTERFACE_H
#define MPU_INTERFACE_H

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Raw sample container representing a single point in time-series
struct SensorSample {
    float accelX; // m/s^2
    float accelY; // m/s^2
    float accelZ; // m/s^2
    float gyroX;  // rad/s
    float gyroY;  // rad/s
    float gyroZ;  // rad/s
    uint32_t timestamp; // millis()
};

namespace Sensors {

class MPUInterface {
public:
    MPUInterface();
    bool begin();
    bool readSample(SensorSample& sample);
    bool readEvents(sensors_event_t& accel, sensors_event_t& gyro, sensors_event_t& temp);

private:
    Adafruit_MPU6050 _mpu;
};

} // namespace Sensors

#endif // MPU_INTERFACE_H
