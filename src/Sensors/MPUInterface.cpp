#include "MPUInterface.h"
#include <Wire.h>

namespace Sensors {

MPUInterface::MPUInterface() {}

bool MPUInterface::begin() {
    Wire.begin();
    
    if (!_mpu.begin()) {
        return false;
    }

    _mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    _mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    _mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    return true;
}

bool MPUInterface::readSample(SensorSample& sample) {
    sensors_event_t accelEvent;
    sensors_event_t gyroEvent;
    sensors_event_t temp;

    if (!_mpu.getEvent(&accelEvent, &gyroEvent, &temp)) {
        return false;
    }

    sample.accelX = accelEvent.acceleration.x;
    sample.accelY = accelEvent.acceleration.y;
    sample.accelZ = accelEvent.acceleration.z;
    
    sample.gyroX = gyroEvent.gyro.x;
    sample.gyroY = gyroEvent.gyro.y;
    sample.gyroZ = gyroEvent.gyro.z;
    
    sample.timestamp = millis();

    return true;
}

} // namespace Sensors
