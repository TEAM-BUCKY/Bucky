#include "Compass.h"

bool Compass::init(TwoWire& wire, int calibrationSamples) {
    if (!imu.begin_I2C(LSM6DS_I2CADDR_DEFAULT, &wire)) {
        return false;
    }

    imu.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    imu.setGyroDataRate(LSM6DS_RATE_104_HZ);

    // Calibrate gyro bias — keep robot still during startup
    gyroBiasZ = 0;
    sensors_event_t accel, gyro, temp;

    delay(100);
    for (int i = 0; i < calibrationSamples; i++) {
        imu.getEvent(&accel, &gyro, &temp);
        gyroBiasZ += gyro.gyro.z;
        delay(5);
    }
    gyroBiasZ /= calibrationSamples;

    lastUpdateMicros = micros();
    heading = 0;

    return true;
}

void Compass::update() {
    sensors_event_t accel, gyro, temp;
    imu.getEvent(&accel, &gyro, &temp);

    unsigned long now = micros();
    float dt = (now - lastUpdateMicros) / 1000000.0f;
    lastUpdateMicros = now;

    // Integrate gyro Z (rad/s → degrees)
    float gyroZ = gyro.gyro.z - gyroBiasZ;
    heading += gyroZ * dt * (180.0f / PI);
}

float Compass::getHeading() const {
    return heading;
}

void Compass::reset() {
    heading = 0;
}
