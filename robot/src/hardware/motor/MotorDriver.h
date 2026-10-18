#ifndef ROBOT_DRV8870DDAR_H
#define ROBOT_DRV8870DDAR_H

#include <cstdint>

#define DRV8870DDAR // OPTION: Choose what driver user is using.

#ifdef DRV8870DDAR

constexpr uint32_t DRIVER_PWM_FREQ_HZ = 25000;
constexpr uint32_t DRIVER_PWM_RESOLUTION = 10000;

#else

constexpr uint32_t DRIVER_PWM_FREQ_HZ = 25000;
constexpr uint32_t DRIVER_PWM_RESOLUTION = 10000;

#endif

#endif //ROBOT_DRV8870DDAR_H