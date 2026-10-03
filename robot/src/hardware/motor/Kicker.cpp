#include "Kicker.h"

constexpr uint32_t KICKER_PWM_FREQ_HZ = 5000;
constexpr uint32_t KICKER_PWM_RESOLUTION = 3399;

void KickerDriver::init(const float minSpeed, const float maxSpeed)
{
    changeSpeed(minSpeed, maxSpeed);

    kickerPwm = pwm_pin_init(kickerPin);
    pwm_init(&kickerPwm, KICKER_PWM_FREQ_HZ, KICKER_PWM_RESOLUTION);
    pwm_write(&kickerPwm, 0);
    pwm_sync_register(&kickerPwm);
    pwm_sync_timers();
}

void KickerDriver::kick(const float speed) const
{
    pwm_write(&kickerPwm, static_cast<uint32_t>(speed * speedRange.scale + speedRange.min));
}

void KickerDriver::stop() const
{
    pwm_write(&kickerPwm, 0);
}
