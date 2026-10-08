#include "Kicker.h"

#include "optimizations/logic.h"

constexpr uint32_t KICKER_PWM_FREQ_HZ = 5000;
constexpr uint32_t KICKER_PWM_RESOLUTION = 3399;

void Kicker::init()
{
    kickerPwm = pwm_pin_init(kickerPin);
    pwm_init(&kickerPwm, KICKER_PWM_FREQ_HZ, KICKER_PWM_RESOLUTION);
    pwm_write(&kickerPwm, 0);
    pwm_sync_register(&kickerPwm);
    pwm_sync_timers();
}

/**
 * Kicks the ball with the specified speed. Applies the voltage to the solenoid.
 * @param speed The speed of the kick, in the range [0.0, 1.0].
 */
void Kicker::kick(float speed) const
{
    speed = clamp(speed, 0.0f, 1.0f);
    pwm_write(&kickerPwm, static_cast<uint32_t>(speed * KICKER_PWM_RESOLUTION));
}

/**
 * Stops the kicker by setting the PWM duty cycle to 0. This will stop applying voltage to the solenoid.
 */
void Kicker::stop() const
{
    pwm_write(&kickerPwm, 0);
}
