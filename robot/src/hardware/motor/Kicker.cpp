#include "Kicker.h"

#include <hardware/motor/MotorDriver.h>
#include <optimizations/logic.h>



void Kicker::init()
{
    kickerPWM = pwm_pin_init(kickerPin);
    pwm_init(&kickerPWM, DRIVER_PWM_FREQ_HZ, DRIVER_PWM_RESOLUTION);
    pwm_write(&kickerPWM, 0);
    pwm_sync_register(&kickerPWM);
    pwm_sync_timers();
}

/**
 * Kicks the ball with the specified speed. Applies the voltage to the solenoid.
 * @param speed The speed of the kick, in the range [0.0, 1.0].
 */
void Kicker::kick(const Percentage speed) const
{
    pwm_write(&kickerPWM, static_cast<uint32_t>(speed * DRIVER_PWM_RESOLUTION));
}

/**
 * Stops the kicker by setting the PWM duty cycle to 0. This will stop applying voltage to the solenoid.
 */
void Kicker::stop() const
{
    pwm_write(&kickerPWM, 0);
}
