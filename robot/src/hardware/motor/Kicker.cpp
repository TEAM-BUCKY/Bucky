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

// speed in [0, 1] is the PWM duty applied to the solenoid.
void Kicker::kick(const Percentage speed) const
{
    pwm_write(&kickerPWM, static_cast<uint32_t>(speed * DRIVER_PWM_RESOLUTION));
}

void Kicker::stop() const
{
    pwm_write(&kickerPWM, 0);
}
