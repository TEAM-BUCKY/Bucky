/* Host stand-in for the stm32duino Arduino core.
 *
 * Only what the firmware uses. Time (millis/micros/delay*) is the simulator's virtual clock:
 * every call is a point where simulated hardware events and interrupts can run, and where a
 * firmware program running in lockstep parks until Python advances the world.
 * Constants and macros match cores/arduino/wiring_constants.h.
 */
#ifndef Arduino_h
#define Arduino_h

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "stm32_def.h"
#include "PinNames.h"

#ifdef __cplusplus
#include <algorithm>
using std::min;
using std::max;
#include "WString.h"
#include "Print.h"
#endif

#define INPUT             0x0
#define OUTPUT            0x1
#define INPUT_PULLUP      0x2
#define INPUT_FLOATING    INPUT
#define INPUT_PULLDOWN    0x3
#define INPUT_ANALOG      0x4
#define OUTPUT_OPEN_DRAIN 0x5

#define PI         3.1415926535897932384626433832795
#define HALF_PI    1.5707963267948966192313216916398
#define TWO_PI     6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

#define LOW     0x0
#define HIGH    0x1
#define CHANGE  0x2
#define FALLING 0x3
#define RISING  0x4

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define radians(deg) ((deg) * DEG_TO_RAD)
#define degrees(rad) ((rad) * RAD_TO_DEG)
#define sq(x) ((x) * (x))

#ifdef __cplusplus
extern "C" {
#endif

void init(void);

uint32_t millis(void);
uint32_t micros(void);
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);

void pinMode(uint32_t pin, uint32_t mode);
void digitalWrite(uint32_t pin, uint32_t value);
int digitalRead(uint32_t pin);
void digitalWriteFast(PinName pin, uint32_t value);
int digitalReadFast(PinName pin);
void analogReadResolution(int bits);
uint32_t analogRead(uint32_t pin);

/* Digital pin numbers are PinName & PNAME_MASK in the simulator (one number per pad). */
uint32_t pinNametoDigitalPin(PinName p);
PinName digitalPinToPinName(uint32_t pin);

#ifdef __cplusplus
}

long map(long x, long in_min, long in_max, long out_min, long out_max);
long random(long howbig);
long random(long howsmall, long howbig);
#endif

#endif /* Arduino_h */
