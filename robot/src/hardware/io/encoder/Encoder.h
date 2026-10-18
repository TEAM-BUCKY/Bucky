#ifndef BUCKY_ENCODER_H
#define BUCKY_ENCODER_H

#include <stdbool.h>
#include <stdint.h>
#include <PinNames.h>

#define ENCODER_MAX 3

#ifdef __cplusplus
extern "C" {
#endif

typedef struct EncoderPins {
    PinName pinA;
    PinName pinB;
} EncoderPins;

void encoder_init(uint8_t index, const EncoderPins* pins);

int32_t encoder_get_ticks(uint8_t index);
float   encoder_get_speed(uint8_t index);
void    encoder_reset(uint8_t index);
bool    encoder_is_active(uint8_t index);
void    encoder_update_speed(uint8_t index);

#ifdef __cplusplus
}
#endif

#endif // BUCKY_ENCODER_H
