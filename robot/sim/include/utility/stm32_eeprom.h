/* Host stand-in for stm32duino's utility/stm32_eeprom.h (flash-backed EEPROM emulation). */
#ifndef BUCKY_SIM_STM32_EEPROM_H
#define BUCKY_SIM_STM32_EEPROM_H

#include <stdint.h>

/* One 8 KB flash sector on the H5. */
#define E2END 0x1FFF

#ifdef __cplusplus
extern "C" {
#endif

uint8_t eeprom_read_byte(uint32_t pos);
void eeprom_write_byte(uint32_t pos, uint8_t value);

void eeprom_buffer_fill(void);
void eeprom_buffer_flush(void);
uint8_t eeprom_buffered_read_byte(uint32_t pos);
void eeprom_buffered_write_byte(uint32_t pos, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif
