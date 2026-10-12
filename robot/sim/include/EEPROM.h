/* Host stand-in for stm32duino's EEPROM library (flash emulation).
 *
 * Same API and the same two-level model as the real one: eeprom_read_byte() reads "flash",
 * eeprom_buffered_* work on a RAM page buffer that eeprom_buffer_flush() writes back. The flash
 * image starts erased (0xFF) and is readable / writable from Python (bucky_fw.sim.eeprom). */
#ifndef BUCKY_SIM_EEPROM_H
#define BUCKY_SIM_EEPROM_H

#include <Arduino.h>
#include "utility/stm32_eeprom.h"

class EEPROMClass {
public:
    uint8_t read(int idx) { return eeprom_read_byte(static_cast<uint32_t>(idx)); }
    void write(int idx, uint8_t val) { eeprom_write_byte(static_cast<uint32_t>(idx), val); }
    void update(int idx, uint8_t val) { if (read(idx) != val) write(idx, val); }
    uint16_t length() { return E2END + 1; }

    template <typename T>
    T& get(int idx, T& t) {
        auto* ptr = reinterpret_cast<uint8_t*>(&t);
        for (size_t i = 0; i < sizeof(T); i++) ptr[i] = read(idx + static_cast<int>(i));
        return t;
    }

    template <typename T>
    const T& put(int idx, const T& t) {
        const auto* ptr = reinterpret_cast<const uint8_t*>(&t);
        for (size_t i = 0; i < sizeof(T); i++) update(idx + static_cast<int>(i), ptr[i]);
        return t;
    }
};

inline EEPROMClass EEPROM;

#endif
