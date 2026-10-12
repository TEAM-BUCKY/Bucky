// Flash-emulated EEPROM (stm32duino utility/stm32_eeprom.c semantics). The flash image persists
// across sim.reboot(), like real flash; Python can read, write or erase it.
#include <utility/stm32_eeprom.h>

#include <cstring>

namespace sim::eeprom {
uint8_t flash[E2END + 1];
uint8_t buffer[E2END + 1];
void erase() {
    std::memset(flash, 0xFF, sizeof flash);
    std::memset(buffer, 0xFF, sizeof buffer);
}
const bool initialised = [] {
    erase();
    return true;
}();
}  // namespace sim::eeprom

using sim::eeprom::buffer;
using sim::eeprom::flash;

extern "C" {

uint8_t eeprom_read_byte(const uint32_t pos) { return pos <= E2END ? flash[pos] : 0; }

void eeprom_write_byte(const uint32_t pos, const uint8_t value) {
    eeprom_buffer_fill();
    eeprom_buffered_write_byte(pos, value);
    eeprom_buffer_flush();
}

void eeprom_buffer_fill(void) { std::memcpy(buffer, flash, sizeof flash); }
void eeprom_buffer_flush(void) { std::memcpy(flash, buffer, sizeof flash); }

uint8_t eeprom_buffered_read_byte(const uint32_t pos) { return pos <= E2END ? buffer[pos] : 0; }

void eeprom_buffered_write_byte(const uint32_t pos, const uint8_t value) {
    if (pos <= E2END) buffer[pos] = value;
}

}  // extern "C"
