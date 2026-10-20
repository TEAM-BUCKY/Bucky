#ifndef BUCKY_HM10_H
#define BUCKY_HM10_H

#include <stdint.h>
#include <string.h>

#include <hardware/io/serial/SerialPort.h>

// HM-10 BLE bridge: AT commands (no line ending) only work while unlinked; OK+CONN/OK+LOST are filtered from RX.

class HM10 final : public SerialPort {
public:
    static constexpr uint8_t NAME_MAX = 12;

    using SerialPort::read;
    using SerialPort::write;

    // Blocks up to ~2 s. False when absent, or still linked from before a reset.
    bool begin(const UartHardware& hw, uint32_t baud, const char* name = nullptr);

    [[nodiscard]] bool configured() const { return atReady; }
    bool connected() override;

    // Drops data while no central is linked: the module would read it as AT commands.
    size_t write(const uint8_t* data, size_t len) override;
    int availableForWrite() override;
    size_t available() override;
    size_t read(uint8_t* out, size_t len) override;

    // True only when the whole string was queued.
    bool writeString(const char* str) { return writeAll(str, strlen(str)); }

private:
    static constexpr uint16_t RX_SIZE = 256;
    static constexpr uint32_t RESPONSE_TIMEOUT_MS = 300;

    void switchBaud(uint32_t rate);
    bool command(const char* cmd, const char* expect, char* reply = nullptr, size_t replySize = 0);
    void discardInput();

    // Pull bytes from the UART, strip connection notifications into `linked`.
    void pump();
    void accept(uint8_t b);
    void emit(uint8_t b);

    UartSerialPort<512, 256> uart;
    const UartHardware* hw = nullptr;

    bool atReady = false;
    bool linked = false;

    uint8_t rxBuf[RX_SIZE] = {};
    uint16_t rxHead = 0, rxTail = 0;

    char pending[8] = {};   // possible start of a notification, held back from the data
    uint8_t pendingLen = 0;
    uint32_t pendingSince = 0;
};

#endif // BUCKY_HM10_H
