#include "HM10.h"

#include <string.h>
#include <wiring_time.h>
#include <initializer_list>

namespace
{
    constexpr auto NOTE_CONN = "OK+CONN";
    constexpr auto NOTE_LOST = "OK+LOST";
    constexpr uint32_t PENDING_FLUSH_MS = 20;   // a held-back "OK+..." that never completes is data
    constexpr uint32_t REPLY_QUIET_MS = 20;     // replies have no terminator: done once the line goes quiet
    constexpr uint32_t RESET_BOOT_MS = 1000;

    // AT+BAUD<n> codes; -1 when the module cannot run at `baud`.
    int8_t baudCode(const uint32_t baud)
    {
        switch (baud) {
            case 9600:   return 0;
            case 19200:  return 1;
            case 38400:  return 2;
            case 57600:  return 3;
            case 115200: return 4;
            case 230400: return 8;
            default:     return -1;
        }
    }
}

bool HM10::begin(const UartHardware& uartHw, const uint32_t baud, const char* name)
{
    hw = &uartHw;
    atReady = false;
    linked = false;
    rxHead = rxTail = 0;
    pendingLen = 0;

    // The requested rate first (already set from an earlier boot), then the factory 9600.
    const uint32_t candidates[] = {baud, 9600, 115200, 57600, 38400, 19200, 230400};
    uint32_t found = 0;
    for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++) {
        const uint32_t rate = candidates[i];
        if (i > 0 && rate == baud) continue;
        switchBaud(rate);
        if (command("AT", "OK")) {
            found = rate;
            break;
        }
    }
    if (found == 0) {
        switchBaud(baud);
        linked = true;
        return false;
    }
    atReady = true;

    bool needReset = false;
    bool ok = true;

    const int8_t code = baudCode(baud);
    if (found != baud) {
        if (code < 0) {
            ok = false;
        } else {
            const char cmd[] = {'A', 'T', '+', 'B', 'A', 'U', 'D', static_cast<char>('0' + code), '\0'};
            const char expect[] = {'O', 'K', '+', 'S', 'e', 't', ':', static_cast<char>('0' + code), '\0'};
            if (command(cmd, expect)) needReset = true;
            else ok = false;
        }
    }

    ok &= command("AT+NOTI1", "OK+Set:1");

    if (name != nullptr) {
        char current[NAME_MAX + 1] = {};
        if (!command("AT+NAME?", "OK+NAME:", current, sizeof(current)) || strcmp(current, name) != 0) {
            char cmd[8 + NAME_MAX] = "AT+NAME";
            char expect[8 + NAME_MAX] = "OK+Set:";
            strncat(cmd, name, NAME_MAX);
            strncat(expect, name, NAME_MAX);
            if (command(cmd, expect)) needReset = true;
            else ok = false;
        }
    }

    if (needReset) {
        command("AT+RESET", "OK+RESET");
        delay(RESET_BOOT_MS);
        switchBaud(code >= 0 ? baud : found);
        atReady = command("AT", "OK");
        ok &= atReady;
    }

    return ok;
}

bool HM10::connected()
{
    pump();
    return linked;
}

size_t HM10::write(const uint8_t* data, const size_t len)
{
    if (!connected()) return 0;
    return uart.write(data, len);
}

int HM10::availableForWrite()
{
    return connected() ? uart.availableForWrite() : 0;
}

size_t HM10::available()
{
    pump();
    return static_cast<uint16_t>(rxHead - rxTail) & (RX_SIZE - 1);
}

size_t HM10::read(uint8_t* out, const size_t len)
{
    pump();
    size_t n = 0;
    while (n < len && rxTail != rxHead) {
        out[n++] = rxBuf[rxTail];
        rxTail = (rxTail + 1) & (RX_SIZE - 1);
    }
    return n;
}

void HM10::switchBaud(const uint32_t rate)
{
    uart.begin(*hw, rate);
    discardInput();
}

bool HM10::command(const char* cmd, const char* expect, char* reply, const size_t replySize)
{
    discardInput();
    uart.write(reinterpret_cast<const uint8_t*>(cmd), strlen(cmd));

    char response[48] = {};
    size_t len = 0;
    const char* match = nullptr;
    const uint32_t start = millis();
    uint32_t lastByte = start;
    while (millis() - start < RESPONSE_TIMEOUT_MS) {
        uint8_t b;
        if (uart.read(&b, 1) == 1) {
            if (len < sizeof(response) - 1) response[len++] = static_cast<char>(b);
            lastByte = millis();
            if (match == nullptr) match = strstr(response, expect);
            continue;
        }
        if (match != nullptr && millis() - lastByte >= REPLY_QUIET_MS) break;
        delay(1);
    }
    if (match == nullptr) return false;

    if (reply != nullptr && replySize > 0) {
        strncpy(reply, match + strlen(expect), replySize - 1);
        reply[replySize - 1] = '\0';
    }
    return true;
}

void HM10::discardInput()
{
    uint8_t scratch[32];
    while (uart.read(scratch, sizeof(scratch)) > 0) {}
}

void HM10::pump()
{
    uint8_t chunk[32];
    size_t n;
    while ((n = uart.read(chunk, sizeof(chunk))) > 0) {
        for (size_t i = 0; i < n; i++) accept(chunk[i]);
    }

    if (pendingLen > 0 && millis() - pendingSince >= PENDING_FLUSH_MS) {
        for (uint8_t i = 0; i < pendingLen; i++) emit(pending[i]);
        pendingLen = 0;
    }
}

void HM10::accept(const uint8_t b)
{
    if (pendingLen == 0) pendingSince = millis();
    pending[pendingLen++] = static_cast<char>(b);

    while (pendingLen > 0) {
        bool prefix = false;
        for (const char* note : {NOTE_CONN, NOTE_LOST}) {
            if (strncmp(pending, note, pendingLen) != 0) continue;
            if (pendingLen == strlen(note)) {
                linked = note == NOTE_CONN;
                pendingLen = 0;
                return;
            }
            prefix = true;
        }
        if (prefix) return;

        // Not a notification after all: release the first byte and re-check the rest.
        emit(pending[0]);
        memmove(pending, pending + 1, --pendingLen);
        pendingSince = millis();
    }
}

void HM10::emit(const uint8_t b)
{
    const uint16_t next = (rxHead + 1) & (RX_SIZE - 1);
    if (next == rxTail) return;   // full: drop the newest byte
    rxBuf[rxHead] = b;
    rxHead = next;
}
