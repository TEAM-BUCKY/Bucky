#ifndef BUCKY_DEBUG_H
#define BUCKY_DEBUG_H

#include <Print.h>

#include "hardware/io/serial/SerialPort.h"

#define DEBUG_LOGGING

enum DebugSubject : uint8_t {
    DEBUG_SUBJ_MAIN,

    DEBUG_SUBJ_MOTOR,
    DEBUG_SUBJ_ENCODER,

    DEBUG_SUBJ_SENSOR,
    DEBUG_SUBJ_POSITION,

    DEBUG_SUBJ_GPORT,
    DEBUG_SUBJ_CALIBRATION,

    DEBUG_SUBJ_OTHER,

    DEBUG_SUBJ_COUNT
};

static_assert(DEBUG_SUBJ_COUNT <= 32, "subject mask is 32 bits");

extern UsbSerialPort host;

class Debugger final : public Print {
public:
    using Print::write;

    void setOutput(Print& port) { out = &port; }
    void setBlocking(const bool on) { blocking = on; }

    // Bytes lost because the output buffer was full.
    [[nodiscard]] uint32_t dropped() const { return droppedBytes; }

    [[nodiscard]] bool isAllowed(const DebugSubject subject) const {
        return (denied & subjectMask(subject)) == 0;
    }

    void deny(const DebugSubject subject) { denied |= subjectMask(subject); }
    void allow(const DebugSubject subject) { denied &= ~subjectMask(subject); }
    void denyAll() { denied = ~0U; }
    void allowAll() { denied = 0; }

    void denyList(const DebugSubject* subjects, const size_t count) {
        for (size_t i = 0; i < count; ++i) deny(subjects[i]);
    }

    void allowList(const DebugSubject* subjects, const size_t count) {
        for (size_t i = 0; i < count; ++i) allow(subjects[i]);
    }

    size_t write(const uint8_t* data, size_t len) override {
        size_t sent = out->write(data, len);

        while (blocking && sent < len)
            sent += out->write(data + sent, len - sent);
        droppedBytes += len - sent;
        return sent;
    }

    size_t write(const uint8_t b) override { return write(&b, 1); }
    int availableForWrite() override { return out->availableForWrite(); }

private:
    static constexpr uint32_t subjectMask(const DebugSubject subject) { return 1UL << subject; }

    Print* out = &host;
    uint32_t denied = 0;
    uint32_t droppedBytes = 0;
    bool blocking = false;
};

inline Debugger dbg;

#ifdef DEBUG_LOGGING
constexpr bool DEBUG_ENABLED = true;
#else
constexpr bool DEBUG_ENABLED = false;
#endif

#define DBG_IF(cond, ...) do { if (DEBUG_ENABLED && (cond)) __VA_ARGS__; } while (0)

#define DBG_PRINT(...)    DBG_IF(true, dbg.print(__VA_ARGS__))
#define DBG_PRINTLN(...)  DBG_IF(true, dbg.println(__VA_ARGS__))
#define DBG_PRINTF(...)   DBG_IF(true, dbg.printf(__VA_ARGS__))

#define DBG_PRINT_SUBJECT(subject, ...)   DBG_IF(dbg.isAllowed(subject), dbg.print(__VA_ARGS__))
#define DBG_PRINTLN_SUBJECT(subject, ...) DBG_IF(dbg.isAllowed(subject), dbg.println(__VA_ARGS__))
#define DBG_PRINTF_SUBJECT(subject, ...)  DBG_IF(dbg.isAllowed(subject), dbg.printf(__VA_ARGS__))

#define DBG_DENY(subject)              DBG_IF(true, dbg.deny(subject))
#define DBG_DENY_ALL()                 DBG_IF(true, dbg.denyAll())
#define DBG_DENY_LIST(subjects, count) DBG_IF(true, dbg.denyList(subjects, count))

#define DBG_ALLOW(subject)              DBG_IF(true, dbg.allow(subject))
#define DBG_ALLOW_ALL()                 DBG_IF(true, dbg.allowAll())
#define DBG_ALLOW_LIST(subjects, count) DBG_IF(true, dbg.allowList(subjects, count))

#endif // BUCKY_DEBUG_H
