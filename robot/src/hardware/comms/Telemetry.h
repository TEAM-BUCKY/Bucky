#ifndef BUCKY_TELEMETRY_H
#define BUCKY_TELEMETRY_H

#include <Print.h>

#include <hardware/io/serial/SerialPort.h>

class Compass;
class Motors;
class Sonar;
class IRRing;
class LineSensor;
class EKF;

// Sources a frame is built from; null ones are left out of the frame.
struct TelemetrySources {
    Compass* compass = nullptr;
    Motors* motors = nullptr;
    Sonar* sonar = nullptr;
    IRRing* irRing = nullptr;
    LineSensor* line = nullptr;
    EKF* ekf = nullptr;
};

// Streams robot state and debug logs as newline-delimited JSON over up to MAX_LINKS serial
// links (USB, HM-10, ...), one JSON object per line:
//   {"t":"hello","v":1,"board":..,"ms":..}   once each time a link comes up
//   {"t":"state","ms":..,"seq":..,...}        every period while the link is up
//   {"t":"log","ms":..,"m":".."}             one complete debug line
// Only cached values are read, so tick() never blocks on a sensor. A frame that does not fit in a
// link's TX buffer is dropped whole and counted in "drop".
class Telemetry {
public:
    static constexpr uint8_t PROTOCOL_VERSION = 1;
    static constexpr uint8_t MAX_LINKS = 2;

    void begin(const char* boardName, const TelemetrySources& sources);

    bool addLink(SerialPort& port, uint16_t periodMs);

    // Where log lines go while no link is up (normally `host`). Also skipped while it is itself an up link.
    void setRawLogOutput(Print& out) { rawOut = &out; }
    void setLogBlocking(const bool on) { blockingRaw = on; }

    // Pass to dbg.setOutput(). With no link up, bytes pass straight through to the raw output; with a
    // link up, they are collected per line and sent as "log" messages.
    Print& logSink() { return sink; }

    // Call from the main loop: detects links coming up and sends due frames.
    void tick();

    [[nodiscard]] uint32_t droppedFrames() const { return drops; }

private:
    static constexpr size_t FRAME_SIZE = 640;
    static constexpr size_t LOG_LINE_SIZE = 160;

    struct Link {
        SerialPort* port;
        uint16_t periodMs;
        uint32_t lastSend;
        uint32_t seq;
        bool up;
    };

    class LogSink final : public Print {
    public:
        explicit LogSink(Telemetry& owner) : owner(owner) {}

        using Print::write;
        size_t write(const uint8_t* data, size_t len) override;
        size_t write(uint8_t b) override { return write(&b, 1); }
        int availableForWrite() override { return LOG_LINE_SIZE; }

    private:
        Telemetry& owner;
        char line[LOG_LINE_SIZE] = {};
        size_t len = 0;

        void flushLine();
    };

    void sendHello(Link& link);
    void sendState(Link& link);
    void sendLog(const char* text, size_t len);
    void writeRaw(const uint8_t* data, size_t len);
    bool send(Link& link, const char* data, size_t len);
    [[nodiscard]] bool anyLinkUp() const;
    [[nodiscard]] bool rawIsUpLink() const;

    const char* board = "";
    TelemetrySources src;
    Link links[MAX_LINKS] = {};
    uint8_t linkCount = 0;

    Print* rawOut = nullptr;
    bool blockingRaw = false;
    LogSink sink{*this};

    uint32_t drops = 0;
};

#endif // BUCKY_TELEMETRY_H
