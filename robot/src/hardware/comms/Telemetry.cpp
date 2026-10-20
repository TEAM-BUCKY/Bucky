#include "Telemetry.h"

#include <Arduino.h>

#include <hardware/comms/JsonWriter.h>
#include <hardware/motor/Motors.h>
#include <hardware/sensors/ir/IRRing.h>
#include <hardware/sensors/ln/LineSensor.h>
#include <hardware/sensors/pos/Compass.h>
#include <hardware/sensors/pos/Sonar.h>
#include <strategy/EKF.h>

void Telemetry::begin(const char* boardName, const TelemetrySources& sources)
{
    board = boardName;
    src = sources;
}

bool Telemetry::addLink(SerialPort& port, const uint16_t periodMs)
{
    if (linkCount >= MAX_LINKS) return false;
    links[linkCount++] = {&port, periodMs, 0, 0, false};
    return true;
}

void Telemetry::tick()
{
    const uint32_t now = millis();
    for (uint8_t i = 0; i < linkCount; i++) {
        Link& link = links[i];
        const bool up = link.port->connected();
        if (!up) {
            link.up = false;
            continue;
        }
        if (!link.up) {
            link.up = true;
            link.seq = 0;
            link.lastSend = now - link.periodMs;
            sendHello(link);
        }
        if (now - link.lastSend >= link.periodMs) {
            link.lastSend = now;
            sendState(link);
        }
    }
}

void Telemetry::sendHello(Link& link)
{
    char buf[96];
    JsonWriter w(buf, sizeof(buf));
    w.beginObject()
     .field("t", "hello")
     .field("v", PROTOCOL_VERSION)
     .field("board", board)
     .field("ms", millis())
     .endObject()
     .newline();
    if (!w.overflowed()) send(link, buf, w.length());
}

void Telemetry::sendState(Link& link)
{
    char buf[FRAME_SIZE];
    JsonWriter w(buf, sizeof(buf));
    w.beginObject()
     .field("t", "state")
     .field("ms", millis())
     .field("seq", link.seq++);

    if (src.compass != nullptr) {
        w.field("heading", src.compass->getHeading(), 1);
        w.field("offset", src.compass->getOffset(), 1);
    }

    if (src.motors != nullptr) {
        float speeds[Motors::MOTOR_COUNT];
        src.motors->getEncoderSpeeds(speeds);
        w.key("enc").beginArray();
        for (const float s : speeds) w.value(s, 2);
        w.endArray();
    }

    if (src.sonar != nullptr) {
        const SonarReading& r = src.sonar->last();
        w.key("sonar").beginArray();
        for (int i = 0; i < SONAR_COUNT; i++) {
            if (r.valid[i]) w.value(r.distance[i], 1);
            else w.null();
        }
        w.endArray();
    }

    if (src.irRing != nullptr) {
        const BallReading& b = src.irRing->ball();
        w.key("ball").beginObject()
         .field("seen", b.seen)
         .field("angle", b.angle, 3)
         .field("dist", b.distanceCm, 1)
         .field("x", b.position.x, 1)
         .field("y", b.position.y, 1)
         .field("conf", static_cast<float>(b.confidence), 2)
         .endObject();

        const uint16_t* raw = src.irRing->raw();
        w.key("ir").beginArray();
        for (uint8_t i = 0; i < src.irRing->config().sensors; i++) w.value(raw[i]);
        w.endArray();
    }

    if (src.line != nullptr) {
        w.key("line").beginArray();
        for (uint8_t i = 0; i < LineSensor::SENSORS; i++) w.value(src.line->reflected(LineColor::Green, i));
        w.endArray();
    }

    if (src.ekf != nullptr) {
        const State& s = src.ekf->getState();
        w.key("pose").beginObject()
         .field("x", s.robotX, 1)
         .field("y", s.robotY, 1)
         .field("theta", s.robotTheta, 3)
         .field("vx", s.speedX, 1)
         .field("vy", s.speedY, 1)
         .field("omega", s.robotOmega, 3)
         .endObject();
    }

    w.field("drop", drops);
    w.endObject().newline();

    if (w.overflowed()) {
        drops++;
        return;
    }
    send(link, buf, w.length());
}

void Telemetry::sendLog(const char* text, const size_t len)
{
    char buf[LOG_LINE_SIZE * 2 + 48];
    JsonWriter w(buf, sizeof(buf));
    w.beginObject()
     .field("t", "log")
     .field("ms", millis())
     .field("m", text, len)
     .endObject()
     .newline();
    if (w.overflowed()) {
        drops++;
        return;
    }
    for (uint8_t i = 0; i < linkCount; i++)
        if (links[i].up) send(links[i], buf, w.length());
}

bool Telemetry::send(Link& link, const char* data, const size_t len)
{
    if (link.port->writeAll(data, len)) return true;
    drops++;
    return false;
}

void Telemetry::writeRaw(const uint8_t* data, const size_t len)
{
    if (rawOut == nullptr) return;
    size_t sent = rawOut->write(data, len);
    while (blockingRaw && sent < len) sent += rawOut->write(data + sent, len - sent);
}

bool Telemetry::anyLinkUp() const
{
    for (uint8_t i = 0; i < linkCount; i++)
        if (links[i].up) return true;
    return false;
}

bool Telemetry::rawIsUpLink() const
{
    for (uint8_t i = 0; i < linkCount; i++)
        if (links[i].up && static_cast<Print*>(links[i].port) == rawOut) return true;
    return false;
}

size_t Telemetry::LogSink::write(const uint8_t* data, const size_t n)
{
    if (!owner.anyLinkUp()) {
        if (len > 0) {   // a line started while a link was up
            owner.writeRaw(reinterpret_cast<const uint8_t*>(line), len);
            len = 0;
        }
        owner.writeRaw(data, n);
        return n;
    }

    for (size_t i = 0; i < n; i++) {
        const char c = static_cast<char>(data[i]);
        if (c == '\r') continue;
        if (c == '\n') {
            flushLine();
            continue;
        }
        line[len++] = c;
        if (len == LOG_LINE_SIZE) flushLine();
    }
    return n;
}

void Telemetry::LogSink::flushLine()
{
    owner.sendLog(line, len);
    if (!owner.rawIsUpLink()) {
        owner.writeRaw(reinterpret_cast<const uint8_t*>(line), len);
        owner.writeRaw(reinterpret_cast<const uint8_t*>("\r\n"), 2);
    }
    len = 0;
}
