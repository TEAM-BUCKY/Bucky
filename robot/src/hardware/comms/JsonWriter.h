#ifndef BUCKY_JSON_WRITER_H
#define BUCKY_JSON_WRITER_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <type_traits>

// Streaming JSON into a caller-owned buffer, no heap. Commas are inserted automatically.
// On overflow the output is cut short and overflowed() turns true; the buffer stays NUL-terminated.
class JsonWriter {
public:
    JsonWriter(char* buf, const size_t size) : buf(buf), size(size) {
        if (size > 0) buf[0] = '\0';
    }

    JsonWriter& beginObject() { open('{'); return *this; }
    JsonWriter& endObject() { close('}'); return *this; }
    JsonWriter& beginArray() { open('['); return *this; }
    JsonWriter& endArray() { close(']'); return *this; }

    JsonWriter& key(const char* k) {
        separator();
        string(k);
        put(':');
        afterKey = true;
        return *this;
    }

    template<typename T>
    std::enable_if_t<std::is_integral_v<T> && !std::is_same_v<T, bool>, JsonWriter&> value(const T v) {
        separator();
        if constexpr (std::is_signed_v<T>) {
            if (v < 0) {
                put('-');
                digits(static_cast<uint32_t>(-static_cast<int64_t>(v)));
            } else {
                digits(static_cast<uint32_t>(v));
            }
        } else {
            digits(static_cast<uint32_t>(v));
        }
        return *this;
    }

    JsonWriter& value(const bool v) {
        separator();
        text(v ? "true" : "false");
        return *this;
    }

    // Fixed decimals (at most 6). NaN and infinity become null.
    JsonWriter& value(const float v, uint8_t decimals = 2) {
        separator();
        if (!isfinite(v)) {
            text("null");
            return *this;
        }
        if (decimals > 6) decimals = 6;
        uint32_t scale = 1;
        for (uint8_t i = 0; i < decimals; i++) scale *= 10;

        const uint64_t scaled = static_cast<uint64_t>(fabsf(v) * static_cast<float>(scale) + 0.5f);
        if (v < 0.0f && scaled != 0) put('-');
        digits(static_cast<uint32_t>(scaled / scale));
        if (decimals > 0) {
            put('.');
            uint32_t frac = static_cast<uint32_t>(scaled % scale);
            for (uint32_t div = scale / 10; div > 0; div /= 10) {
                put(static_cast<char>('0' + frac / div));
                frac %= div;
            }
        }
        return *this;
    }

    JsonWriter& value(const char* s) {
        separator();
        string(s);
        return *this;
    }

    // String from a buffer that need not be NUL-terminated.
    JsonWriter& value(const char* s, const size_t len) {
        separator();
        string(s, len);
        return *this;
    }

    JsonWriter& null() {
        separator();
        text("null");
        return *this;
    }

    template<typename... Args>
    JsonWriter& field(const char* k, Args... v) { return key(k).value(v...); }

    JsonWriter& newline() { put('\n'); return *this; }

    [[nodiscard]] size_t length() const { return len; }
    [[nodiscard]] bool overflowed() const { return overflow; }
    [[nodiscard]] const char* c_str() const { return buf; }

private:
    static constexpr uint8_t MAX_DEPTH = 8;

    char* buf;
    size_t size;
    size_t len = 0;
    bool overflow = false;

    bool hasItems[MAX_DEPTH] = {};
    uint8_t depth = 0;
    bool afterKey = false;

    void put(const char c) {
        if (len + 1 >= size) {
            overflow = true;
            return;
        }
        buf[len++] = c;
        buf[len] = '\0';
    }

    void text(const char* s) {
        while (*s) put(*s++);
    }

    void digits(uint32_t v) {
        char tmp[10];
        uint8_t n = 0;
        do {
            tmp[n++] = static_cast<char>('0' + v % 10);
            v /= 10;
        } while (v > 0);
        while (n > 0) put(tmp[--n]);
    }

    void string(const char* s, const size_t n) {
        static constexpr char HEX_DIGITS[] = "0123456789abcdef";
        put('"');
        for (size_t i = 0; i < n; i++) {
            const char c = s[i];
            switch (c) {
                case '"':  text("\\\""); break;
                case '\\': text("\\\\"); break;
                case '\n': text("\\n"); break;
                case '\r': text("\\r"); break;
                case '\t': text("\\t"); break;
                default:
                    if (static_cast<uint8_t>(c) < 0x20) {
                        text("\\u00");
                        put(HEX_DIGITS[c >> 4]);
                        put(HEX_DIGITS[c & 0xF]);
                    } else {
                        put(c);
                    }
            }
        }
        put('"');
    }

    void string(const char* s) {
        size_t n = 0;
        while (s[n]) n++;
        string(s, n);
    }

    // Comma before every item except the first in its container; nothing right after a key.
    void separator() {
        if (afterKey) {
            afterKey = false;
            return;
        }
        if (depth == 0) return;
        bool& items = hasItems[(depth > MAX_DEPTH ? MAX_DEPTH : depth) - 1];
        if (items) put(',');
        items = true;
    }

    void open(const char c) {
        separator();
        put(c);
        if (depth < MAX_DEPTH) hasItems[depth] = false;
        depth++;
    }

    void close(const char c) {
        put(c);
        if (depth > 0) depth--;
    }
};

#endif // BUCKY_JSON_WRITER_H
