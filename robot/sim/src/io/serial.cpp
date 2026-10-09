// Replacements for hardware/io/serial/UsbSerialPort.cpp and hardware/io/uart/UARTDMA.c.
//
// USB CDC: as on the robot, writes before begin() vanish and writes after it are accepted in full
// (the Debugger's blocking loop must never see a short write, or it would spin without a time
// hook). Output is captured for Python; Python can inject received bytes.
// UART (Bluetooth): TX is captured per USART, RX is injected. No baud-rate timing is modelled.
#include "hardware/io/serial/SerialPort.h"
#include "hardware/io/uart/UARTDMA.h"

#include <cstring>
#include <map>

#include "core/world.h"
#include "io/io.h"

namespace {

std::string usb_tx_buf, usb_rx_buf;
bool usb_is_connected = true;
std::map<const USART_TypeDef*, std::string> uart_tx_bufs, uart_rx_bufs;

void reset_serial() {
    usb_rx_buf.clear();
    for (auto& [u, s] : uart_rx_bufs) s.clear();
    // TX captures are Python's to drain; a reboot does not erase what was already printed.
}

const bool registered = [] {
    sim::world().reset_hooks.emplace_back(reset_serial);
    return true;
}();

size_t take(std::string& from, uint8_t* out, const size_t len) {
    const size_t n = std::min(len, from.size());
    std::memcpy(out, from.data(), n);
    from.erase(0, n);
    return n;
}

}  // namespace

namespace sim::io {
std::string& usb_tx() { return usb_tx_buf; }
std::string& usb_rx() { return usb_rx_buf; }
bool& usb_connected() { return usb_is_connected; }
std::string& uart_tx(USART_TypeDef* u) { return uart_tx_bufs[u]; }
std::string& uart_rx(USART_TypeDef* u) { return uart_rx_bufs[u]; }
}  // namespace sim::io

// ---- UsbSerialPort ----------------------------------------------------------------------------

void UsbSerialPort::onUsbIrq(void* /*ctx*/) {}
void UsbSerialPort::drain() {}

void UsbSerialPort::begin() { started = true; }

bool UsbSerialPort::connected() const { return usb_is_connected; }

int UsbSerialPort::availableForWrite() { return static_cast<int>(TX_SIZE - 1); }

size_t UsbSerialPort::write(const uint8_t* data, const size_t len) {
    if (!started) return len;
    usb_tx_buf.append(reinterpret_cast<const char*>(data), len);
    return len;
}

size_t UsbSerialPort::available() { return usb_rx_buf.size(); }

size_t UsbSerialPort::read(uint8_t* out, const size_t len) { return take(usb_rx_buf, out, len); }

// ---- UART DMA ---------------------------------------------------------------------------------

extern "C" {

void uart_dma_init(UartDma* u, const UartDmaConfig* cfg) {
    *u = UartDma{};
    u->cfg = *cfg;
}

size_t uart_dma_write(UartDma* u, const uint8_t* data, const size_t len) {
    uart_tx_bufs[u->cfg.uart].append(reinterpret_cast<const char*>(data), len);
    return len;
}

size_t uart_dma_write_available(const UartDma* u) { return u->cfg.tx_size ? u->cfg.tx_size - 1U : 0U; }

void uart_dma_flush(const UartDma* /*u*/) {}

size_t uart_dma_available(UartDma* u) { return uart_rx_bufs[u->cfg.uart].size(); }

size_t uart_dma_read(UartDma* u, uint8_t* out, const size_t len) {
    return take(uart_rx_bufs[u->cfg.uart], out, len);
}

int uart_dma_read_byte(UartDma* u) {
    uint8_t b;
    return uart_dma_read(u, &b, 1) ? b : -1;
}

}  // extern "C"
