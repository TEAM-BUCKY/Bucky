#ifndef BUCKY_SERIAL_PORT_H
#define BUCKY_SERIAL_PORT_H

#include <Arduino.h>

#include "hardware/io/uart/UARTDMA.h"

// Non-blocking byte streams for the host computer (USB CDC) and the Bluetooth
// module (USART + DMA). Nothing here ever waits:
//
//   write  copies into a ring buffer and returns at once; the bytes are moved
//          out by interrupts (DMA complete / USB IN complete). Whatever does
//          not fit is dropped and the return value says how much was taken.
//   read   returns what has already arrived, possibly nothing.
//
// Print is inherited, so port.print(x) / port.println(x) work and never block.
class SerialPort : public Print {
public:
    using Print::write;

    // Queue up to len bytes. Returns how many were accepted.
    size_t write(const uint8_t* data, size_t len) override = 0;
    size_t write(const uint8_t b) override { return write(&b, 1); }

    // Queue the whole buffer or nothing, so a packet is never cut in half.
    bool writeAll(const void* data, const size_t len) {
        if (static_cast<size_t>(availableForWrite()) < len) return false;
        return write(static_cast<const uint8_t*>(data), len) == len;
    }

    // Free space in the transmit buffer.
    int availableForWrite() override = 0;

    // Bytes waiting to be read.
    virtual size_t available() = 0;

    // Copy up to len received bytes. Returns how many were copied.
    virtual size_t read(uint8_t* out, size_t len) = 0;

    // Next byte, or -1 when nothing is waiting.
    int read() {
        uint8_t b;
        return read(&b, 1) ? b : -1;
    }
};

// ------------------------------------------------------------------ USB ---

// USB CDC to the host computer. Bytes queue in a RAM ring that the USB
// interrupt feeds into the core's CDC packet queue, so a write never waits
// for the bus. Before begin() and while no terminal has the port open, output
// is discarded.
//
// Do not write through Arduino `Serial` once this is in use: both would feed
// the same CDC queue from different contexts. Reading `Serial` is fine.
class UsbSerialPort final : public SerialPort {
public:
    static constexpr size_t TX_SIZE = 2048;   // power of two

    using SerialPort::read;
    using SerialPort::write;

    void begin();

    // A terminal has the port open and is reading it.
    [[nodiscard]] bool connected() const;

    size_t write(const uint8_t* data, size_t len) override;
    int availableForWrite() override;
    size_t available() override;
    size_t read(uint8_t* out, size_t len) override;

private:
    static void onUsbIrq(void* ctx);
    void drain();

    uint8_t txBuf[TX_SIZE] = {};
    volatile uint16_t txHead = 0;   // written by the producer (main loop)
    volatile uint16_t txTail = 0;   // advanced by the USB interrupt
    bool started = false;
};

// ----------------------------------------------------------------- UART ---

// USART with DMA in both directions; see UARTDMA.h. Buffer sizes must be
// powers of two.
template<uint16_t TxSize, uint16_t RxSize>
class UartSerialPort final : public SerialPort {
    static_assert((TxSize & (TxSize - 1)) == 0 && TxSize >= 2, "TxSize must be a power of two");
    static_assert((RxSize & (RxSize - 1)) == 0 && RxSize >= 2, "RxSize must be a power of two");

public:
    using SerialPort::read;
    using SerialPort::write;

    // onRx (optional) runs from the USART idle-line interrupt after each burst.
    void begin(const UartHardware& hw, const uint32_t baud, const uint8_t irqPriority = 4,
               const IrqHandler onRx = nullptr, void* onRxCtx = nullptr) {
        const UartDmaConfig cfg = {
            .uart = hw.instance,
            .tx_pin = hw.tx,
            .rx_pin = hw.rx,
            .baud = baud,
            .tx_dma = hw.dmaTx,
            .tx_request = hw.dmaTxRequest,
            .tx_buf = txBuf,
            .tx_size = TxSize,
            .rx_dma = hw.dmaRx,
            .rx_request = hw.dmaRxRequest,
            .rx_buf = rxBuf,
            .rx_size = RxSize,
            .irq_priority = irqPriority,
            .on_rx = onRx,
            .on_rx_ctx = onRxCtx,
        };
        uart_dma_init(&uart, &cfg);
    }

    size_t write(const uint8_t* data, const size_t len) override { return uart_dma_write(&uart, data, len); }
    int availableForWrite() override { return static_cast<int>(uart_dma_write_available(&uart)); }
    size_t available() override { return uart_dma_available(&uart); }
    size_t read(uint8_t* out, const size_t len) override { return uart_dma_read(&uart, out, len); }

private:
    UartDma uart = {};
    uint8_t txBuf[TxSize] = {};
    uint8_t rxBuf[RxSize] = {};
};

#endif // BUCKY_SERIAL_PORT_H
