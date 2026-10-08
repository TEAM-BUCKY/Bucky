#include "SerialPort.h"

#include <usbd_conf.h>
#include <usbd_cdc_if.h>
#include <WSerial.h>

#include "hardware/io/irq/IRQ.h"

extern "C" void USB_IRQHandler(void);

static constexpr uint16_t TX_MASK = UsbSerialPort::TX_SIZE - 1U;
static_assert((UsbSerialPort::TX_SIZE & TX_MASK) == 0, "TX_SIZE must be a power of two");

void UsbSerialPort::onUsbIrq(void* ctx) {
    USB_IRQHandler();
    static_cast<UsbSerialPort*>(ctx)->drain();
}

void UsbSerialPort::begin() {
    if (started) return;
    Serial.begin();
    irq_attach(USB_IRQn, onUsbIrq, this, USBD_IRQ_PRIO);
    started = true;
}

bool UsbSerialPort::connected() const {
    return CDC_connected();
}

void UsbSerialPort::drain() {
    uint16_t tail = txTail;
    const uint16_t head = txHead;
    if (tail == head) return;

    if (!CDC_connected()) {
        txTail = head;
        return;
    }

    while (tail != head) {
        const int space = CDC_TransmitQueue_WriteSize(&TransmitQueue);
        if (space <= 0) break;

        const uint16_t run = head > tail ? head - tail : TX_SIZE - tail;
        const uint16_t n = run < space ? run : static_cast<uint16_t>(space);
        CDC_TransmitQueue_Enqueue(&TransmitQueue, &txBuf[tail], n);
        tail = (tail + n) & TX_MASK;
    }
    txTail = tail;
    CDC_continue_transmit();
}

int UsbSerialPort::availableForWrite() {
    return static_cast<int>((txTail - txHead - 1U) & TX_MASK);
}

size_t UsbSerialPort::write(const uint8_t* data, size_t len) {
    if (!started) return len;

    const auto space = static_cast<size_t>(availableForWrite());
    if (len > space) len = space;
    if (len == 0) return 0;

    uint16_t head = txHead;
    const size_t first = len < TX_SIZE - head ? len : TX_SIZE - head;
    memcpy(&txBuf[head], data, first);
    memcpy(&txBuf[0], data + first, len - first);
    head = (head + len) & TX_MASK;

    __DMB();
    txHead = head;

    NVIC_DisableIRQ(USB_IRQn);
    drain();
    NVIC_EnableIRQ(USB_IRQn);
    return len;
}

size_t UsbSerialPort::available() {
    return static_cast<size_t>(CDC_ReceiveQueue_ReadSize(&ReceiveQueue));
}

size_t UsbSerialPort::read(uint8_t* out, size_t len) {
    if (len > UINT16_MAX) len = UINT16_MAX;
    const uint16_t n = CDC_ReceiveQueue_Read(&ReceiveQueue, out, static_cast<uint16_t>(len));
    CDC_resume_receive();
    return n;
}
