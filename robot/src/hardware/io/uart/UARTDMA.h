#ifndef BUCKY_UART_DMA_H
#define BUCKY_UART_DMA_H

#include <stdbool.h>
#include <stddef.h>
#include <PinNames.h>

#include "hardware/io/mcu.h"
#include "hardware/io/dma/DMA.h"
#include "hardware/io/irq/IRQ.h"

// USART with DMA on both directions.
//
// RX: a circular DMA channel streams RDR into rx_buf continuously; reads just
//     chase the DMA write position, so no per-byte interrupts. The optional
//     on_rx callback fires from the USART IDLE interrupt (end of a burst).
// TX: writes go into a ring buffer; the largest contiguous run is sent as one
//     DMA transfer and the transfer-complete interrupt chains the next run.
//
// Buffers are owned by the caller. rx_size/tx_size must be powers of two and
// at most 65535 (one DMA block).

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    USART_TypeDef* uart;
    PinName        tx_pin;
    PinName        rx_pin;
    uint32_t       baud;

    DmaChannel*    tx_dma;
    uint32_t       tx_request;     /* DMA_REQUEST_USARTx_TX / GPDMA1_REQUEST_USARTx_TX */
    uint8_t*       tx_buf;
    uint16_t       tx_size;

    DmaChannel*    rx_dma;
    uint32_t       rx_request;
    uint8_t*       rx_buf;
    uint16_t       rx_size;

    uint8_t        irq_priority;
    IrqHandler     on_rx;          /* optional, called on RX idle line */
    void*          on_rx_ctx;
} UartDmaConfig;

typedef struct {
    UartDmaConfig cfg;

    volatile uint16_t tx_head;     /* written by producer */
    volatile uint16_t tx_tail;     /* advanced by the DMA TC interrupt */
    volatile uint16_t tx_inflight; /* bytes in the running DMA transfer */

    uint16_t rx_tail;              /* next byte the reader will consume */
} UartDma;

void uart_dma_init(UartDma* u, const UartDmaConfig* cfg);

// Queue bytes for transmission. Returns how many fit in the TX buffer.
size_t uart_dma_write(UartDma* u, const uint8_t* data, size_t len);

// Free space in the TX buffer.
size_t uart_dma_write_available(const UartDma* u);

// Block until everything queued has left the shift register.
void uart_dma_flush(const UartDma* u);

size_t uart_dma_available(UartDma* u);
size_t uart_dma_read(UartDma* u, uint8_t* out, size_t len);

// -1 when no byte is waiting.
int uart_dma_read_byte(UartDma* u);

#ifdef __cplusplus
}
#endif

#endif // BUCKY_UART_DMA_H
