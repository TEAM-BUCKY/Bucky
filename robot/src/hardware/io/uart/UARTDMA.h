#ifndef BUCKY_UART_DMA_H
#define BUCKY_UART_DMA_H

#include <stdbool.h>
#include <stddef.h>
#include <PinNames.h>

#include <hardware/io/mcu.h>
#include <hardware/io/dma/DMA.h>
#include <hardware/io/irq/IRQ.h>

// rx_size/tx_size must be powers of two, at most 65535.

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    USART_TypeDef* instance;
    PinName        tx;
    PinName        rx;
    DmaChannel*    dmaTx;
    uint32_t       dmaTxRequest;
    DmaChannel*    dmaRx;
    uint32_t       dmaRxRequest;
} UartHardware;

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

    uint16_t rx_tail;
} UartDma;

void uart_dma_init(UartDma* u, const UartDmaConfig* cfg);

// Queue bytes for transmission. Returns how many fit in the TX buffer.
size_t uart_dma_write(UartDma* u, const uint8_t* data, size_t len);

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
