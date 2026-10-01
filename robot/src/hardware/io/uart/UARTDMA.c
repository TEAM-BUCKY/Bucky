#include "UARTDMA.h"

#include <string.h>
#include <Arduino.h>
#include <PeripheralPins.h>
#include <pinmap.h>
#include <uart.h>

#include "optimizations/bitboard.h"

/* Enable the instance's RCC clock and return its interrupt. */
#define UART_CASE(inst) \
    if (uart == inst) { __HAL_RCC_##inst##_CLK_ENABLE(); return inst##_IRQn; }

static IRQn_Type uart_clock_enable(const USART_TypeDef* uart)
{
    UART_CASE(USART1)
    UART_CASE(USART2)
    UART_CASE(USART3)
#ifdef UART4
    UART_CASE(UART4)
#endif
#ifdef UART5
    UART_CASE(UART5)
#endif
#ifdef USART6
    UART_CASE(USART6)
#endif
#ifdef UART7
    UART_CASE(UART7)
#endif
#ifdef UART8
    UART_CASE(UART8)
#endif
    return USART3_IRQn;
}

/* Must run with interrupts masked or from the TX DMA interrupt. */
static void tx_kick(UartDma* u)
{
    if (u->tx_inflight != 0U || u->tx_head == u->tx_tail) return;

    const uint16_t head = u->tx_head;
    const uint16_t tail = u->tx_tail;
    const uint16_t len  = head > tail ? (uint16_t)(head - tail) : (uint16_t)(u->cfg.tx_size - tail);

    dma_setup(u->cfg.tx_dma, DMA_MEM_TO_PERIPH, DMA_WIDTH_8, &u->cfg.uart->TDR,
              &u->cfg.tx_buf[tail], len, u->cfg.tx_request, false);
    dma_enable_events(u->cfg.tx_dma, DMA_EVT_TC);
    u->tx_inflight = len;
    u->cfg.uart->ICR = USART_ICR_TCCF;
    dma_enable(u->cfg.tx_dma);
}

static void tx_dma_isr(void* ctx)
{
    UartDma* u = (UartDma*)ctx;
    if (!(dma_take_events(u->cfg.tx_dma) & DMA_EVT_TC)) return;

    u->tx_tail = (uint16_t)((u->tx_tail + u->tx_inflight) & (u->cfg.tx_size - 1U));
    u->tx_inflight = 0;
    tx_kick(u);
}

static void uart_isr(void* ctx)
{
    UartDma* u = (UartDma*)ctx;
    USART_TypeDef* uart = u->cfg.uart;
    const uint32_t isr = uart->ISR;

    uart->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;

    if (isr & USART_ISR_IDLE) {
        uart->ICR = USART_ICR_IDLECF;
        if (u->cfg.on_rx) u->cfg.on_rx(u->cfg.on_rx_ctx);
    }
}

void uart_dma_init(UartDma* u, const UartDmaConfig* cfg)
{
    memset(u, 0, sizeof(*u));
    u->cfg = *cfg;
    USART_TypeDef* uart = cfg->uart;

    const IRQn_Type uart_irq = uart_clock_enable(uart);
    pinmap_pinout(cfg->tx_pin, PinMap_UART_TX);
    pinmap_pinout(cfg->rx_pin, PinMap_UART_RX);

    uart->CR1 = 0;
    uart->CR2 = 0;
    uart->CR3 = USART_CR3_OVRDIS;
    uart->PRESC = 0;

    UART_HandleTypeDef handle = {0};
    handle.Instance = uart;
    const uint32_t clk = uart_get_clock_source_freq(&handle);
    uart->BRR = (clk + cfg->baud / 2U) / cfg->baud;

    dma_setup(cfg->rx_dma, DMA_PERIPH_TO_MEM, DMA_WIDTH_8, &uart->RDR, cfg->rx_buf,
              cfg->rx_size, cfg->rx_request, true);
    dma_enable(cfg->rx_dma);

    dma_reset(cfg->tx_dma);
    irq_attach(dma_irqn(cfg->tx_dma), tx_dma_isr, u, cfg->irq_priority);

    setMask(uart->CR3, USART_CR3_DMAR | USART_CR3_DMAT);
    uart->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE
              | (cfg->on_rx ? USART_CR1_IDLEIE : 0U);

    if (cfg->on_rx)
        irq_attach(uart_irq, uart_isr, u, cfg->irq_priority);
}

size_t uart_dma_write_available(const UartDma* u)
{
    const uint16_t mask = (uint16_t)(u->cfg.tx_size - 1U);
    return (size_t)((u->tx_tail - u->tx_head - 1U) & mask);
}

size_t uart_dma_write(UartDma* u, const uint8_t* data, size_t len)
{
    const uint16_t mask = (uint16_t)(u->cfg.tx_size - 1U);
    size_t free_bytes = uart_dma_write_available(u);
    if (len > free_bytes) len = free_bytes;

    uint16_t head = u->tx_head;
    for (size_t i = 0; i < len; i++) {
        u->cfg.tx_buf[head] = data[i];
        head = (uint16_t)((head + 1U) & mask);
    }

    const uint32_t primask = irq_lock();
    u->tx_head = head;
    tx_kick(u);
    irq_restore(primask);

    return len;
}

void uart_dma_flush(const UartDma* u)
{
    while (u->tx_inflight != 0U || u->tx_head != u->tx_tail) {}
    while (!testMask(u->cfg.uart->ISR, USART_ISR_TC)) {}
}

size_t uart_dma_available(UartDma* u)
{
    const uint16_t mask = (uint16_t)(u->cfg.rx_size - 1U);
    const uint16_t head = (uint16_t)((u->cfg.rx_size - dma_remaining(u->cfg.rx_dma)) & mask);
    return (size_t)((head - u->rx_tail) & mask);
}

size_t uart_dma_read(UartDma* u, uint8_t* out, size_t len)
{
    const uint16_t mask = (uint16_t)(u->cfg.rx_size - 1U);
    const size_t avail = uart_dma_available(u);
    if (len > avail) len = avail;

    for (size_t i = 0; i < len; i++) {
        out[i] = u->cfg.rx_buf[u->rx_tail];
        u->rx_tail = (uint16_t)((u->rx_tail + 1U) & mask);
    }
    return len;
}

int uart_dma_read_byte(UartDma* u)
{
    uint8_t b;
    return uart_dma_read(u, &b, 1) ? b : -1;
}
