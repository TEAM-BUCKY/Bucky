#include "I2CDMA.h"
#include "optimizations/bitboard.h"

#define I2C_TIMEOUT 170000U  /* ~1 ms at 170 MHz */

static FORCE_INLINE uint32_t i2c_cr2_write(const uint8_t addr, const uint8_t n_bytes) {
    return (uint32_t)addr << 1 | (uint32_t)n_bytes << I2C_CR2_NBYTES_Pos;
}

static FORCE_INLINE uint32_t i2c_cr2_read(const uint8_t addr, const uint8_t n_bytes) {
    return i2c_cr2_write(addr, n_bytes) | I2C_CR2_RD_WRN;
}

static void gpio_init_i2c_pin(GPIO_TypeDef* port, const uint8_t pin, const uint8_t af) {
    writeField(port->AFR[pin >> 3], 0xFU, (pin & 7) * 4, af);
    writeField(port->MODER,   3U, pin * 2, 2U);
    setBit(port->OTYPER, pin);
    writeField(port->PUPDR,   3U, pin * 2, 1U);
    writeField(port->OSPEEDR, 3U, pin * 2, 3U);
}

static void enable_gpio_clock(const GPIO_TypeDef* port) {
    const uint32_t idx = ((uint32_t)port - (uint32_t)GPIOA) / 0x400U;
    setMask(RCC->AHB2ENR, RCC_AHB2ENR_GPIOAEN << idx);
}

static uint32_t dma_channel_index(const DMA_TypeDef* dma, const DMA_Channel_TypeDef* ch) {
    return ((uint32_t)ch - (uint32_t)dma - 0x08U) / 0x14U + 1U;
}

static FORCE_INLINE uint32_t i2c_poll_isr(I2C_TypeDef* i2c, const uint32_t flag)
{
    uint32_t isr, cnt = I2C_TIMEOUT, combined;
    asm volatile(
        "ORR  %[cmb], %[cmb], %[nack]\n\t"
        "1:\n\t"
        "LDR  %[isr], [%[base], %[off]]\n\t"
        "TST  %[isr], %[cmb]\n\t"
        "BNE  2f\n\t"
        "SUBS %[cnt], %[cnt], #1\n\t"
        "BNE  1b\n\t"
        "MOVS %[isr], #0\n\t"
        "2:"
        : [isr] "=&r" (isr), [cnt] "+r" (cnt), [cmb] "=r" (combined)
        : [base] "r" (i2c),
          "2" (flag),
          [nack] "I" (I2C_ISR_NACKF),
          [off]  "J" (offsetof(I2C_TypeDef, ISR))
        : "cc"
    );
    return isr;
}

static FORCE_INLINE bool i2c_poll(I2C_TypeDef* i2c, const uint32_t flag)
{
    const uint32_t isr = i2c_poll_isr(i2c, flag);
    if (isr & flag) return true;
    if (isr & I2C_ISR_NACKF)
        i2c->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;
    return false;
}

static FORCE_INLINE void i2c_wait_idle(I2C_TypeDef* i2c)
{
    while (testMask(i2c->ISR, I2C_ISR_BUSY)) {}
    if (testMask(i2c->ISR, I2C_ISR_STOPF))
        i2c->ICR = I2C_ICR_STOPCF;
}

static FORCE_INLINE void i2c_start_write(I2C_TypeDef* i2c, const uint8_t addr, const uint8_t n_bytes)
{
    i2c->CR2 = i2c_cr2_write(addr, n_bytes) | I2C_CR2_START;
}

static FORCE_INLINE void i2c_start_write_auto(I2C_TypeDef* i2c, const uint8_t addr, const uint8_t n_bytes)
{
    i2c->CR2 = i2c_cr2_write(addr, n_bytes) | I2C_CR2_AUTOEND | I2C_CR2_START;
}

static FORCE_INLINE void i2c_start_read_auto(I2C_TypeDef* i2c, const uint8_t addr, const uint8_t n_bytes)
{
    i2c->CR2 = i2c_cr2_read(addr, n_bytes) | I2C_CR2_AUTOEND | I2C_CR2_START;
}

static FORCE_INLINE void i2c_finish(I2C_TypeDef* i2c)
{
    i2c_poll(i2c, I2C_ISR_STOPF);
    i2c->ICR = I2C_ICR_STOPCF;
}


void i2c_dma_init(I2CDMABus* bus, I2C_TypeDef* i2c,
                   GPIO_TypeDef* sda_port, const uint8_t sda_pin, const uint8_t sda_af,
                   GPIO_TypeDef* scl_port, const uint8_t scl_pin, const uint8_t scl_af,
                   DMA_TypeDef* dma, DMA_Channel_TypeDef* dma_rx,
                   DMAMUX_Channel_TypeDef* dma_mux_rx,
                   const uint32_t mux_rx, const IRQn_Type dma_rx_irqn) {

    bus->i2c    = i2c;
    bus->dma    = dma;
    bus->dma_rx = dma_rx;
    bus->busy   = false;

    const uint32_t ch = dma_channel_index(dma, dma_rx);
    bus->ifcr_mask = 0xFU << ((ch - 1) * 4);
    bus->rxdr_addr = (uint32_t)&i2c->RXDR;

    setMask(RCC->AHB1ENR, RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN);
    enable_gpio_clock(sda_port);
    enable_gpio_clock(scl_port);

    if      (i2c == I2C1) setMask(RCC->APB1ENR1, RCC_APB1ENR1_I2C1EN);
    else if (i2c == I2C2) setMask(RCC->APB1ENR1, RCC_APB1ENR1_I2C2EN);
    else if (i2c == I2C3) setMask(RCC->APB1ENR1, RCC_APB1ENR1_I2C3EN);
    __DSB();

    gpio_init_i2c_pin(sda_port, sda_pin, sda_af);
    gpio_init_i2c_pin(scl_port, scl_pin, scl_af);

    clearMask(i2c->CR1, I2C_CR1_PE);
    i2c->TIMINGR = I2C_TIMING_FM_400K;
    setMask(i2c->CR1, I2C_CR1_PE);

    dma_rx->CCR   = 0;
    dma_mux_rx->CCR = mux_rx;

    NVIC_SetPriority(dma_rx_irqn, 2);
    NVIC_EnableIRQ(dma_rx_irqn);
}

void i2c_dma_write_reg(const I2CDMABus* bus, const uint8_t addr,
                        const uint8_t reg, const uint8_t value) {
    I2C_TypeDef* i2c = bus->i2c;
    i2c_wait_idle(i2c);

    i2c_start_write_auto(i2c, addr, 2);

    if (!i2c_poll(i2c, I2C_ISR_TXIS)) return;
    i2c->TXDR = reg;

    if (!i2c_poll(i2c, I2C_ISR_TXIS)) return;
    i2c->TXDR = value;

    i2c_finish(i2c);
}

uint8_t i2c_dma_read_reg_blocking(const I2CDMABus* bus, const uint8_t addr,
                                   const uint8_t reg) {
    I2C_TypeDef* i2c = bus->i2c;
    i2c_wait_idle(i2c);

    i2c_start_write(i2c, addr, 1);

    if (!i2c_poll(i2c, I2C_ISR_TXIS)) return 0;
    i2c->TXDR = reg;

    if (!i2c_poll(i2c, I2C_ISR_TC)) return 0;

    i2c_start_read_auto(i2c, addr, 1);

    if (!i2c_poll(i2c, I2C_ISR_RXNE)) return 0;
    const uint8_t val = (uint8_t)i2c->RXDR;

    i2c_finish(i2c);
    return val;
}

void i2c_dma_read_reg(I2CDMABus* bus, const uint8_t addr, const uint8_t reg,
                       volatile uint8_t* buf, const uint8_t len) {
    I2C_TypeDef* i2c = bus->i2c;

    while (bus->busy) {}
    i2c_wait_idle(i2c);

    i2c_start_write(i2c, addr, 1);

    if (!i2c_poll(i2c, I2C_ISR_TXIS)) return;
    i2c->TXDR = reg;

    if (!i2c_poll(i2c, I2C_ISR_TC)) return;

    bus->busy = true;

    bus->dma_rx->CCR   = 0;
    bus->dma_rx->CPAR  = bus->rxdr_addr;
    bus->dma_rx->CMAR  = (uint32_t)buf;
    bus->dma_rx->CNDTR = len;
    bus->dma_rx->CCR   = DMA_CCR_MINC | DMA_CCR_TCIE;

    setMask(i2c->CR1, I2C_CR1_RXDMAEN);
    setMask(bus->dma_rx->CCR, DMA_CCR_EN);

    i2c_start_read_auto(i2c, addr, len);
}

bool i2c_dma_probe(const I2CDMABus* bus, const uint8_t addr) {
    I2C_TypeDef* i2c = bus->i2c;
    i2c_wait_idle(i2c);

    i2c_start_write_auto(i2c, addr, 0);

    const uint32_t isr = i2c_poll_isr(i2c, I2C_ISR_STOPF);
    const bool found = isr & I2C_ISR_STOPF && !(isr & I2C_ISR_NACKF);
    i2c->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF;
    return found;
}