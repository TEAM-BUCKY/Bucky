#include "I2CDMA.h"

#include <PeripheralPins.h>
#include <PortNames.h>
#include <pinmap.h>

#include "hardware/io/irq/IRQ.h"
#include "optimizations/bitboard.h"

#define I2C_TIMEOUT (F_CPU / 1000U)  /* ~1 ms of polling */

static FORCE_INLINE uint32_t i2c_cr2_write(const uint8_t addr, const uint8_t n_bytes) {
    return (uint32_t)addr << 1 | (uint32_t)n_bytes << I2C_CR2_NBYTES_Pos;
}

static FORCE_INLINE uint32_t i2c_cr2_read(const uint8_t addr, const uint8_t n_bytes) {
    return i2c_cr2_write(addr, n_bytes) | I2C_CR2_RD_WRN;
}

static void gpio_init_i2c_pin(const PinName pin, const PinMap* map) {
    GPIO_TypeDef* port = set_GPIO_Port_Clock(STM_PORT(pin));
    const uint32_t n  = STM_PIN(pin);
    const uint32_t af = STM_PIN_AFNUM(pinmap_function(pin, map));
    writeField(port->AFR[n >> 3], 0xFU, (n & 7) * 4, af);
    writeField(port->MODER,   3U, n * 2, 2U);
    setBit(port->OTYPER, n);
    writeField(port->PUPDR,   3U, n * 2, 1U);
    writeField(port->OSPEEDR, 3U, n * 2, 3U);
}

static void i2c_clock_enable(const I2C_TypeDef* i2c) {
    if      (i2c == I2C1) __HAL_RCC_I2C1_CLK_ENABLE();
    else if (i2c == I2C2) __HAL_RCC_I2C2_CLK_ENABLE();
#ifdef I2C3
    else if (i2c == I2C3) __HAL_RCC_I2C3_CLK_ENABLE();
#endif
#ifdef I2C4
    else if (i2c == I2C4) __HAL_RCC_I2C4_CLK_ENABLE();
#endif
}

/* Fast-mode Plus needs the 20 mA pad drivers switched on. */
static void i2c_enable_fmp(const I2C_TypeDef* i2c, const PinName sda, const PinName scl) {
#if defined(MCU_FAMILY_G4)
    (void)sda; (void)scl;
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    if      (i2c == I2C1) setMask(SYSCFG->CFGR1, SYSCFG_CFGR1_I2C1_FMP);
    else if (i2c == I2C2) setMask(SYSCFG->CFGR1, SYSCFG_CFGR1_I2C2_FMP);
    else if (i2c == I2C3) setMask(SYSCFG->CFGR1, SYSCFG_CFGR1_I2C3_FMP);
#elif defined(MCU_FAMILY_H5)
    /* H5 only has FM+ drivers on PB6..PB9; other pads run at standard drive. */
    (void)i2c;
    __HAL_RCC_SBS_CLK_ENABLE();
    const PinName pins[2] = {sda, scl};
    for (int i = 0; i < 2; i++) {
        const PinName pad = (PinName)(pins[i] & PNAME_MASK);
        if (pad >= PB_6 && pad <= PB_9)
            setMask(SBS->PMCR, SBS_PMCR_PB6_FMP << (pad - PB_6));
    }
#endif
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


static void i2c_dma_rx_complete(void* ctx) {
    I2CDMABus* bus = (I2CDMABus*)ctx;
    dma_take_events(bus->dma_rx);
    clearMask(bus->i2c->CR1, I2C_CR1_RXDMAEN);
    bus->busy = false;
}

void i2c_dma_init_raw(I2CDMABus* bus, I2C_TypeDef* i2c, const PinName sda, const PinName scl,
                      DmaChannel* dma_rx, const uint32_t rx_request,
                      const uint32_t timing, const uint8_t irq_priority) {

    bus->i2c        = i2c;
    bus->dma_rx     = dma_rx;
    bus->rx_request = rx_request;
    bus->busy       = false;

    i2c_clock_enable(i2c);
    __DSB();

    gpio_init_i2c_pin(sda, PinMap_I2C_SDA);
    gpio_init_i2c_pin(scl, PinMap_I2C_SCL);

    if (timing != I2C_TIMING_FM_400K)
        i2c_enable_fmp(i2c, sda, scl);

    clearMask(i2c->CR1, I2C_CR1_PE);
    i2c->TIMINGR = timing;
    setMask(i2c->CR1, I2C_CR1_PE);

    dma_reset(dma_rx);
    irq_attach(dma_irqn(dma_rx), i2c_dma_rx_complete, bus, irq_priority);
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

    dma_setup(bus->dma_rx, DMA_PERIPH_TO_MEM, DMA_WIDTH_8, &i2c->RXDR, buf, len,
              bus->rx_request, false);
    dma_enable_events(bus->dma_rx, DMA_EVT_TC);

    setMask(i2c->CR1, I2C_CR1_RXDMAEN);
    dma_enable(bus->dma_rx);

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