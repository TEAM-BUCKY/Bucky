#include "IRSensor.h"

#include "../optimizations/optimizations.h"
#include "../dma/DMA.h"
#include "../adc/ADC.h"
#include "../bitboard/bitboard.h"

// ---- Register helpers ----

static FORCE_INLINE void gpio_set_af(GPIO_TypeDef* gpio, const uint8_t pin, const uint8_t af) {
    writeField(gpio->MODER, 3U, pin * 2, 2U);
    writeField(gpio->AFR[pin >> 3], 0xFU, (pin & 7) * 4, af);
}

static FORCE_INLINE void gpio_set_analog(GPIO_TypeDef* gpio, const uint8_t pin) {
    writeField(gpio->MODER, 3U, pin * 2, 3U);
}

static FORCE_INLINE void tim_start(TIM_TypeDef* tim) {
    setMask(tim->CR1, TIM_CR1_CEN);
}

static FORCE_INLINE void tim_set_trgo(TIM_TypeDef* tim, const uint32_t mms) {
    tim->CR2 = (tim->CR2 & ~TIM_CR2_MMS_Msk) | (mms << TIM_CR2_MMS_Pos);
}

static FORCE_INLINE void tim_set_dma_burst(TIM_TypeDef* tim, const uint32_t base_reg, const uint32_t count) {
    tim->DCR = (count - 1) << TIM_DCR_DBL_Pos | (base_reg << TIM_DCR_DBA_Pos);
}

static FORCE_INLINE void tim_enable_update_dma(TIM_TypeDef* tim) {
    setMask(tim->DIER, TIM_DIER_UDE);
}

static constexpr uint32_t MMS_OC1REF   = 4;
static constexpr uint32_t MMS_OC2REF   = 5;
static constexpr uint32_t TIM_DBA_ARR  = 11;

// ---- Timer parameters ----
// 170 MHz / (PSC+1) = 56.667 MHz counter clock

static constexpr uint32_t IR_TIM_PSC     = 2;
static constexpr uint32_t IR_FAST_ARR    = 84;    // 1.50us pulse
static constexpr uint32_t IR_FAST_CCR    = 42;    // 50% duty
static constexpr uint32_t IR_SILENCE_ARR =
    (IR_MODE == IRBallMode::MODE_D) ? 45862 : 4542;
static constexpr uint32_t IR_CYCLE_COUNT = IR_SWEEPS_PER_CYCLE * 17;

// Preload lag: DMA write at event N takes effect at period N+2.
// Silence at period 16 -> DMA index 14; repeats every 17 for Mode A.
static constexpr bool is_silence_idx(const uint32_t i) {
    return i % 17 == 14;
}

// ---- Buffers ----

static uint32_t tim4_dma_buf[IR_BOARD1_ENABLED ? IR_CYCLE_COUNT * 3 : 1];
static uint32_t tim3_dma_buf[IR_BOARD2_ENABLED ? IR_CYCLE_COUNT * 4 : 1];
static volatile uint16_t board1_adc_buffer[IR_ADC_BUFFER_SIZE];
static volatile uint16_t board2_adc_buffer[IR_ADC_BUFFER_SIZE];

// ---- Internal helpers ----

static void fill_timer_dma_buf(uint32_t* buf, const uint32_t words_per_entry, const uint32_t ccr_offset) {
    for (uint32_t i = 0; i < IR_CYCLE_COUNT; i++) {
        const uint32_t base = i * words_per_entry;
        const bool silence = is_silence_idx(i);
        buf[base] = silence ? IR_SILENCE_ARR : IR_FAST_ARR;
        for (uint32_t w = 1; w < words_per_entry; w++)
            buf[base + w] = 0;
        if (!silence)
            buf[base + ccr_offset] = IR_FAST_CCR;
    }
}

static void init_timer_pwm(TIM_TypeDef* tim, const uint8_t channel) {
    tim->PSC = IR_TIM_PSC;
    tim->ARR = IR_FAST_ARR;

    // PWM mode 1 + preload enable (OC1M=110, OC1PE=1)
    volatile uint32_t* ccmr = &tim->CCMR1 + (channel >> 1);
    const uint8_t shift = (channel & 1) * 8;
    writeField(*ccmr, 0xFFU, shift, 0x68U);

    (&tim->CCR1)[channel] = IR_FAST_CCR;
    setBit(tim->CCER, channel * 4);
    setMask(tim->CR1, TIM_CR1_ARPE);

    tim->EGR = TIM_EGR_UG;
    tim->SR = 0;
}

template<uint8_t Board> struct BoardCfg;

template<> struct BoardCfg<1> {
    static constexpr uint32_t sensor_count = IR_BOARD1_SENSOR_COUNT;
    static constexpr uint8_t  clk_pin = 6, clk_af = 2, adc_pin = 4;
    static constexpr uint8_t  tim_ch = 0;
    static constexpr uint32_t trgo = MMS_OC1REF, burst_words = 3, ccr_off = 2;
    static constexpr uint32_t adc_ch = 17, extsel = ADCExtSel::TIM4_TRGO;
    static constexpr uint32_t dma_tim_mux = DMAMux::TIM4_UP, dma_adc_mux = DMAMux::ADC_2;

    static auto timer()      { return TIM4; }
    static auto adc()        { return ADC2; }
    static auto clk_gpio()   { return GPIOB; }
    static auto adc_gpio()   { return GPIOA; }
    static auto dma_tim()    { return DMA1_Channel1; }
    static auto dmamux_tim() { return DMAMUX1_Channel0; }
    static auto dma_adc()    { return DMA1_Channel2; }
    static auto dmamux_adc() { return DMAMUX1_Channel1; }
    static auto tim_buf()    { return tim4_dma_buf; }
    static auto adc_buf()    { return board1_adc_buffer; }
};

template<> struct BoardCfg<2> {
    static constexpr uint32_t sensor_count = IR_BOARD2_SENSOR_COUNT;
    static constexpr uint8_t  clk_pin = 5, clk_af = 2, adc_pin = 14;
    static constexpr uint8_t  tim_ch = 1;
    static constexpr uint32_t trgo = MMS_OC2REF, burst_words = 4, ccr_off = 3;
    static constexpr uint32_t adc_ch = 5, extsel = ADCExtSel::TIM3_TRGO;
    static constexpr uint32_t dma_tim_mux = DMAMux::TIM3_UP, dma_adc_mux = DMAMux::ADC_1;

    static auto timer()      { return TIM3; }
    static auto adc()        { return ADC1; }
    static auto clk_gpio()   { return GPIOB; }
    static auto adc_gpio()   { return GPIOB; }
    static auto dma_tim()    { return DMA1_Channel3; }
    static auto dmamux_tim() { return DMAMUX1_Channel2; }
    static auto dma_adc()    { return DMA1_Channel4; }
    static auto dmamux_adc() { return DMAMUX1_Channel3; }
    static auto tim_buf()    { return tim3_dma_buf; }
    static auto adc_buf()    { return board2_adc_buffer; }
};

template<uint8_t Board, bool Enabled>
static void init_board() {
    if constexpr (!Enabled) return;

    using board = BoardCfg<Board>;

    gpio_set_af(board::clk_gpio(), board::clk_pin, board::clk_af);
    gpio_set_analog(board::adc_gpio(), board::adc_pin);

    fill_timer_dma_buf(board::tim_buf(), board::burst_words, board::ccr_off);

    adc_disable(board::adc());
    dma_init_mem_to_periph_32(board::dma_tim(), board::dmamux_tim(),
                              &board::timer()->DMAR, board::tim_buf(),
                              IR_CYCLE_COUNT * board::burst_words, board::dma_tim_mux);
    dma_init_periph_to_mem_16(board::dma_adc(), board::dmamux_adc(),
                              &board::adc()->DR, board::adc_buf(),
                              IR_ADC_BUFFER_SIZE, board::dma_adc_mux);

    dma_enable(board::dma_tim());
    dma_enable(board::dma_adc());

    adc_init_triggered(board::adc(), board::adc_ch, board::extsel);

    init_timer_pwm(board::timer(), board::tim_ch);
    tim_set_trgo(board::timer(), board::trgo);
    tim_set_dma_burst(board::timer(), TIM_DBA_ARR, board::burst_words);
    tim_enable_update_dma(board::timer());
    tim_start(board::timer());
}

volatile uint16_t* ir_get_buffer(const uint8_t board) {
    return board == 1 ? board1_adc_buffer : board2_adc_buffer;
}

uint32_t ir_get_sensor_count(const uint8_t board) {
    return board == 1 ? BoardCfg<1>::sensor_count : BoardCfg<2>::sensor_count;
}

void ir_sensor_init() {
    setMask(RCC->AHB1ENR,  RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN);
    setMask(RCC->AHB2ENR,  RCC_AHB2ENR_ADC12EN | RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN);
    setMask(RCC->APB1ENR1, RCC_APB1ENR1_TIM3EN | RCC_APB1ENR1_TIM4EN);
    __DSB();

    ADC12_COMMON->CCR = (ADC12_COMMON->CCR & ~ADC_CCR_CKMODE_Msk)
                      | (3U << ADC_CCR_CKMODE_Pos); // AHB/4 = 42.5 MHz

    init_board<1, IR_BOARD1_ENABLED>();
    init_board<2, IR_BOARD2_ENABLED>();
}
