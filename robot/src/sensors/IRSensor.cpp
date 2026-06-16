#include "sensors/IRSensor.h"

#include "optimizations/optimizations.h"
#include "io/dma/DMA.h"
#include "io/adc/ADC.h"
#include "optimizations/bitboard.h"

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
    tim->CR2 = tim->CR2 & ~TIM_CR2_MMS_Msk | mms << TIM_CR2_MMS_Pos;
}

static FORCE_INLINE void tim_set_dma_burst(TIM_TypeDef* tim, const uint32_t base_reg, const uint32_t count) {
    tim->DCR = (count - 1) << TIM_DCR_DBL_Pos | base_reg << TIM_DCR_DBA_Pos;
}

static FORCE_INLINE void tim_enable_update_dma(TIM_TypeDef* tim) {
    setMask(tim->DIER, TIM_DIER_UDE);
}

static constexpr uint32_t MMS_OC1REF   = 4;
static constexpr uint32_t MMS_OC2REF   = 5;
static constexpr uint32_t TIM_DBA_ARR  = 11;

// ---- Timer parameters (DMA burst mode, Board 1) ----
// 170 MHz / (PSC+1) = 56.667 MHz counter clock

static constexpr uint32_t IR_TIM_PSC     = 2;
static constexpr uint32_t IR_FAST_ARR    = 84;    // 1.50us pulse
static constexpr uint32_t IR_FAST_CCR    = 42;    // 50% duty
static constexpr uint32_t IR_SILENCE_ARR =
    (IR_MODE == IRBallMode::MODE_D) ? 45862 : 4542;
static constexpr uint32_t IR_CYCLE_COUNT = IR_SWEEPS_PER_CYCLE * 17;

// Preload lag: DMA write at event N takes effect at period N+1.
// Silence after all 16 pulses -> DMA index 15; repeats every 17 for Mode A.
static constexpr bool is_silence_idx(const uint32_t i) {
    return i % 17 == 15;
}

// ---- Timer parameters (gated mode, Board 2) ----
// TIM2 master: 1 MHz tick → 925 µs cycle, 240 µs gate window
// TIM3 slave:  170 MHz → 16 pulses @ 15 µs during gate window
static constexpr uint32_t IR_GATE_PSC = 169;   // 170 MHz / 170 = 1 MHz
static constexpr uint32_t IR_GATE_ARR = 924;   // 925 µs total period
static constexpr uint32_t IR_GATE_CCR = 240;   // 240 µs gate HIGH
static constexpr uint32_t IR_CLK_ARR  = 2549;  // 2550 / 170 MHz = 15 µs
static constexpr uint32_t IR_CLK_CCR  = 1274;  // 50% duty

static uint32_t tim4_dma_buf[IR_BOARD1_ENABLED ? IR_CYCLE_COUNT * 3 : 1];

// Board 2 gated mode: accumulate IR_SWEEPS_PER_CYCLE gate scans
// so IRBallProcessor can take max across sweeps, matching Board 1's design.
static constexpr uint32_t IR_BOARD2_ADC_HALF = IR_SWEEPS_PER_CYCLE * IR_MUX_CHANNELS;

static volatile uint16_t board1_dma_buf[IR_BOARD1_ENABLED ? 2 * IR_ADC_BUFFER_SIZE : 1];
static volatile uint16_t board2_dma_buf[IR_BOARD2_ENABLED ? 2 * IR_BOARD2_ADC_HALF : 1];

static volatile uint16_t* volatile board1_ready = board1_dma_buf;
static volatile uint16_t* volatile board2_ready = board2_dma_buf;
static volatile uint32_t board1_frame_seq = 0;
static volatile uint32_t board2_frame_seq = 0;

static constexpr uint32_t DMA_HTIF(const uint32_t ch) { return 1U << ((ch - 1) * 4 + 2); }
static constexpr uint32_t DMA_TCIF(const uint32_t ch) { return 1U << ((ch - 1) * 4 + 1); }

extern "C" {

void DMA1_Channel2_IRQHandler(void) {
    const uint32_t isr = DMA1->ISR;
    if (isr & DMA_HTIF(2)) {
        board1_ready = board1_dma_buf;
        ++board1_frame_seq;
        DMA1->IFCR = DMA_HTIF(2);
    }
    if (isr & DMA_TCIF(2)) {
        board1_ready = board1_dma_buf + IR_ADC_BUFFER_SIZE;
        ++board1_frame_seq;
        DMA1->IFCR = DMA_TCIF(2);
    }
}

void DMA1_Channel4_IRQHandler(void) {
    const uint32_t isr = DMA1->ISR;
    if (isr & DMA_HTIF(4)) {
        board2_ready = board2_dma_buf;
        ++board2_frame_seq;
        DMA1->IFCR = DMA_HTIF(4);
    }
    if (isr & DMA_TCIF(4)) {
        board2_ready = board2_dma_buf + IR_BOARD2_ADC_HALF;
        ++board2_frame_seq;
        DMA1->IFCR = DMA_TCIF(4);
    }
}

}

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

    volatile uint32_t* ccmr = &tim->CCMR1 + (channel >> 1);
    const uint8_t shift = (channel & 1) * 8;
    writeField(*ccmr, 0xFFU, shift, 0x68U);

    (&tim->CCR1)[channel] = IR_FAST_CCR;
    setBit(tim->CCER, channel * 4);
    setMask(tim->CR1, TIM_CR1_ARPE);

    tim->EGR = TIM_EGR_UG;
    tim->SR = 0;
}

// ---- TIM2: gate master for Board 2 ----
// Generates 925 µs repetition cycle with 24 µs HIGH gate window.
// OC1REF sent as TRGO to gate TIM3 via internal trigger.
static void init_tim2_gate() {
    TIM2->PSC = IR_GATE_PSC;
    TIM2->ARR = IR_GATE_ARR;

    // PWM Mode 1 on CH1 (OC1M=110, OC1PE=1)
    writeField(TIM2->CCMR1, 0xFFU, 0, 0x68U);
    TIM2->CCR1 = IR_GATE_CCR;

    // MMS = OC1REF: TRGO follows gate window
    tim_set_trgo(TIM2, MMS_OC1REF);
    setMask(TIM2->CR1, TIM_CR1_ARPE);

    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0;
}

// ---- TIM3: gated slave for Board 2 ----
// Generates 16 clock pulses at ~1.5 µs period while TIM2 gate is HIGH.
// OC2REF as TRGO triggers ADC on each pulse rising edge.
// Physical output on PB5 (TIM3_CH2 AF2) drives 74HC4040 CLK.
static void init_tim3_gated() {
    TIM3->PSC = 0;
    TIM3->ARR = IR_CLK_ARR;

    // PWM Mode 1 on CH2 (OC2M=110, OC2PE=1)
    writeField(TIM3->CCMR1, 0xFFU, 8, 0x68U);
    TIM3->CCR2 = IR_CLK_CCR;

    // Enable CH2 output
    setBit(TIM3->CCER, 4);

    setMask(TIM3->CR1, TIM_CR1_ARPE);

    // Gated slave mode (SMS=0101) with ITR1 (TS=001) = TIM2_TRGO
    // RM0440 Table 254: TIM3 ITR1 = TIM2_TRGO
    TIM3->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_2
               | TIM_SMCR_TS_0;

    // MMS = OC2REF: TRGO triggers ADC on each clock pulse
    tim_set_trgo(TIM3, MMS_OC2REF);

    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR = 0;
}

template<uint8_t Board> struct BoardCfg;

template<> struct BoardCfg<1> {
    static constexpr bool     gated = false;
    static constexpr uint32_t sensor_count = IR_BOARD1_SENSOR_COUNT;
    static constexpr uint32_t adc_half = IR_ADC_BUFFER_SIZE;
    static constexpr uint8_t  clk_pin = 6, clk_af = 2, adc_pin = 4;
    static constexpr uint8_t  tim_ch = 0;
    static constexpr uint32_t trgo = MMS_OC1REF, burst_words = 3, ccr_off = 2;
    static constexpr uint32_t adc_ch = 17, extsel = ADC_EXTSEL_TIM4_TRGO, adc_smp = 2;
    static constexpr uint32_t dma_tim_mux = DMAMUX_REQ_TIM4_UP, dma_adc_mux = DMAMUX_REQ_ADC_2;

    static auto timer()      { return TIM4; }
    static auto adc()        { return ADC2; }
    static auto clk_gpio()   { return GPIOB; }
    static auto adc_gpio()   { return GPIOA; }
    static auto dma_tim()    { return DMA1_Channel1; }
    static auto dmamux_tim() { return DMAMUX1_Channel0; }
    static auto dma_adc()    { return DMA1_Channel2; }
    static auto dmamux_adc() { return DMAMUX1_Channel1; }
    static constexpr IRQn_Type dma_adc_irqn = DMA1_Channel2_IRQn;
    static auto tim_buf()    { return tim4_dma_buf; }
    static auto adc_buf()    { return board1_dma_buf; }
};

template<> struct BoardCfg<2> {
    static constexpr bool     gated = true;
    static constexpr uint32_t sensor_count = IR_BOARD2_SENSOR_COUNT;
    static constexpr uint32_t adc_half = IR_BOARD2_ADC_HALF;
    static constexpr uint8_t  clk_pin = 5, clk_af = 2, adc_pin = 14;
    static constexpr uint8_t  tim_ch = 1;
    static constexpr uint32_t trgo = MMS_OC2REF, burst_words = 4, ccr_off = 3;
    static constexpr uint32_t adc_ch = 4, extsel = ADC_EXTSEL_TIM3_TRGO, adc_smp = 4;
    static constexpr uint32_t dma_adc_mux = DMAMUX_REQ_ADC_4;

    static auto timer()      { return TIM3; }
    static auto adc()        { return ADC4; }
    static auto clk_gpio()   { return GPIOB; }
    static auto adc_gpio()   { return GPIOB; }
    static auto dma_adc()    { return DMA1_Channel4; }
    static auto dmamux_adc() { return DMAMUX1_Channel3; }
    static constexpr IRQn_Type dma_adc_irqn = DMA1_Channel4_IRQn;
    static auto adc_buf()    { return board2_dma_buf; }
};

template<uint8_t Board, bool Enabled>
static void init_board() {
    if constexpr (!Enabled) return;

    using board = BoardCfg<Board>;

    gpio_set_af(board::clk_gpio(), board::clk_pin, board::clk_af);
    gpio_set_analog(board::adc_gpio(), board::adc_pin);

    adc_disable(board::adc());

    if constexpr (!board::gated) {
        fill_timer_dma_buf(board::tim_buf(), board::burst_words, board::ccr_off);
        dma_init_mem_to_periph_32(board::dma_tim(), board::dmamux_tim(),
                                  &board::timer()->DMAR, board::tim_buf(),
                                  IR_CYCLE_COUNT * board::burst_words, board::dma_tim_mux);
    }

    dma_init_periph_to_mem_16(board::dma_adc(), board::dmamux_adc(),
                              &board::adc()->DR, board::adc_buf(),
                              2 * board::adc_half, board::dma_adc_mux);

    setMask(board::dma_adc()->CCR, DMA_CCR_HTIE | DMA_CCR_TCIE);
    NVIC_SetPriority(board::dma_adc_irqn, 3);
    NVIC_EnableIRQ(board::dma_adc_irqn);

    if constexpr (!board::gated)
        dma_enable(board::dma_tim());
    dma_enable(board::dma_adc());

    adc_init_triggered(board::adc(), board::adc_ch, board::extsel, board::adc_smp);

    if constexpr (board::gated) {
        // TIM2→TIM3 gated mode: start master first, then slave
        init_tim2_gate();
        init_tim3_gated();
        tim_start(TIM2);
        tim_start(board::timer());
    } else {
        // DMA burst mode: timer self-modulates ARR/CCR via DMA
        init_timer_pwm(board::timer(), board::tim_ch);
        tim_set_trgo(board::timer(), board::trgo);
        tim_set_dma_burst(board::timer(), TIM_DBA_ARR, board::burst_words);
        tim_enable_update_dma(board::timer());
        tim_start(board::timer());
    }
}

const uint16_t* ir_get_buffer(const uint8_t board) {
    return const_cast<const uint16_t*>(board == 1 ? board1_ready : board2_ready);
}

uint32_t ir_get_sensor_count(const uint8_t board) {
    return board == 1 ? BoardCfg<1>::sensor_count : BoardCfg<2>::sensor_count;
}

void ir_sensor_init() {
    setMask(RCC->AHB1ENR,  RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN);
    setMask(RCC->AHB2ENR,  RCC_AHB2ENR_ADC12EN | RCC_AHB2ENR_ADC345EN
                          | RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN);
    setMask(RCC->APB1ENR1, RCC_APB1ENR1_TIM2EN | RCC_APB1ENR1_TIM3EN | RCC_APB1ENR1_TIM4EN);
    setMask(RCC->APB2ENR,  RCC_APB2ENR_SYSCFGEN);
    __DSB();

    // PB14 is OPAMP2_VINP and OPAMP5_VINP. Even disabled, each OPAMP's
    // input mux leaks ~18MΩ to VDD, pulling ADC readings toward 4095.
    // VP_SEL=11 routes to internal DAC, disconnecting PB14 from both.
    OPAMP2->CSR = OPAMP_CSR_VPSEL_1 | OPAMP_CSR_VPSEL_0;  // VP_SEL=11
    OPAMP5->CSR = OPAMP_CSR_VPSEL_1 | OPAMP_CSR_VPSEL_0;  // VP_SEL=11

    ADC12_COMMON->CCR = ADC12_COMMON->CCR & ~ADC_CCR_CKMODE_Msk
        | 3U << ADC_CCR_CKMODE_Pos;

    ADC345_COMMON->CCR = ADC345_COMMON->CCR & ~ADC_CCR_CKMODE_Msk
        | 3U << ADC_CCR_CKMODE_Pos;

    init_board<1, IR_BOARD1_ENABLED>();
    init_board<2, IR_BOARD2_ENABLED>();
}

uint32_t ir_get_frame_sequence(const uint8_t board)
{
    return board == 1 ? board1_frame_seq : board2_frame_seq;
}

bool ir_has_new_frame(const uint8_t board, const uint32_t lastSequence)
{
    return ir_get_frame_sequence(board) != lastSequence;
}
