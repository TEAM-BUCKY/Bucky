/* Host stand-in for the STM32H5 CMSIS device header.
 *
 * Peripheral register blocks are plain RAM structs with the exact CMSIS field order (static
 * asserts below), and every instance macro (TIM1, GPIOA, ADC1, …) points at a simulator-owned
 * object. The firmware's inline register code (gpio.h, pwm.h, Timer.h) therefore runs unchanged;
 * the simulator observes the registers lazily, or through the GPIO write hooks for BSRR/ODR.
 *
 * Deliberately NOT defined: CORDIC (cordic.c builds its libm fallback) and HRTIM1 (H5 has none).
 * Values that come from the real headers (request numbers, trigger encodings, bit positions)
 * are copied from framework-arduinoststm32 4.21200.0, stm32h562xx.h / stm32h5xx_hal_*.h.
 */
#ifndef BUCKY_SIM_STM32H5XX_H
#define BUCKY_SIM_STM32H5XX_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define STM32H5xx_SIM 1

#ifndef __IO
#define __IO volatile
#endif
#ifndef __I
#define __I volatile const
#endif

#include "cmsis_sim.h"

/* ---- Interrupt numbers (stm32h562xx.h) -------------------------------------------------- */
typedef enum {
    SysTick_IRQn = -1,
    EXTI0_IRQn = 11, EXTI1_IRQn, EXTI2_IRQn, EXTI3_IRQn, EXTI4_IRQn, EXTI5_IRQn, EXTI6_IRQn,
    EXTI7_IRQn, EXTI8_IRQn, EXTI9_IRQn, EXTI10_IRQn, EXTI11_IRQn, EXTI12_IRQn, EXTI13_IRQn,
    EXTI14_IRQn, EXTI15_IRQn,
    GPDMA1_Channel0_IRQn = 27, GPDMA1_Channel1_IRQn, GPDMA1_Channel2_IRQn, GPDMA1_Channel3_IRQn,
    GPDMA1_Channel4_IRQn, GPDMA1_Channel5_IRQn, GPDMA1_Channel6_IRQn, GPDMA1_Channel7_IRQn,
    ADC1_IRQn = 37,
    TIM1_CC_IRQn = 44, TIM2_IRQn = 45, TIM3_IRQn = 46, TIM4_IRQn = 47,
    I2C2_EV_IRQn = 53, USART2_IRQn = 59, LPTIM1_IRQn = 64, ADC2_IRQn = 69,
    TIM15_IRQn = 71, TIM16_IRQn = 72, TIM17_IRQn = 73, USB_DRD_FS_IRQn = 74,
    TIM12_IRQn = 120, TIM13_IRQn = 121,
} IRQn_Type;

#define USB_IRQn USB_DRD_FS_IRQn

/* ---- Register blocks -------------------------------------------------------------------- */

#ifdef __cplusplus
extern "C" {
#endif
/* Write hooks for GPIO output registers (sim/src/io/gpio.cpp). */
void sim_gpio_odr_written(volatile void* odr_reg, uint32_t old_value, uint32_t new_value);
void sim_gpio_bsrr_written(volatile void* bsrr_reg, uint32_t value);
#ifdef __cplusplus
}

/* ODR and BSRR report every write so pin edges (sonar trigger, manually stepped G-port clock)
 * reach the simulator immediately. Same size and layout as a uint32_t register. */
struct SimGpioOdr {
    uint32_t v;
    operator uint32_t() const volatile { return v; }
    void set(uint32_t n) volatile { const uint32_t o = v; v = n; sim_gpio_odr_written(this, o, n); }
    SimGpioOdr& operator=(uint32_t n) volatile { set(n); return const_cast<SimGpioOdr&>(*this); }
    SimGpioOdr& operator|=(uint32_t n) volatile { set(v | n); return const_cast<SimGpioOdr&>(*this); }
    SimGpioOdr& operator&=(uint32_t n) volatile { set(v & n); return const_cast<SimGpioOdr&>(*this); }
    SimGpioOdr& operator^=(uint32_t n) volatile { set(v ^ n); return const_cast<SimGpioOdr&>(*this); }
};

struct SimGpioBsrr {
    uint32_t v;
    operator uint32_t() const volatile { return 0; }   /* write-only */
    SimGpioBsrr& operator=(uint32_t n) volatile {
        sim_gpio_bsrr_written(this, n);
        return const_cast<SimGpioBsrr&>(*this);
    }
};
#define SIM_GPIO_ODR_T  SimGpioOdr
#define SIM_GPIO_BSRR_T SimGpioBsrr
#else
#define SIM_GPIO_ODR_T  __IO uint32_t
#define SIM_GPIO_BSRR_T __IO uint32_t
#endif

typedef struct {
    __IO uint32_t MODER;
    __IO uint32_t OTYPER;
    __IO uint32_t OSPEEDR;
    __IO uint32_t PUPDR;
    __IO uint32_t IDR;
    SIM_GPIO_ODR_T ODR;
    SIM_GPIO_BSRR_T BSRR;
    __IO uint32_t LCKR;
    __IO uint32_t AFR[2];
    __IO uint32_t BRR;
    __IO uint32_t HSLVR;
    __IO uint32_t SECCFGR;
} GPIO_TypeDef;

typedef struct {
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t SMCR;
    __IO uint32_t DIER;
    __IO uint32_t SR;
    __IO uint32_t EGR;
    __IO uint32_t CCMR1;
    __IO uint32_t CCMR2;
    __IO uint32_t CCER;
    __IO uint32_t CNT;
    __IO uint32_t PSC;
    __IO uint32_t ARR;
    __IO uint32_t RCR;
    __IO uint32_t CCR1;
    __IO uint32_t CCR2;
    __IO uint32_t CCR3;
    __IO uint32_t CCR4;
    __IO uint32_t BDTR;
    __IO uint32_t CCR5;
    __IO uint32_t CCR6;
    __IO uint32_t CCMR3;
    __IO uint32_t DTR2;
    __IO uint32_t ECR;
    __IO uint32_t TISEL;
    __IO uint32_t AF1;
    __IO uint32_t AF2;
    __IO uint32_t OR1;
    uint32_t RESERVED0[220];
    __IO uint32_t DCR;
    __IO uint32_t DMAR;
} TIM_TypeDef;

typedef struct {
    __IO uint32_t ISR;
    __IO uint32_t ICR;
    __IO uint32_t DIER;
    __IO uint32_t CFGR;
    __IO uint32_t CR;
    __IO uint32_t CCR1;
    __IO uint32_t ARR;
    __IO uint32_t CNT;
    __IO uint32_t RESERVED0;
    __IO uint32_t CFGR2;
    __IO uint32_t RCR;
    __IO uint32_t CCMR1;
    __IO uint32_t RESERVED1;
    __IO uint32_t CCR2;
} LPTIM_TypeDef;

typedef struct {
    __IO uint32_t ISR;
    __IO uint32_t IER;
    __IO uint32_t CR;
    __IO uint32_t CFGR;
    __IO uint32_t CFGR2;
    __IO uint32_t SMPR1;
    __IO uint32_t SMPR2;
    uint32_t RESERVED1;
    __IO uint32_t TR1;
    __IO uint32_t TR2;
    __IO uint32_t TR3;
    uint32_t RESERVED2;
    __IO uint32_t SQR1;
    __IO uint32_t SQR2;
    __IO uint32_t SQR3;
    __IO uint32_t SQR4;
    __IO uint32_t DR;
    uint32_t RESERVED3;
    uint32_t RESERVED4;
    __IO uint32_t JSQR;
    uint32_t RESERVED5[4];
    __IO uint32_t OFR1;
    __IO uint32_t OFR2;
    __IO uint32_t OFR3;
    __IO uint32_t OFR4;
    uint32_t RESERVED6[4];
    __IO uint32_t JDR1;
    __IO uint32_t JDR2;
    __IO uint32_t JDR3;
    __IO uint32_t JDR4;
    uint32_t RESERVED7[4];
    __IO uint32_t AWD2CR;
    __IO uint32_t AWD3CR;
    uint32_t RESERVED8;
    uint32_t RESERVED9;
    __IO uint32_t DIFSEL;
    __IO uint32_t CALFACT;
    uint32_t RESERVED10[4];
    __IO uint32_t OR;
} ADC_TypeDef;

typedef struct {
    __IO uint32_t CSR;
    uint32_t RESERVED1;
    __IO uint32_t CCR;
    __IO uint32_t CDR;
} ADC_Common_TypeDef;

typedef struct {
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t OAR1;
    __IO uint32_t OAR2;
    __IO uint32_t TIMINGR;
    __IO uint32_t TIMEOUTR;
    __IO uint32_t ISR;
    __IO uint32_t ICR;
    __IO uint32_t PECR;
    __IO uint32_t RXDR;
    __IO uint32_t TXDR;
} I2C_TypeDef;

typedef struct {
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t CR3;
    __IO uint32_t BRR;
    __IO uint32_t GTPR;
    __IO uint32_t RTOR;
    __IO uint32_t RQR;
    __IO uint32_t ISR;
    __IO uint32_t ICR;
    __IO uint32_t RDR;
    __IO uint32_t TDR;
    __IO uint32_t PRESC;
} USART_TypeDef;

typedef struct {
    __IO uint32_t CLBAR;
    uint32_t RESERVED1[2];
    __IO uint32_t CFCR;
    __IO uint32_t CSR;
    __IO uint32_t CCR;
    uint32_t RESERVED2[10];
    __IO uint32_t CTR1;
    __IO uint32_t CTR2;
    __IO uint32_t CBR1;
    __IO uint32_t CSAR;
    __IO uint32_t CDAR;
    __IO uint32_t CTR3;
    __IO uint32_t CBR2;
    uint32_t RESERVED3[8];
    __IO uint32_t CLLR;
} DMA_Channel_TypeDef;

#ifdef __cplusplus
static_assert(offsetof(GPIO_TypeDef, ODR) == 0x14 && offsetof(GPIO_TypeDef, BSRR) == 0x18,
              "GPIO layout");
static_assert(offsetof(TIM_TypeDef, CNT) == 0x24 && offsetof(TIM_TypeDef, CCR1) == 0x34 &&
              offsetof(TIM_TypeDef, CCR4) == 0x40 && offsetof(TIM_TypeDef, DCR) == 0x3DC,
              "TIM layout");
static_assert(offsetof(ADC_TypeDef, DR) == 0x40, "ADC layout");
static_assert(offsetof(LPTIM_TypeDef, CCR2) == 0x34, "LPTIM layout");
#endif

/* ---- Simulator-owned instances ------------------------------------------------------------ */
#ifdef __cplusplus
extern "C" {
#endif
extern GPIO_TypeDef sim_GPIOA, sim_GPIOB, sim_GPIOC, sim_GPIOD, sim_GPIOE, sim_GPIOF, sim_GPIOG,
                    sim_GPIOH, sim_GPIOI;
extern TIM_TypeDef sim_TIM1, sim_TIM2, sim_TIM3, sim_TIM4, sim_TIM5, sim_TIM6, sim_TIM7, sim_TIM8,
                   sim_TIM12, sim_TIM13, sim_TIM14, sim_TIM15, sim_TIM16, sim_TIM17;
extern LPTIM_TypeDef sim_LPTIM1, sim_LPTIM2, sim_LPTIM3, sim_LPTIM4, sim_LPTIM5, sim_LPTIM6;
extern ADC_TypeDef sim_ADC1, sim_ADC2;
extern ADC_Common_TypeDef sim_ADC12_COMMON;
extern I2C_TypeDef sim_I2C1, sim_I2C2, sim_I2C3, sim_I2C4;
extern USART_TypeDef sim_USART1, sim_USART2, sim_USART3, sim_UART4, sim_UART5;
extern DMA_Channel_TypeDef sim_GPDMA1_Channel[8];
#ifdef __cplusplus
}
#endif

/* PortNames.h numbers the ports by which GPIOx_BASE macros exist (A..I on the H562). */
#define GPIOA_BASE 1
#define GPIOB_BASE 1
#define GPIOC_BASE 1
#define GPIOD_BASE 1
#define GPIOE_BASE 1
#define GPIOF_BASE 1
#define GPIOG_BASE 1
#define GPIOH_BASE 1
#define GPIOI_BASE 1

#define GPIOA (&sim_GPIOA)
#define GPIOB (&sim_GPIOB)
#define GPIOC (&sim_GPIOC)
#define GPIOD (&sim_GPIOD)
#define GPIOE (&sim_GPIOE)
#define GPIOF (&sim_GPIOF)
#define GPIOG (&sim_GPIOG)
#define GPIOH (&sim_GPIOH)
#define GPIOI (&sim_GPIOI)

#define TIM1  (&sim_TIM1)
#define TIM2  (&sim_TIM2)
#define TIM3  (&sim_TIM3)
#define TIM4  (&sim_TIM4)
#define TIM5  (&sim_TIM5)
#define TIM6  (&sim_TIM6)
#define TIM7  (&sim_TIM7)
#define TIM8  (&sim_TIM8)
#define TIM12 (&sim_TIM12)
#define TIM13 (&sim_TIM13)
#define TIM14 (&sim_TIM14)
#define TIM15 (&sim_TIM15)
#define TIM16 (&sim_TIM16)
#define TIM17 (&sim_TIM17)

#define LPTIM1 (&sim_LPTIM1)
#define LPTIM2 (&sim_LPTIM2)
#define LPTIM3 (&sim_LPTIM3)
#define LPTIM4 (&sim_LPTIM4)
#define LPTIM5 (&sim_LPTIM5)
#define LPTIM6 (&sim_LPTIM6)

#define ADC1 (&sim_ADC1)
#define ADC2 (&sim_ADC2)
#define ADC12_COMMON (&sim_ADC12_COMMON)

#define I2C1 (&sim_I2C1)
#define I2C2 (&sim_I2C2)
#define I2C3 (&sim_I2C3)
#define I2C4 (&sim_I2C4)

#define USART1 (&sim_USART1)
#define USART2 (&sim_USART2)
#define USART3 (&sim_USART3)
#define UART4  (&sim_UART4)
#define UART5  (&sim_UART5)

#define GPDMA1 1
#define GPDMA1_Channel0 (&sim_GPDMA1_Channel[0])
#define GPDMA1_Channel1 (&sim_GPDMA1_Channel[1])
#define GPDMA1_Channel2 (&sim_GPDMA1_Channel[2])
#define GPDMA1_Channel3 (&sim_GPDMA1_Channel[3])
#define GPDMA1_Channel4 (&sim_GPDMA1_Channel[4])
#define GPDMA1_Channel5 (&sim_GPDMA1_Channel[5])
#define GPDMA1_Channel6 (&sim_GPDMA1_Channel[6])
#define GPDMA1_Channel7 (&sim_GPDMA1_Channel[7])

/* ---- Bit definitions used by the kept firmware code --------------------------------------- */
#define TIM_CR1_CEN            (0x1UL << 0)
#define TIM_CR1_ARPE           (0x1UL << 7)
#define TIM_CR2_MMS_Pos        (4U)
#define TIM_CR2_MMS_Msk        (0x200007UL << TIM_CR2_MMS_Pos)
#define TIM_SMCR_SMS_Pos       (0U)
#define TIM_SMCR_SMS_Msk       (0x10007UL << TIM_SMCR_SMS_Pos)
#define TIM_SMCR_SMS_0         (0x00001UL << TIM_SMCR_SMS_Pos)
#define TIM_EGR_UG             (0x1UL << 0)
#define TIM_CCMR1_CC1S_Pos     (0U)
#define TIM_CCMR1_CC1S_0       (0x1UL << TIM_CCMR1_CC1S_Pos)
#define TIM_CCMR1_CC2S_Pos     (8U)
#define TIM_CCMR1_CC2S_0       (0x1UL << TIM_CCMR1_CC2S_Pos)
#define TIM_CCMR1_IC1F_Pos     (4U)
#define TIM_CCMR1_IC2F_Pos     (12U)
#define TIM_BDTR_MOE           (0x1UL << 15)

#define LPTIM_CR_ENABLE        (0x1UL << 0)
#define LPTIM_CR_CNTSTRT       (0x1UL << 2)
#define LPTIM_CFGR_PRESC_Pos   (9U)
#define LPTIM_CFGR_PRESC_Msk   (0x7UL << LPTIM_CFGR_PRESC_Pos)
#define LPTIM_CCMR1_CC1E       (0x1UL << 1)
#define LPTIM_CCMR1_CC2E       (0x1UL << 17)

#define ADC_CFGR_EXTSEL_Pos    (5U)
#define ADC_CFGR_EXTSEL_Msk    (0x1FUL << ADC_CFGR_EXTSEL_Pos)
#define ADC_CFGR_EXTEN_Pos     (10U)
#define ADC_CFGR_EXTEN_0       (0x1UL << ADC_CFGR_EXTEN_Pos)
#define ADC_CR_ADEN            (0x1UL << 0)

/* HAL ADC trigger encodings (LL_ADC_REG_TRIG_EXT_*: EXTSEL bits | default rising edge). */
#define ADC_EXTERNALTRIG_T15_TRGO    ((14UL << ADC_CFGR_EXTSEL_Pos) | ADC_CFGR_EXTEN_0)
#define ADC_EXTERNALTRIG_LPTIM1_CH1  ((18UL << ADC_CFGR_EXTSEL_Pos) | ADC_CFGR_EXTEN_0)
#define SIM_ADC_EXTSEL_TIM15_TRGO    14U
#define SIM_ADC_EXTSEL_LPTIM1_CH1    18U

/* GPDMA1 hardware request numbers (stm32h5xx_hal_dma.h). */
#define GPDMA1_REQUEST_ADC1       0U
#define GPDMA1_REQUEST_ADC2       1U
#define GPDMA1_REQUEST_I2C2_RX    15U
#define GPDMA1_REQUEST_I2C3_RX    18U
#define GPDMA1_REQUEST_USART2_RX  23U
#define GPDMA1_REQUEST_USART2_TX  24U

#define IS_TIM_BREAK_INSTANCE(INSTANCE) \
    (((INSTANCE) == TIM1) || ((INSTANCE) == TIM8) || ((INSTANCE) == TIM15) || \
     ((INSTANCE) == TIM16) || ((INSTANCE) == TIM17))
#define IS_TIM_ENCODER_INTERFACE_INSTANCE(INSTANCE) \
    (((INSTANCE) == TIM1) || ((INSTANCE) == TIM2) || ((INSTANCE) == TIM3) || \
     ((INSTANCE) == TIM4) || ((INSTANCE) == TIM5) || ((INSTANCE) == TIM8))

/* HAL GPIO pull / speed constants (used by pin_function callers). */
#define GPIO_NOPULL   0x0U
#define GPIO_PULLUP   0x1U
#define GPIO_PULLDOWN 0x2U

#endif /* BUCKY_SIM_STM32H5XX_H */
