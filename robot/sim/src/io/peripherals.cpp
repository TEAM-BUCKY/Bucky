// Register blocks of every non-GPIO peripheral the firmware addresses, zeroed on reboot.
#include <cstring>

#include "core/world.h"
#include "io/io.h"

extern "C" {
TIM_TypeDef sim_TIM1, sim_TIM2, sim_TIM3, sim_TIM4, sim_TIM5, sim_TIM6, sim_TIM7, sim_TIM8,
    sim_TIM12, sim_TIM13, sim_TIM14, sim_TIM15, sim_TIM16, sim_TIM17;
LPTIM_TypeDef sim_LPTIM1, sim_LPTIM2, sim_LPTIM3, sim_LPTIM4, sim_LPTIM5, sim_LPTIM6;
ADC_TypeDef sim_ADC1, sim_ADC2;
ADC_Common_TypeDef sim_ADC12_COMMON;
I2C_TypeDef sim_I2C1, sim_I2C2, sim_I2C3, sim_I2C4;
USART_TypeDef sim_USART1, sim_USART2, sim_USART3, sim_UART4, sim_UART5;
DMA_Channel_TypeDef sim_GPDMA1_Channel[8];
}

namespace {

template <typename T>
void zero(T& t) { std::memset(static_cast<void*>(&t), 0, sizeof(T)); }

void reset_peripherals() {
    for (TIM_TypeDef* t : {&sim_TIM1, &sim_TIM2, &sim_TIM3, &sim_TIM4, &sim_TIM5, &sim_TIM6,
                           &sim_TIM7, &sim_TIM8, &sim_TIM12, &sim_TIM13, &sim_TIM14, &sim_TIM15,
                           &sim_TIM16, &sim_TIM17}) {
        zero(*t);
        t->ARR = 0xFFFFU;   // reset value
    }
    for (LPTIM_TypeDef* t : {&sim_LPTIM1, &sim_LPTIM2, &sim_LPTIM3, &sim_LPTIM4, &sim_LPTIM5,
                             &sim_LPTIM6}) {
        zero(*t);
        t->ARR = 1U;
    }
    zero(sim_ADC1);
    zero(sim_ADC2);
    zero(sim_ADC12_COMMON);
    for (I2C_TypeDef* i : {&sim_I2C1, &sim_I2C2, &sim_I2C3, &sim_I2C4}) zero(*i);
    for (USART_TypeDef* u : {&sim_USART1, &sim_USART2, &sim_USART3, &sim_UART4, &sim_UART5}) zero(*u);
    for (auto& c : sim_GPDMA1_Channel) zero(c);
}

const bool registered = [] {
    sim::world().reset_hooks.emplace_back(reset_peripherals);
    reset_peripherals();
    return true;
}();

}  // namespace
