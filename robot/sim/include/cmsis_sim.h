/* CMSIS core intrinsics for the host simulator.
 *
 * PRIMASK is simulated: __disable_irq() masks the simulator's interrupt dispatch and
 * re-enabling it runs any interrupt that became pending meanwhile, the way the NVIC would.
 * Barriers are no-ops (the firmware and its "interrupts" share one host thread). */
#ifndef BUCKY_SIM_CMSIS_H
#define BUCKY_SIM_CMSIS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t sim_get_primask(void);
void sim_set_primask(uint32_t primask);

static inline uint32_t __get_PRIMASK(void) { return sim_get_primask(); }
static inline void __set_PRIMASK(uint32_t primask) { sim_set_primask(primask); }
static inline void __disable_irq(void) { sim_set_primask(1U); }
static inline void __enable_irq(void) { sim_set_primask(0U); }

static inline void __DMB(void) {}
static inline void __DSB(void) {}
static inline void __ISB(void) {}
static inline void __NOP(void) {}

#ifdef __cplusplus
}
#endif

#endif /* BUCKY_SIM_CMSIS_H */
