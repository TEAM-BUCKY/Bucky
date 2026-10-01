#ifndef BUCKY_BOARD_H
#define BUCKY_BOARD_H

// Pin and peripheral assignment for the PCB being built, selected by MCU family.
// Everything that differs between boards lives in these headers; drivers only
// receive the values.

#include "hardware/io/mcu.h"

#if defined(MCU_FAMILY_G4)
#include "board_g474re.h"
#elif defined(MCU_FAMILY_H5)
#include "board_h562rg.h"
#endif

#endif // BUCKY_BOARD_H
