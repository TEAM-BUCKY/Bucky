#ifndef BUCKY_BITBOARD_H
#define BUCKY_BITBOARD_H
#include <cstdint>

#define setBit(board, position) ((board) |= (1U << (position)))
#define clearBit(board, position) ((board) &= ~(1U << (position)))
#define toggleBit(board, position) ((board) ^= (1U << (position)))
#define getBit(board, position) ((board) & (1U << (position)))
#define popBit(board) ((board) & ((board) - 1))
#define Bitloop(board) for (; (board); (board) = popBit(board))
#define GetLSB(board) (__builtin_ctz(board))

#define writeField(reg, mask, shift, value) \
    ((reg) = ((reg) & ~((uint32_t)(mask) << (shift))) | ((uint32_t)(value) << (shift)))

#define clearField(reg, mask, shift) \
    ((reg) &= ~((uint32_t)(mask) << (shift)))

#define setMask(reg, mask) ((reg) |= (mask))
#define clearMask(reg, mask) ((reg) &= ~(mask))
#define toggleMask(reg, mask) ((reg) ^= (mask))
#define testMask(reg, mask) ((reg) & (mask))

#define combineBytes(high, low) \
    (((high) << 8) | (low))

#endif //BUCKY_BITBOARD_H