#ifndef BUCKY_BITBOARD_H
#define BUCKY_BITBOARD_H
#include <cstdint>

#define setBit(board, position) ((board) |= (1U << (position)))
#define clearBit(board, position) ((board) &= ~(1U << (position)))
#define toggleBit(board, position) ((board) ^= (1U << (position)))
#define getBit(board, position) ((board) & (1U << (position)))
#define popBit(board) ((board) & ((board) - 1))
#define Bitloop(board) for (; (board); (board) = popBit(board))

#endif //BUCKY_BITBOARD_H