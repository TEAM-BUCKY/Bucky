#ifndef BUCKY_DEBUG_H
#define BUCKY_DEBUG_H

#define DEBUG_LOGGING

#ifdef DEBUG_LOGGING
  #define DBG_PRINT(...)    Serial.print(__VA_ARGS__)
  #define DBG_PRINTLN(...)  Serial.println(__VA_ARGS__)
#else
  #define DBG_PRINT(...)    ((void)0)
  #define DBG_PRINTLN(...)  ((void)0)
#endif

#endif // BUCKY_DEBUG_H
