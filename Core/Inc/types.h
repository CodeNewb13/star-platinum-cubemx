#pragma once
#include "flameColors.h"
#include "macros.h"
#include <stdint.h>

typedef enum {
  // Move right, forward, left, back
  movr,
  movf,
  movl,
  movb,

  // Rotate to left, right, or back (180 degrees)
  rotl,
  rotr,
  rotb,

  // Take ore in the front grid
  take,

  // Put flame on HIGH lookout on index [0 - 3]
  puth0,
  puth1,
  puth2,
  puth3,

  stop,
} BotInstruction;

typedef enum { Right, Bottom, Left, Top } BotOrientation;

typedef struct {
  int8_t x;
  int8_t y;
} Node;

typedef struct {
  Node arr[Q_SIZE];
  uint8_t size;
  uint8_t popIndex; // It can get to -1
  uint8_t pushIndex;
} Queue, Stack; // Since they use the same term

typedef struct {
  uint8_t x;
  uint8_t y;
  BotOrientation orientation;
  FlameColor storage[4];
} Bot;
