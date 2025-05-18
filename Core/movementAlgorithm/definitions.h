#pragma once
#include <stdint.h>

// Constants
#define Q_SIZE 20
#define MAP_SIZE 5
#define PHASES 6
#define NUM_STRATEGIES (PHASES * PHASES)

// Basic enums
typedef enum { Empty, Blue, Red } FlameColor;

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

  // Special state for calibration
  calibrate,
} BotInstruction;

typedef enum { Right, Bottom, Left, Top } BotOrientation;

// Core data structures
typedef struct {
  int8_t x;
  int8_t y;
} Node;

typedef struct {
  Node arr[Q_SIZE];
  uint8_t size;
  uint8_t popIndex;
  uint8_t pushIndex;
} Queue, Stack;

typedef struct {
  uint8_t x;
  uint8_t y;
  BotOrientation orientation;
  FlameColor storage[4];
} Bot;
