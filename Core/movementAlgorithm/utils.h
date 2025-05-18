#pragma once
#include "definitions.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Array utilities
uint8_t findMin(uint8_t *arr, uint8_t size);
void resetArray(uint8_t *arr, uint8_t size, uint8_t value);
FlameColor toggleColor(FlameColor curr);

// Data structure operations
void initQueue(Queue *q);
Node popQueue(Queue *q);
void pushQueue(Queue *q, uint8_t x, uint8_t y);
void initStack(Stack *q);
Node popStack(Stack *q);
void pushStack(Stack *q, uint8_t x, uint8_t y);
void initVisitedMatrix(Node matrix[MAP_SIZE][MAP_SIZE]);

// Map utilities
FlameColor (*generate2DArray(FlameColor *const map))[MAP_SIZE];
void printInstruction(BotInstruction *arr, uint8_t size);
void printMap(FlameColor map[MAP_SIZE][MAP_SIZE]);
void transposeMatrix(FlameColor map[MAP_SIZE][MAP_SIZE]);