#include "utils.h"
// #include "types.h" - Already included via definitions.h
#include <stdint.h>

uint8_t findMin(uint8_t *arr, uint8_t size) {
  uint8_t min = 255;
  for (uint8_t i = 0; i < size; i++) {
    if (arr[i] < min && arr[i] != 0) // Ignore zero
      min = arr[i];
  }
  return min;
}

void resetArray(uint8_t *arr, uint8_t size, uint8_t value) {
  for (uint8_t i = 0; i < size; i++) {
    arr[i] = value;
  }
}

FlameColor toggleColor(FlameColor curr) { return curr == Blue ? Red : Blue; }

void initVisitedMatrix(Node matrix[MAP_SIZE][MAP_SIZE]) {
  for (int i = 0; i < MAP_SIZE; i++) {
    for (int j = 0; j < MAP_SIZE; j++) {
      // Mark for unvisited as -2
      matrix[i][j].x = -2;
      matrix[i][j].y = -2;
    }
  }
}

void initQueue(Queue *q) {
  q->size = 0;
  q->popIndex = 0;
  q->pushIndex = 0;
}

Node popQueue(Queue *q) {
  if (q->size == 0)
    printf("Error: queue is currently empty and tried to be pop.");
  if (q->popIndex >= 20)
    q->popIndex -= Q_SIZE;
  Node temp;
  temp.x = q->arr[q->popIndex].x;
  temp.y = q->arr[q->popIndex].y;
  q->size--;
  q->popIndex++;
  return temp;
}

void pushQueue(Queue *q, uint8_t x, uint8_t y) {
  if (q->size > Q_SIZE)
    printf("Error: trying to push to FULL queue");
  if (q->pushIndex >= 20)
    q->pushIndex -= Q_SIZE;
  q->arr[q->pushIndex].x = x;
  q->arr[q->pushIndex].y = y;
  q->size++;
  q->pushIndex++;
}

void initStack(Stack *q) {
  q->size = 0;
  q->popIndex = 0;
  q->pushIndex = 0;
}

Node popStack(Stack *q) {
  if (q->size == 0)
    printf("Error: stack is currently empty and tried to be pop.");
  Node temp;
  temp.x = q->arr[q->popIndex].x;
  temp.y = q->arr[q->popIndex].y;
  q->size--;
  q->popIndex--;
  q->pushIndex--;
  return temp;
}

void pushStack(Stack *q, uint8_t x, uint8_t y) {
  if (q->size > Q_SIZE)
    printf("Error: trying to push to FULL stack");
  q->arr[q->pushIndex].x = x;
  q->arr[q->pushIndex].y = y;
  q->size++;
  q->pushIndex++;
  q->popIndex = q->pushIndex - 1; // Pops the element on top
}

// A pointer that return a pointer to an array of 5 elements
// Generate 2d matrix from flattened array passed from WASM
FlameColor (*generate2DArray(FlameColor *const map))[MAP_SIZE] {
  FlameColor(*arr)[MAP_SIZE] = malloc(sizeof(int[MAP_SIZE][MAP_SIZE]));

  int count = 0;
  for (int i = 0; i < MAP_SIZE; i++) {
    for (int j = 0; j < MAP_SIZE; j++) {
      // NOTE: turns out, I aleady flipped my usage from the javascript too,
      // so no need to retranslate the code NOTE: check for later when
      // implementing to the robot, whether the orientation of the map is
      // correct, when inputting, manually, it should be transposed
      arr[i][j] = map[count];
      count++;
    }
  }

  return arr;
}

void printInstruction(BotInstruction *arr, uint8_t size) {
  for (int i = 0; i < size; i++) {
    printf("%d ", arr[i]);
  }
  printf("\n");
}

void printMap(FlameColor map[MAP_SIZE][MAP_SIZE]) {
  FlameColor mapCopy[MAP_SIZE][MAP_SIZE];
  memcpy(mapCopy, map, sizeof(FlameColor) * MAP_SIZE * MAP_SIZE);
  transposeMatrix(mapCopy);
  for (int i = 0; i < MAP_SIZE; i++) {
    for (int j = 0; j < MAP_SIZE; j++) {
      printf("%d ", mapCopy[i][j]);
    }
    printf("\n");
  }
}

void transposeMatrix(FlameColor map[MAP_SIZE][MAP_SIZE]) {
  for (int i = 1; i < MAP_SIZE; i++) {
    for (int j = 0; j < i; j++) {
      FlameColor temp = map[i][j];
      map[i][j] = map[j][i];
      map[j][i] = temp;
    }
  }
}
