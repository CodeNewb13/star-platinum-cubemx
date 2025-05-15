#pragma once
#include "definitions.h"

// Core pathfinding function (externally visible)
extern void pathFind(BotInstruction *moves, FlameColor *const map);

// Helper: perform one 4-torch phase + lookout + place
static int findPhasePath(
    const FlameColor pattern[4], uint8_t lookoutFlag,
    FlameColor map[MAP_SIZE][MAP_SIZE], Bot *const bot,
    BotInstruction *moves, uint8_t *insIndex,
    uint8_t *lookoutMap
);

// Major algorithm components
void findBestAlternatePath(FlameColor map[MAP_SIZE][MAP_SIZE], Bot *const bot, BotInstruction *moves);

// Path utilities
uint8_t findPathToTarget(FlameColor target, uint8_t lookoutFlag,
                       FlameColor map[MAP_SIZE][MAP_SIZE], Bot *const bot,
                       Node *targetPos, uint8_t *lookoutMap,
                       BotInstruction *moves, uint8_t *insIndex);

int BFS(const FlameColor map[MAP_SIZE][MAP_SIZE], Bot *const bot,
        FlameColor target, Node visited[MAP_SIZE][MAP_SIZE], Node *targetPos,
        uint8_t lookoutFlag, uint8_t *lookoutMap);

void traceBackPath(const Node visited[MAP_SIZE][MAP_SIZE], Stack *prevNodes,
                   const Node targetPos);

void generateInstruction(Stack *prevNodes, Bot *bot, BotInstruction *moves,
                       uint8_t *insIndex);

// Bot movement and orientation
void normalizeDirection(const BotOrientation orientation, Node *dir);
void orientateBot(Bot *bot, BotOrientation orientation, BotInstruction *moves,
                uint8_t *insIndex);