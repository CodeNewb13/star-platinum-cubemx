#pragma once
#include "flameColors.h"
#include "macros.h"
#include "types.h"
#include "utils.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern void pathFind(BotInstruction *moves, FlameColor *const map);
uint8_t findPathToTarget(FlameColor target, uint8_t lookoutFlag,
                         FlameColor map[MAP_SIZE][MAP_SIZE], Bot *const bot,
                         Node *targetPos, uint8_t *lookoutMap,
                         BotInstruction *moves, uint8_t *insIndex);
int findAlternateColor(FlameColor target,
                                FlameColor map[MAP_SIZE][MAP_SIZE],
                                Bot *const bot,
                                BotInstruction *moves,
                                uint8_t *insIndex,
                                uint8_t *lookoutMap,
                                int mode);

void findBestAlternatePath(FlameColor map[MAP_SIZE][MAP_SIZE], Bot *const bot,
                           BotInstruction *moves);
/**
 * Performs a Breadth-First Search (BFS) on the given map to find a path to
 * the target.
 *
 * @param map         The 2D array representing the map with FlameColor
 * values.
 * @param bot         A pointer to the Bot object performing the search.
 * @param target      The target FlameColor to search for.
 * @param visited     A 2D array to track visited nodes during the search.
 * @param targetPos   A pointer to store the position of the target if found.
 * @param lookoutFlag A flag indicating whether or not to search for lookout.
 * @param lookoutMap  A pointer to a map for storing lookout availability.
 * @return            An integer indicating the success or failure of the
 * search.
 */
// TODO: technical debt here, const in this parameter doesn't really mean that
// the map can't be changed I believe, change later
int BFS(const FlameColor map[MAP_SIZE][MAP_SIZE], Bot *const bot,
        FlameColor target, Node visited[MAP_SIZE][MAP_SIZE], Node *targetPos,
        uint8_t lookoutFlag, uint8_t *lookoutMap);
void initVisitedMatrix(Node matrix[5][5]);
void traceBackPath(const Node visited[MAP_SIZE][MAP_SIZE], Stack *prevNodes,
                   const Node targetPos);
void generateInstruction(Stack *prevNodes, Bot *bot, BotInstruction *moves,
                         uint8_t *insIndex);
// Normalize the direction the bot should take, based on the current
// orientation
void normalizeDirection(const BotOrientation orientation, Node *dir);
void orientateBot(Bot *bot, BotOrientation orientation, BotInstruction *moves,
                  uint8_t *insIndex);
