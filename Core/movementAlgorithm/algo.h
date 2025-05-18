#pragma once
#include "definitions.h"

/**
 * Fire Collector Simulation - Path Finding Algorithm
 * 
 * This header defines the functions for navigating a bot through a grid map
 * to collect flames of specific colors and place them in optimal patterns.
 */

//==============================================================================
// PRIMARY INTERFACE
//==============================================================================

/**
 * Main entry point for pathfinding algorithm
 * Generates a sequence of moves for the bot to collect and place flames
 * 
 * @param moves Output array to store the generated bot instructions
 * @param map Input grid map containing flame locations and colors
 */
extern void pathFind(BotInstruction *moves, FlameColor *const map);

//==============================================================================
// PATH PLANNING AND OPTIMIZATION
//==============================================================================

/**
 * Finds the optimal torch placement strategy by evaluating different collection patterns
 * 
 * @param map 2D grid of flame colors
 * @param bot Bot's current state (position, orientation)
 * @param moves Output array to store the generated bot instructions
 */
void optimizeTorchPlacementStrategy(FlameColor map[MAP_SIZE][MAP_SIZE], Bot *const bot, BotInstruction *moves);

/**
 * Executes one complete cycle of collecting 4 torches and placing them
 * 
 * @param pattern Array of 4 flame colors to collect in sequence
 * @param lookoutFlag Whether to search for a lookout position (1) or not (0)
 * @param map 2D grid of flame colors
 * @param bot Bot's current state (position, orientation)
 * @param moves Output array to store the generated bot instructions
 * @param insIndex Pointer to current index in the moves array
 * @param lookoutMap Array tracking which columns have been used as lookouts
 * @return 1 if successful, 0 if impossible to complete
 */
static int processTorchPlacementPhase(
    const FlameColor pattern[3], uint8_t lookoutFlag,
    FlameColor map[MAP_SIZE][MAP_SIZE], Bot *const bot,
    BotInstruction *moves, uint8_t *insIndex,
    uint8_t *lookoutMap
);

/**
 * Navigate the bot to collect a specific flame color or reach a lookout position
 * 
 * @param target Color to search for, or any color if in lookout mode
 * @param lookoutFlag Whether to search for a lookout position (1) or flame (0)
 * @param map 2D grid of flame colors
 * @param bot Bot's current state (position, orientation)
 * @param targetPos Output parameter - coordinates of the found target
 * @param lookoutMap Array tracking which columns have been used as lookouts
 * @param moves Output array to store the generated bot instructions
 * @param insIndex Pointer to current index in the moves array
 * @return 1 if path found, 0 if impossible to reach target
 */
uint8_t navigateToTarget(
    FlameColor target, uint8_t lookoutFlag,
    FlameColor map[MAP_SIZE][MAP_SIZE], Bot *const bot,
    Node *targetPos, uint8_t *lookoutMap,
    BotInstruction *moves, uint8_t *insIndex
);

//==============================================================================
// PATHFINDING ALGORITHMS
//==============================================================================

/**
 * Breadth-First Search algorithm to find path to target
 * 
 * @param map 2D grid of flame colors
 * @param bot Bot's current state (position, orientation)
 * @param target Target flame color to search for
 * @param visited 2D array to track visited positions and path reconstruction
 * @param targetPos Output parameter - coordinates of the found target
 * @param lookoutFlag Whether to search for a lookout position (1) or flame (0)
 * @param lookoutMap Array tracking which columns have been used as lookouts
 * @return 1 if path found, 0 if impossible to reach target
 */
int BFS(const FlameColor map[MAP_SIZE][MAP_SIZE], Bot *const bot,
        FlameColor target, Node visited[MAP_SIZE][MAP_SIZE], Node *targetPos,
        uint8_t lookoutFlag, uint8_t *lookoutMap);

/**
 * Reconstructs shortest path found by BFS
 * 
 * @param visited 2D array containing parent nodes from BFS
 * @param prevNodes Stack to store the path in reverse order
 * @param targetPos Coordinates of target position
 */
void reconstructPath(const Node visited[MAP_SIZE][MAP_SIZE], Stack *prevNodes,
                   const Node targetPos);

/**
 * Converts path nodes into bot movement instructions
 * 
 * @param prevNodes Stack containing the path in reverse order
 * @param bot Bot's current state (position, orientation)
 * @param moves Output array to store the generated bot instructions
 * @param insIndex Pointer to current index in the moves array
 */
void generateInstruction(Stack *prevNodes, Bot *bot, BotInstruction *moves,
                       uint8_t *insIndex);

//==============================================================================
// BOT MOVEMENT UTILITIES
//==============================================================================

/**
 * Transforms direction vector based on bot's orientation
 * 
 * @param orientation Current orientation of the bot
 * @param dir Direction vector to be normalized
 */
void normalizeDirection(const BotOrientation orientation, Node *dir);

/**
 * Rotates the bot to face a specified direction
 * 
 * @param bot Bot's current state (position, orientation)
 * @param orientation Target orientation 
 * @param moves Output array to store the generated bot instructions
 * @param insIndex Pointer to current index in the moves array
 */
void orientateBot(Bot *bot, BotOrientation orientation, BotInstruction *moves,
                uint8_t *insIndex);