#include "algo.h"
#include "utils.h"
#include <stdint.h>
#include <string.h>
#include <limits.h>

static const FlameColor patterns[PHASES][3] = {
    {Red, Red, Blue},
    {Red, Blue, Red},
    {Red, Blue, Blue},
    {Blue, Blue, Red},
    {Blue, Red, Blue},
    {Blue, Red, Red}
};



//==============================================================================
// MAIN ALGORITHM ENTRY POINT
//==============================================================================

extern void pathFind(BotInstruction *moves, FlameColor *const map) {
    FlameColor (*matrix)[MAP_SIZE] = generate2DArray(map);

    printMap(matrix);

    // Initialize bot struct
    Bot bot;
    bot.x = 2;
    bot.y = 0;
    bot.orientation = Bottom;
    for (int i = 0; i < 3; i++) {
        bot.storage[i] = 0;     // Empty flame storage
    }

    uint8_t insIndex = 0;        // Index pointing to next empty slot on moves array
    uint8_t lookoutMap[5] = {0}; // Array to track which columns have been used as lookouts

    // Find the optimal sequence of moves to collect and place flames
    optimizeTorchPlacementStrategy(matrix, &bot, moves);
}

static int processTorchPlacementPhase(
    const FlameColor pattern[3], uint8_t lookoutFlag,
    FlameColor map[MAP_SIZE][MAP_SIZE], Bot *const bot,
    BotInstruction *moves, uint8_t *insIndex,
    uint8_t *lookoutMap
) {
    Node targetPos;
    
    // Step 1: Collect 3 flames in the specified pattern
    for (int i = 0; i < 3; ++i) {
        FlameColor target = pattern[i];
        // Navigate to the next flame of the target color
        if (!navigateToTarget(target, 0, map, bot, &targetPos, lookoutMap, moves, insIndex)) {
            return 0; // Failed to find path to target flame
        }
        // Collect the flame
        moves[(*insIndex)++] = take;
        map[targetPos.x][targetPos.y] = Empty; // Remove flame from map
    }

    // Step 2: Navigate to lookout position (bottom row)
    FlameColor lastColor = pattern[2];
    if (!navigateToTarget(lastColor, 1, map, bot, &targetPos, lookoutMap, moves, insIndex)) {
        return 0; // Failed to find path to lookout position
    }
    
    // Mark this lookout column as used
    lookoutMap[targetPos.x] = 1;
    
    // Move to the lookout position
    moves[(*insIndex)++] = movf;
    
    // Orient the bot to face downward (for consistent torch placement)
    orientateBot(bot, Bottom, moves, insIndex);
    (*insIndex)++; // Skip redundant index adjustment from orientateBot

    // Step 3: Place the 3 torches horizontally
    static const BotInstruction putInstr[3] = {puth0, puth1, puth2};
    for (int j = 0; j < 3; ++j) {
        moves[(*insIndex)++] = putInstr[j];
    }
    
    return 1; // Successfully completed the phase
}

/**
 * Navigates to a target flame or lookout position
 * 
 * Finds shortest path using BFS and translates it to bot instructions
 */

uint8_t navigateToTarget(
    FlameColor target, uint8_t lookoutFlag,
    FlameColor map[MAP_SIZE][MAP_SIZE], Bot *const bot,
    Node *targetPos, uint8_t *lookoutMap,
    BotInstruction *moves, uint8_t *insIndex
) {
    uint8_t found = 0;
    Node visited[MAP_SIZE][MAP_SIZE]; // Stores previous nodes for path reconstruction
    Stack prevNodes;                  // Stack for path storage
    initStack(&prevNodes);
    
    // Find shortest path to target using BFS
    found = BFS(map, bot, target, visited, targetPos, lookoutFlag, lookoutMap);
    
    if (found) {
        // Reconstruct the path from current position to target
        reconstructPath(visited, &prevNodes, *targetPos);
        
        // Generate movement instructions from the path
        generateInstruction(&prevNodes, bot, moves, insIndex);
    }
    
    return found;
}

//==============================================================================
// OPTIMIZATION AND STRATEGY
//==============================================================================

/**
 * Finds the optimal torch collection strategy by testing all pattern combinations
 * 
 * This function:
 * 1. Tests all combinations of flame collection patterns
 * 2. Selects the pattern that requires the fewest bot instructions
 */
void optimizeTorchPlacementStrategy(
    FlameColor map[MAP_SIZE][MAP_SIZE], Bot *const bot,
    BotInstruction *moves
) {
    FlameColor mapCopy[MAP_SIZE][MAP_SIZE];
    uint8_t lookoutMap[MAP_SIZE] = {0};                // Tracks used lookout columns
    BotInstruction movesBuffer[NUM_STRATEGIES][64];    // Stores instructions for each strategy
    uint8_t insCounts[NUM_STRATEGIES] = {0};           // Number of instructions for each strategy
    
    uint16_t bestCount = UINT16_MAX;                   // Best (lowest) instruction count found
    int stratIdx = 0;                                  // Current strategy index
    
    // Test all combinations of flame collection patterns
    for (int p1 = 0; p1 < PHASES; ++p1) {
        for (int p2 = 0; p2 < PHASES; ++p2) {
            // Reset bot state to starting position
            bot->x = 2;
            bot->y = 0;
            bot->orientation = Bottom;
            
            // Reset map and lookout state
            memcpy(mapCopy, map, sizeof mapCopy);
            resetArray(lookoutMap, MAP_SIZE, 0);
            
            uint8_t idx = 0;  // Instruction counter
            
            // Try first phase with pattern p1
            if (!processTorchPlacementPhase(patterns[p1], 1, mapCopy, bot, movesBuffer[stratIdx], &idx, lookoutMap) ||
                idx >= bestCount) {
                stratIdx++;
                continue;  // Skip if first phase failed or already worse than best
            }
            
            // Try second phase with pattern p2
            if (!processTorchPlacementPhase(patterns[p2], 1, mapCopy, bot, movesBuffer[stratIdx], &idx, lookoutMap) ||
                idx >= bestCount) {
                stratIdx++;
                continue;  // Skip if second phase failed or already worse than best
            }
            
            // Record results of this strategy
            insCounts[stratIdx] = idx;
            if (idx < bestCount) {
                bestCount = idx;  // Update best count if this is better
            }
            stratIdx++;
        }
    }
    
    // Find and use the best strategy
    for (int i = 0; i < stratIdx; ++i) {
        if (insCounts[i] == bestCount) {
            // Copy the best strategy to the output moves array
            memcpy(moves, movesBuffer[i], bestCount * sizeof *moves);
            moves[bestCount] = stop;  // Add stop instruction at the end
            return;
        }
    }
    
    // Fallback: if no valid strategy was found
    moves[0] = stop;
}

//==============================================================================
// PATHFINDING ALGORITHMS
//==============================================================================

/**
 * Breadth-First Search to find shortest path to target
 * 
 * Searches for:
 * - A flame of the specified color if lookoutFlag=0
 * - A suitable lookout position on bottom row if lookoutFlag=1
 */

int BFS(
    const FlameColor map[MAP_SIZE][MAP_SIZE], Bot *const bot,
    FlameColor target, Node visited[MAP_SIZE][MAP_SIZE], Node *targetPos,
    uint8_t lookoutFlag, uint8_t *lookoutMap
) {
    Queue q;
    Queue *qp = &q;
    initQueue(qp);
    
    // Initialize visited matrix
    initVisitedMatrix(visited);
    visited[bot->x][bot->y].x = -1;
    visited[bot->x][bot->y].y = -1;  // Mark start position
    
    // Start BFS from current bot position
    pushQueue(qp, bot->x, bot->y);
    
    int endReached = 0;
    while (q.size > 0) {
        Node curr = popQueue(qp);
        
        // Check all four cardinal directions (no diagonals)
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx != 0 && dy != 0)
                    continue;  // Skip diagonals
                if (dx == 0 && dy == 0)
                    continue;  // Skip center
                
                Node neighbor;
                neighbor.x = curr.x + dx;
                neighbor.y = curr.y + dy;
                
                // Skip if outside map boundaries
                if (neighbor.x < 0 || neighbor.y < 0 || neighbor.x >= MAP_SIZE || neighbor.y >= MAP_SIZE)
                    continue;
                
                // Skip obstacles:
                // - In lookout mode: all flames are obstacles
                // - In collect mode: flames of different color are obstacles
                if ((lookoutFlag && map[neighbor.x][neighbor.y] != 0) ||
                    (map[neighbor.x][neighbor.y] != 0 && map[neighbor.x][neighbor.y] != target))
                    continue;
                
                // Skip already visited positions
                if (visited[neighbor.x][neighbor.y].x > -2)
                    continue;
                
                // Record how we reached this position
                visited[neighbor.x][neighbor.y].x = curr.x;
                visited[neighbor.x][neighbor.y].y = curr.y;
                
                // Check if this is a target position:
                // - In lookout mode: position on bottom row that hasn't been used
                // - In collect mode: flame of target color
                if ((lookoutFlag && neighbor.y == 4 && lookoutMap[neighbor.x] == 0) ||
                    map[neighbor.x][neighbor.y] == target) {
                    endReached = 1;
                    targetPos->x = neighbor.x;
                    targetPos->y = neighbor.y;
                    break;
                }
                
                // Continue BFS from this position
                pushQueue(qp, neighbor.x, neighbor.y);
            }
            if (endReached)
                break;
        }
        if (endReached)
            break;
    }
    
    return endReached;  // Return whether a path was found
}

/**
 * Reconstructs path from BFS results
 * 
 * Traces backwards from target position to start position
 */
void reconstructPath(
    const Node visited[MAP_SIZE][MAP_SIZE], Stack *prevNodes,
    const Node targetPos
) {
    Node temp;
    temp.x = targetPos.x;
    temp.y = targetPos.y;
    
    // Build path by following parent pointers backwards
    while (temp.x != -1 && temp.y != -1) {
        pushStack(prevNodes, temp.x, temp.y);
        temp = visited[temp.x][temp.y];
    }
    
    popStack(prevNodes);  // Remove the target position (we don't need to move there)
}

//==============================================================================
// INSTRUCTION GENERATION
//==============================================================================

/**
 * Converts path nodes to bot movement instructions
 * 
 * Generates appropriate movement commands based on the bot's orientation
 * and the direction it needs to move
 */
void generateInstruction(
    Stack *prevNodes, Bot *bot, BotInstruction *moves,
    uint8_t *insIndex
) {
    while (prevNodes->size > 0) {
        // Get next position in path
        Node next = popStack(prevNodes);
        
        // Calculate direction vector
        Node dir;
        dir.x = next.x - bot->x;
        dir.y = next.y - bot->y;
        
        // For the final position, orient the bot to face the target
        if (prevNodes->size == 0) {
            // Orient bot based on the direction vector
            if (dir.x == 1 && dir.y == 0) {
                orientateBot(bot, Right, moves, insIndex);
            }
            else if (dir.x == 0 && dir.y == 1) {
                orientateBot(bot, Bottom, moves, insIndex);
            }
            else if (dir.x == -1 && dir.y == 0) {
                orientateBot(bot, Left, moves, insIndex);
            }
            else if (dir.x == 0 && dir.y == -1) {
                orientateBot(bot, Top, moves, insIndex);
            }
        } 
        else { // Generate movement instruction
            // Normalize direction based on bot's current orientation
            Node normalDir = dir;
            normalizeDirection(bot->orientation, &normalDir);
            
            // Select appropriate movement instruction
            if (normalDir.x == -1 && normalDir.y == 0) {
                moves[*insIndex] = movr;  // Move right
            }
            else if (normalDir.x == 0 && normalDir.y == 1) {
                moves[*insIndex] = movf;  // Move forward
            }
            else if (normalDir.x == 1 && normalDir.y == 0) {
                moves[*insIndex] = movl;  // Move left
            }
            else if (normalDir.x == 0 && normalDir.y == -1) {
                moves[*insIndex] = movb;  // Move backward
            }
            // Note: diagonal movements not implemented
        }
        
        // Update bot's position
        bot->x += dir.x;
        bot->y += dir.y;
        
        // Move to next instruction slot
        (*insIndex)++;
    }
}

//==============================================================================
// BOT MOVEMENT UTILITIES
//==============================================================================

/**
 * Transforms a direction vector based on bot's orientation
 * 
 * Converts a global direction (e.g., "south") to a bot-relative
 * direction (e.g., "forward" if bot is facing south)
 */
void normalizeDirection(const BotOrientation orientation, Node *dir) {
    int8_t temp = dir->x;
    switch (orientation) {
        case Bottom:  // Bot facing down (no transformation needed)
            break;
        case Right:   // Bot facing right (rotate vectors 90° clockwise)
            dir->x = -dir->y;
            dir->y = temp;
            break;
        case Top:     // Bot facing up (rotate vectors 180°)
            dir->x = -dir->x;
            dir->y = -dir->y;
            break;
        case Left:    // Bot facing left (rotate vectors 90° counterclockwise)
            dir->x = dir->y;
            dir->y = -temp;
            break;
    }
}

/**
 * Changes bot orientation with appropriate rotation instruction
 * 
 * Determines most efficient rotation to reach target orientation
 */
void orientateBot(
    Bot *bot, BotOrientation orientation, BotInstruction *moves,
    uint8_t *insIndex
) {
    // Skip if already correctly oriented
    if (bot->orientation == orientation) {
        (*insIndex)--;  // Adjust index to avoid empty slot
        return;
    }
    
    // Determine rotation based on current and target orientations
    if (bot->orientation == Right) {
        if (orientation == Left)
            moves[*insIndex] = rotb;      // 180° turn
        else if (orientation == Top)
            moves[*insIndex] = rotl;      // 90° left
        else if (orientation == Bottom)
            moves[*insIndex] = rotr;      // 90° right
    } 
    else if (bot->orientation == Bottom) {
        if (orientation == Top)
            moves[*insIndex] = rotb;
        else if (orientation == Right)
            moves[*insIndex] = rotl;
        else if (orientation == Left)
            moves[*insIndex] = rotr;
    } 
    else if (bot->orientation == Left) {
        if (orientation == Right)
            moves[*insIndex] = rotb;
        else if (orientation == Bottom)
            moves[*insIndex] = rotl;
        else if (orientation == Top)
            moves[*insIndex] = rotr;
    } 
    else if (bot->orientation == Top) {
        if (orientation == Bottom)
            moves[*insIndex] = rotb;
        else if (orientation == Left)
            moves[*insIndex] = rotl;
        else if (orientation == Right)
            moves[*insIndex] = rotr;
    }
    
    // Update bot's orientation
    bot->orientation = orientation;
}