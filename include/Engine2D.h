#ifndef ENGINE2D_H
#define ENGINE2D_H

#include "colors.h"
#include "Vector2D.h"

typedef struct
{
    double mass;
    VECTOR_2D pos, vel;
} PHYS_BODY;

typedef struct ENGINE_2D ENGINE_2D;

enum MODES
{
    ELASTIC_COLLISION = 1,
    ENABLE_GRAVITY = 2,
    STARTUP_MOVE = 4,
    BOUNDING_BOX = 8,
    ENABLE_LOGGING = 16,
    PAUSED = 32,
    ENABLE_INPUT = 64,
};

ENGINE_2D *Engine2D_Init(void *renderer, void *shared_state_mutex, void *log_file, int fps, int flags);
void Engine2D_CreateCircleObject(ENGINE_2D *engine, RGB24 color, double radius, PHYS_BODY phys_comp);
void Engine2D_RunSimulation(ENGINE_2D *engine);

void Engine2D_Free(ENGINE_2D *engine);

#endif