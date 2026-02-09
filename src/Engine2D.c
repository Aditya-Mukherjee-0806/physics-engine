#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include "Engine2D.h"

#define WINDOW_WIDTH 1280
#define WINDOW_HEIGHT 720
// #define FRAMES_PER_SEC 30
#define DEFAULT_ARR_CAPACITY 512
#define PIXELS_PER_METER 1024
#define MIN_RADIUS 8
#define MAX_RADIUS 16
#define DENSITY 768
#define LOG_FILE "log.txt"
#define LOG_INTERVAL_SECS 1
#define INPUT_BUFFER_SIZE 256
#define ELASTIC 1
#define INELASTIC 0
#define DELIM " \t\r\n"
#define DEFAULT_SPEED 196
#define BUFFER_ZONE 128
#define STARTUP_FRAMES 5

typedef struct
{
    SDL_bool alive;
    Uint16 id;
    RGB24 color;
    double radius;
    PHYS_BODY phys_comp;
} CIRCLE_OBJ;

typedef struct
{
    CIRCLE_OBJ *data;
    int size, cap;
} OBJECT_ARRAY;

struct ENGINE_2D
{
    SDL_Renderer *renderer;
    SDL_mutex *shared_state_mutex;
    SDL_Thread *input_thread;
    FILE *log_file;
    OBJECT_ARRAY *objects;
    double dt;
    int flags;
};

const double π = 3.141592653589793;
const double G = 6.6743E-11 * PIXELS_PER_METER * PIXELS_PER_METER * PIXELS_PER_METER;
// const double dt = 1.0 / FRAMES_PER_SEC;
SDL_bool input_thread_exists = SDL_FALSE;

SDL_bool isPointInsideCircle(VECTOR_2D point, CIRCLE_OBJ circle_obj);
void logArrInfo();
void logInfoOf(FILE *log_file, CIRCLE_OBJ *circle_obj);
void Engine2D_RunSimulation();
void sanitiseObjectArray();
void simulateForces();
void simulateGravitationalForce();
void handleCollision(CIRCLE_OBJ *c1, CIRCLE_OBJ *c2, SDL_bool is_collision_elastic);
void updatePositionsAndCheckBounds();
void RenderFillCircle(SDL_Renderer *renderer, CIRCLE_OBJ *circle_obj);
int processUserInput(void *data);
void handleCreateCommand(ENGINE_2D *engine, char *input);
void handleClearCommand(ENGINE_2D *engine, char *input);
void handleSetCommand(ENGINE_2D *engine, char *input);
void handlePauseCommand(ENGINE_2D *engine, char *input);
void handleResumeCommand(ENGINE_2D *engine, char *input);
CIRCLE_OBJ *findCircleById(OBJECT_ARRAY *objects, int id);
SDL_bool tryParseIntOptionArg(char *command_name, char *input_flag, char *short_option, char *long_option, int *p_arg_value);
SDL_bool tryParseFloatOptionArg(char *command_name, char *input_flag, char *short_option, char *long_option, double *p_arg_value);
SDL_bool tryParseCharOptionArg(char *command_name, char *input_flag, char *short_option, char *long_option, char *p_arg_value);
SDL_bool tryParseStrOptionArg(char *command_name, char *input_flag, char *short_option, char *long_option, char *buffer, int buf_size);

OBJECT_ARRAY *Objects_Init()
{
    OBJECT_ARRAY *objects = (OBJECT_ARRAY *)malloc(sizeof(OBJECT_ARRAY));
    objects->cap = DEFAULT_ARR_CAPACITY;
    objects->size = 0;
    objects->data = (CIRCLE_OBJ *)malloc(sizeof(CIRCLE_OBJ) * objects->cap);
    return objects;
}

void *Objects_Free(OBJECT_ARRAY *objects)
{
    free(objects->data);
    objects->data = NULL;
    objects->cap = objects->size = 0;
    free(objects);
}

ENGINE_2D *Engine2D_Init(void *renderer, void *shared_state_mutex, void *log_file, int fps, int flags)
{
    ENGINE_2D *engine = (ENGINE_2D *)malloc(sizeof(ENGINE_2D));
    engine->renderer = renderer;
    engine->shared_state_mutex = shared_state_mutex;
    engine->log_file = log_file;
    engine->dt = 1.0 / fps;
    engine->flags = flags;
    engine->objects = Objects_Init();
    if ((engine->flags & ENABLE_INPUT) && !input_thread_exists)
    {
        engine->input_thread = SDL_CreateThread(processUserInput, "input thread", engine);
        input_thread_exists = SDL_TRUE;
    }
}

void Engine2D_Free(ENGINE_2D *engine)
{
    engine->renderer = NULL;
    engine->shared_state_mutex = NULL;
    engine->log_file = NULL;
    engine->dt = 0;
    engine->flags = 0;
    Objects_Free(engine->objects);
    engine->objects = NULL;
    free(engine);
}

SDL_bool isPointInsideCircle(VECTOR_2D point, CIRCLE_OBJ circle_obj)
{
    VECTOR_2D dist = Vector2D_Difference(point, circle_obj.phys_comp.pos);
    return Vector2D_Magnitude(dist) <= circle_obj.radius;
}

void logArrInfo(ENGINE_2D *engine)
{
    static int log_count = 1;
    fprintf(engine->log_file, "ENTRY: #%d\n", log_count);
    SDL_LockMutex(engine->shared_state_mutex);
    for (int i = 0; i < engine->objects->size; i++)
    {
        fprintf(engine->log_file, "Circle %d:\n", engine->objects->data[i].id);
        logInfoOf(engine->log_file, engine->objects->data + i);
    }
    SDL_UnlockMutex(engine->shared_state_mutex);
    log_count++;
}

void logInfoOf(FILE *log_file, CIRCLE_OBJ *circle_obj)
{
    if (!circle_obj->alive)
    {
        fprintf(log_file, "is Dead.\n");
        return;
    }
    fprintf(log_file, "Radius = %.2lf\n", circle_obj->radius);
    fprintf(log_file, "Mass = %.2lf\n", circle_obj->phys_comp.mass);
    fprintf(log_file, "Position = (%.2lf, %.2lf)\n", circle_obj->phys_comp.pos.x, circle_obj->phys_comp.pos.y);
    fprintf(log_file, "Velocity = (%.2lf, %.2lf)\n", circle_obj->phys_comp.vel.x, circle_obj->phys_comp.vel.y);
}

void Engine2D_CreateCircleObject(ENGINE_2D *engine, RGB24 color, double radius, PHYS_BODY phys_comp)
{
    static int id = 1;

    CIRCLE_OBJ circle_obj = {
        .alive = SDL_TRUE,
        .id = id++,
        .color = color,
        .radius = radius,
        .phys_comp = phys_comp,
    };

    SDL_LockMutex(engine->shared_state_mutex);

    if (engine->objects->size >= engine->objects->cap)
    {
        CIRCLE_OBJ *temp = (CIRCLE_OBJ *)realloc(engine->objects->data, engine->objects->cap * 4 * sizeof(CIRCLE_OBJ));
        if (temp)
        {
            engine->objects->cap *= 4;
            engine->objects->data = temp;
        }
        else
            fprintf(stderr, "REALLOCATION FAILED in %s\n", __func__);
    }
    engine->objects->data[engine->objects->size++] = circle_obj;

    SDL_UnlockMutex(engine->shared_state_mutex);
}

void Engine2D_RunSimulation(ENGINE_2D *engine)
{
    SDL_LockMutex(engine->shared_state_mutex);

    sanitiseObjectArray(engine->objects);
    simulateForces(engine);
    updatePositionsAndCheckBounds(engine);
    for (int i = 0; i < engine->objects->size; i++)
        RenderFillCircle(engine->renderer, engine->objects->data + i);

    SDL_UnlockMutex(engine->shared_state_mutex);
}

void sanitiseObjectArray(OBJECT_ARRAY *objects)
{
    int i = 0, j = objects->size;
    while (i < j)
    {
        if (objects->data[i].alive)
            i++;
        else
            objects->data[i] = objects->data[--j];
    }
    // all elements before i are alive, and all elements from j onwards are dead
    // when i == j, the loop ends and i points to a dead element
    // therefore total num of alive elements is i
    objects->size = i;

    if (objects->size < objects->cap / 8)
    {
        CIRCLE_OBJ *temp = (CIRCLE_OBJ *)realloc(objects->data, objects->cap / 2 * sizeof(CIRCLE_OBJ));
        if (temp)
        {
            objects->cap /= 2;
            objects->data = temp;
        }
        else
            fprintf(stderr, "REALLOCATION FAILED in %s\n", __func__);
    }
}

void simulateForces(ENGINE_2D *engine)
{
    simulateGravitationalForce(engine);
}

void simulateGravitationalForce(ENGINE_2D *engine)
{
    for (int i = 0; i < engine->objects->size - 1; i++)
    {
        if (!engine->objects->data[i].alive)
            continue;
        for (int j = i + 1; j < engine->objects->size; j++)
        {
            if (!engine->objects->data[j].alive)
                continue;
            double m1 = engine->objects->data[i].phys_comp.mass;
            double m2 = engine->objects->data[j].phys_comp.mass;
            VECTOR_2D pos1 = engine->objects->data[i].phys_comp.pos;
            VECTOR_2D pos2 = engine->objects->data[j].phys_comp.pos;
            VECTOR_2D displacement = Vector2D_Difference(pos2, pos1);
            double dist = Vector2D_Magnitude(displacement);
            if (dist < engine->objects->data[i].radius + engine->objects->data[j].radius)
            {
                handleCollision(engine->objects->data + i, engine->objects->data + j, engine->flags & ELASTIC_COLLISION);
                if (engine->flags & ELASTIC_COLLISION)
                    // stop calculating with dead circles[j] in case of merge
                    continue;
            }
            if (engine->flags & ENABLE_GRAVITY)
            {
                double force_magnitude = G * m1 * m2 / SDL_pow(dist, 2);
                VECTOR_2D force = Vector2D_ScalarProduct(
                    Vector2D_Normalised(displacement),
                    force_magnitude);

                engine->objects->data[i].phys_comp.vel = Vector2D_Sum(
                    engine->objects->data[i].phys_comp.vel,
                    Vector2D_ScalarProduct(force, engine->dt / m1));

                engine->objects->data[j].phys_comp.vel = Vector2D_Difference(
                    engine->objects->data[j].phys_comp.vel,
                    Vector2D_ScalarProduct(force, engine->dt / m2));
            }
        }
    }
}

void handleCollision(CIRCLE_OBJ *c1, CIRCLE_OBJ *c2, SDL_bool is_collision_elastic)
{
    double m1 = c1->phys_comp.mass;
    double m2 = c2->phys_comp.mass;
    VECTOR_2D u1 = c1->phys_comp.vel;
    VECTOR_2D u2 = c2->phys_comp.vel;
    VECTOR_2D pos1 = c1->phys_comp.pos;
    VECTOR_2D pos2 = c2->phys_comp.pos;
    if (is_collision_elastic)
    {
        // bounce c1 and c2 off each other
        c1->phys_comp.vel = Vector2D_ScalarProduct(
            Vector2D_Sum(
                Vector2D_ScalarProduct(u1, m1 - m2),
                Vector2D_ScalarProduct(u2, m2 * 2)),
            1 / (m1 + m2));

        c2->phys_comp.vel = Vector2D_ScalarProduct(
            Vector2D_Sum(
                Vector2D_ScalarProduct(u2, m2 - m1),
                Vector2D_ScalarProduct(u1, m1 * 2)),
            1 / (m1 + m2));

        // Push c1 outside of c2
        VECTOR_2D displacement = Vector2D_Difference(pos1, pos2);
        double push_back_magnitude = c1->radius + c2->radius - Vector2D_Magnitude(displacement);
        VECTOR_2D push_back_vec = Vector2D_ScalarProduct(Vector2D_Normalised(displacement), push_back_magnitude);
        c1->phys_comp.pos = Vector2D_Sum(c1->phys_comp.pos, push_back_vec);
    }
    else
    {
        // Merge c2 into c1
        c1->color = mixTwoColors(c1->color, c2->color);
        // Conservation of Linear Momentum
        c1->phys_comp.vel = Vector2D_ScalarProduct(
            Vector2D_Sum(
                Vector2D_ScalarProduct(u1, m1),
                Vector2D_ScalarProduct(u2, m2)),
            1 / (m1 + m2));
        // Conservation of Centre of Mass
        c1->phys_comp.pos = Vector2D_ScalarProduct(
            Vector2D_Sum(
                Vector2D_ScalarProduct(pos1, m1),
                Vector2D_ScalarProduct(pos2, m2)),
            1 / (m1 + m2));
        // mass of new body is the combined mass of both bodies, and radius is recalculated according to new mass
        c1->phys_comp.mass += c2->phys_comp.mass;
        c1->radius = SDL_sqrt(c1->phys_comp.mass / (π * DENSITY));
        // destroy c2
        c2->alive = 0;
    }
}

void updatePositionsAndCheckBounds(ENGINE_2D *engine)
{
    for (int i = 0; i < engine->objects->size; i++)
    {
        engine->objects->data[i].phys_comp.pos = Vector2D_Sum(
            engine->objects->data[i].phys_comp.pos,
            Vector2D_ScalarProduct(engine->objects->data[i].phys_comp.vel, engine->dt));

        if (engine->flags & BOUNDING_BOX)
        {
            if (engine->objects->data[i].phys_comp.pos.x < engine->objects->data[i].radius ||
                engine->objects->data[i].phys_comp.pos.x > WINDOW_WIDTH - engine->objects->data[i].radius)
            {
                engine->objects->data[i].phys_comp.vel.x *= -1;
                engine->objects->data[i].phys_comp.pos.x = SDL_clamp(
                    engine->objects->data[i].phys_comp.pos.x,
                    engine->objects->data[i].radius,
                    WINDOW_WIDTH - engine->objects->data[i].radius);
            }

            if (engine->objects->data[i].phys_comp.pos.y < engine->objects->data[i].radius ||
                engine->objects->data[i].phys_comp.pos.y > WINDOW_HEIGHT - engine->objects->data[i].radius)
            {
                engine->objects->data[i].phys_comp.vel.y *= -1;
                engine->objects->data[i].phys_comp.pos.y = SDL_clamp(
                    engine->objects->data[i].phys_comp.pos.y,
                    engine->objects->data[i].radius,
                    WINDOW_HEIGHT - engine->objects->data[i].radius);
            }
        }
        else
        {
            if (engine->objects->data[i].phys_comp.pos.x + engine->objects->data[i].radius < 0 - BUFFER_ZONE)
                engine->objects->data[i].alive = 0;
            else if (engine->objects->data[i].phys_comp.pos.y + engine->objects->data[i].radius < 0 - BUFFER_ZONE)
                engine->objects->data[i].alive = 0;
            else if (engine->objects->data[i].phys_comp.pos.x - engine->objects->data[i].radius >= WINDOW_WIDTH + BUFFER_ZONE)
                engine->objects->data[i].alive = 0;
            else if (engine->objects->data[i].phys_comp.pos.y - engine->objects->data[i].radius >= WINDOW_HEIGHT + BUFFER_ZONE)
                engine->objects->data[i].alive = 0;
        }
    }
}

void RenderFillCircle(SDL_Renderer *renderer, CIRCLE_OBJ *circle_obj)
{
    int x = circle_obj->phys_comp.pos.x;
    int y = circle_obj->phys_comp.pos.y;
    int r = circle_obj->radius;
    SDL_SetRenderDrawColor(renderer, circle_obj->color.r, circle_obj->color.g, circle_obj->color.b, SDL_ALPHA_OPAQUE);
    for (int i = x - r; i <= x + r; i++)
    {
        if (i < 0 || i >= WINDOW_WIDTH)
            continue;
        for (int j = y - r; j <= y + r; j++)
        {
            if (j < 0 || j >= WINDOW_HEIGHT)
                continue;
            double dist = Vector2D_Magnitude(Vector2D_Difference((VECTOR_2D){i, j}, (VECTOR_2D){x, y}));
            if (dist <= r)
                SDL_RenderDrawPoint(renderer, i, j);
        }
    }
}

int SDLCALL processUserInput(void *data)
{
    ENGINE_2D *engine = (ENGINE_2D *)data;
    printf("Supported Commands: create, clear, set, pause, resume\n");
    while (SDL_TRUE)
    {
        printf("$ ");
        char input[INPUT_BUFFER_SIZE];
        char command[10];
        fgets(input, INPUT_BUFFER_SIZE, stdin);
        sscanf(input, "%s", command);
        if (strcasecmp(command, "create") == 0)
            handleCreateCommand(engine, input);
        else if (strcasecmp(command, "clear") == 0)
            handleClearCommand(engine, input);
        else if (strcasecmp(command, "set") == 0)
            handleSetCommand(engine, input);
        else if (strcasecmp(command, "pause") == 0)
            handlePauseCommand(engine, input);
        else if (strcasecmp(command, "resume") == 0)
            handleResumeCommand(engine, input);
        else
            printf("command not supported: '%s'\n", command);
    }
}

void handleCreateCommand(ENGINE_2D *engine, char *input)
{
    char *command = strtok(input, DELIM);
    char *flag;
    char color_char = 'w';
    RGB24 color = RGB_WHITE;
    double radius = MIN_RADIUS, mass = π * radius * radius * DENSITY;
    VECTOR_2D pos = {WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2}, vel = {0, 0};
    while ((flag = strtok(NULL, DELIM)) != NULL)
    {
        if (strcasecmp(flag, "--help") == 0)
        {
            printf("Usage: create [OPTION]...\n");
            printf("Create a new object in the simulation\n");
            printf("\n");
            printf("Mandatory arguments to long options are mandatory for short options too.\n");
            printf("-c, --color LETTER\tchoose between primary and secondary colors by their first letter (default: %c)\n", color_char);
            printf("-r, --radius NUM\tset the radius of the circle object (default: %d)\n", (int)radius);
            printf("-m, --mass NUM\tset the mass of the circle object (default: %d)\n", (int)mass);
            printf("\t--posx NUM\tset the x coordinate of the center of the circle object (default: %d)\n", (int)pos.x);
            printf("\t--posy NUM\tset the y coordinate of the center of the circle object (default: %d)\n", (int)pos.y);
            printf("\t--velx NUM\tset the velocity of the circle object in the x-axis (default: %d)\n", (int)vel.x);
            printf("\t--vely NUM\tset the velocity of the circle object in the y-axis (default: %d)\n", (int)vel.y);
            printf("\t--help\tdisplay this help and exit\n");
        }
        else if (tryParseCharOptionArg(command, flag, "-c", "--color", &color_char))
        {
            switch (color_char)
            {
            case 'r':
                color = RGB_RED;
                break;
            case 'g':
                color = RGB_GREEN;
                break;
            case 'b':
                color = RGB_BLUE;
                break;
            case 'y':
                color = RGB_YELLOW;
                break;
            case 'c':
                color = RGB_CYAN;
                break;
            case 'm':
                color = RGB_MAGENTA;
                break;
            case 'w':
                break;
            default:
                printf("create: color '%c' is invalid, defaulting to white\n", color_char);
                printf("try 'create --help' for more information\n");
            }
        }
        else if (tryParseFloatOptionArg(command, flag, "-r", "--radius", &radius))
            ;
        else if (tryParseFloatOptionArg(command, flag, "-m", "--mass", &mass))
            ;
        else if (tryParseFloatOptionArg(command, flag, NULL, "--posx", &pos.x))
            ;
        else if (tryParseFloatOptionArg(command, flag, NULL, "--posy", &pos.y))
            ;
        else if (tryParseFloatOptionArg(command, flag, NULL, "--velx", &vel.x))
            ;
        else if (tryParseFloatOptionArg(command, flag, NULL, "--vely", &vel.y))
            ;
    }
    Engine2D_CreateCircleObject(engine, color, radius, (PHYS_BODY){mass, pos, vel});
}

void handleClearCommand(ENGINE_2D *engine, char *input)
{
    SDL_LockMutex(engine->shared_state_mutex);
    strtok(input, DELIM); // skip the command
    char *flag = strtok(NULL, DELIM);
    int id;
    if (flag == NULL || strcasecmp(flag, "--all") == 0 || strcasecmp(flag, "-a") == 0)
    {
        // clear the object array before the next frame
        engine->objects->size = 0;
        engine->objects->cap = DEFAULT_ARR_CAPACITY;
    }
    else if (sscanf(flag, "--id=%d", &id) == 1)
    {
        CIRCLE_OBJ *circleFound = findCircleById(engine->objects, id);
        if (circleFound != NULL)
            circleFound->alive = SDL_FALSE;
        else
            printf("clear: could not find circle with id: %d\n", id);
    }
    else if (strcasecmp(flag, "--id") == 0)
    {
        char *id_str = strtok(NULL, DELIM);
        if (id_str == NULL)
        {
            printf("clear: option requires an argument -- '%s'\n", flag);
            printf("Try 'clear --help' for more information.\n");
        }
        else
        {
            if (sscanf(id_str, "%d", &id) != 1)
            {
                printf("clear: invalid value for %s: expected integer, got '%s'\n", flag, id_str);
                printf("Try 'clear --help' for more information.\n");
            }
            else
            {
                CIRCLE_OBJ *circleFound = findCircleById(engine->objects, id);
                if (circleFound != NULL)
                    circleFound->alive = SDL_FALSE;
                else
                    printf("clear: could not find circle with id: %d\n", id);
            }
        }
    }
    else if (strcasecmp(flag, "--help") == 0)
    {
        printf("Usage: clear [OPTION]\n"
               "Clear all objects or optionally, a single one specified by its id.\n"
               "\n"
               "-a, --all\tclears all objects; same as \'clear\'"
               "\t--id[=]NUM\tclear only the object with id=NUM, if it exists\n"
               "\t--help\t\tdisplay this help and exit\n");
    }
    else
    {
        printf("clear: invalid option -- '%s'\n", flag);
        printf("Try 'clear --help' for more information.\n");
    }

    SDL_UnlockMutex(engine->shared_state_mutex);
}

void handleSetCommand(ENGINE_2D *engine, char *input)
{
    char *cmd = strtok(input, DELIM);
    char *flag;
    SDL_bool is_flag_provided = SDL_FALSE;
    while ((flag = strtok(NULL, DELIM)) != NULL)
    {
        is_flag_provided = SDL_TRUE;
        int num_arg;
        int arg_buf_size = 16;
        char arg_buf[arg_buf_size];
        if (sscanf(flag, "--elasticity=%d", &num_arg) == 1 || sscanf(flag, "-e=%d", &num_arg) == 1)
        {
            if (num_arg == ELASTIC && !(engine->flags & ELASTIC_COLLISION))
                engine->flags |= ELASTIC_COLLISION;
            else if (num_arg == INELASTIC && (engine->flags & ELASTIC_COLLISION))
                engine->flags &= ~ELASTIC_COLLISION;
            else
            {
                printf("set: elasticity can either be 0 or 1, not %d\n", num_arg);
                printf("Try 'set --help' for more information.\n");
            }
        }
        else if (strcasecmp(flag, "-e") == 0 || strcasecmp(flag, "--elasticity") == 0)
        {
            char *flag_val_str = strtok(NULL, DELIM);
            if (flag_val_str == NULL)
            {
                printf("set: option requires an argument -- '%s'\n", flag);
                printf("Try 'set --help' for more information.\n");
            }
            else if (sscanf(flag_val_str, "%d", &num_arg) != 1)
            {
                printf("set: invalid value for %s: expected integer, got '%s'\n", flag, flag_val_str);
                printf("Try 'set --help' for more information.\n");
            }
            else
            {
                if (num_arg == ELASTIC && !(engine->flags & ELASTIC_COLLISION))
                    engine->flags |= ELASTIC_COLLISION;
                else if (num_arg == INELASTIC && (engine->flags & ELASTIC_COLLISION))
                    engine->flags &= ~ELASTIC_COLLISION;
                else
                {
                    printf("set: elasticity can either be 0 or 1, not %d\n", num_arg);
                    printf("Try 'set --help' for more information.\n");
                }
            }
        }
        else if (tryParseStrOptionArg(cmd, flag, "-g", "--gravity", arg_buf, arg_buf_size))
        {
            if (strcasecmp(arg_buf, "on") == 0)
                engine->flags |= ENABLE_GRAVITY;
            else if (strcasecmp(arg_buf, "off") == 0)
                engine->flags &= ~ENABLE_GRAVITY;
            else
            {
                printf("set: gravity can either be 'on' or 'off', not %s\n", arg_buf);
                printf("Try 'set --help' for more information.\n");
            }
        }
        else if (strcasecmp(flag, "--help") == 0)
        {
            printf("Usage: set OPTION...\n"
                   "Set the value of any supported mathematical variable in the engine.\n"
                   "\n"
                   "Mandatory arguments to long options are mandatory for short options too.\n"
                   "-e, --elasticity[=]{0|1}\tset collisions to be inelastic (0), or perfectly elastic (1)\n"
                   "-g, --gravity STRING\tturn gravity 'on' or 'off'"
                   "\t--help\tdisplay this help and exit\n");
        }
        else
        {
            printf("set: invalid option -- '%s'\n", flag);
            printf("Try 'set --help' for more information.\n");
        }
    }
    if (!is_flag_provided)
    {
        printf("Usage: set OPTION...\n"
               "Try 'set --help' for more information.\n");
    }
}

void handlePauseCommand(ENGINE_2D *engine, char *input)
{
    strtok(input, DELIM); // skip the command
    char *flag = strtok(NULL, DELIM);
    if (flag == NULL)
        engine->flags |= PAUSED;
    else if (strcasecmp(flag, "--help") == 0)
    {
        printf("Usage: pause [OPTION]\n"
               "Pause the simulation if not already paused, otherwise do nothing.\n"
               "\n"
               "\t--help\tdisplay this help and exit\n");
    }
    else
    {
        printf("pause: invalid option -- '%s'\n", flag);
        printf("Try 'pause --help' for more information.\n");
    }
}

void handleResumeCommand(ENGINE_2D *engine, char *input)
{
    strtok(input, DELIM); // skip the command
    char *flag = strtok(NULL, DELIM);
    if (flag == NULL)
        engine->flags &= ~PAUSED;
    else if (strcasecmp(flag, "--help") == 0)
    {
        printf("Usage: resume [OPTION]\n"
               "Resume the simulation if paused, otherwise do nothing.\n"
               "\n"
               "\t--help\tdisplay this help and exit\n");
    }
    else
    {
        printf("resume: invalid option -- '%s'\n", flag);
        printf("Try 'resume --help' for more information.\n");
    }
}

CIRCLE_OBJ *findCircleById(OBJECT_ARRAY *objects, int id)
{
    for (int i = 0; i < objects->size; i++)
    {
        if (objects->data[i].id == id)
            return objects->data + i;
    }
    return NULL;
}

SDL_bool tryParseIntOptionArg(char *command_name, char *input_flag, char *short_option, char *long_option, int *p_arg_value)
{
    if (short_option == NULL)
        short_option = long_option;
    SDL_bool parse_status = SDL_FALSE;
    if (strcasecmp(input_flag, short_option) == 0 || strcasecmp(input_flag, long_option) == 0)
    {
        char *arg_str = strtok(NULL, DELIM);
        if (arg_str == NULL)
        {
            printf("%s: option requires an argument -- '%s'\n", command_name, input_flag);
            printf("Try '%s --help' for more information.\n", command_name);
        }
        else if (sscanf(arg_str, "%d", p_arg_value) != 1)
        {
            printf("%s: invalid value for %s: expected integer, got '%s'\n", command_name, input_flag, arg_str);
            printf("Try '%s --help' for more information.\n", command_name);
        }
        else
        {
            parse_status = SDL_TRUE;
        }
    }
    return parse_status;
}

SDL_bool tryParseFloatOptionArg(char *command_name, char *input_flag, char *short_option, char *long_option, double *p_arg_value)
{
    if (short_option == NULL)
        short_option = long_option;
    SDL_bool parse_status = SDL_FALSE;
    if (strcasecmp(input_flag, short_option) == 0 || strcasecmp(input_flag, long_option) == 0)
    {
        char *arg_str = strtok(NULL, DELIM);
        if (arg_str == NULL)
        {
            printf("%s: option requires an argument -- '%s'\n", command_name, input_flag);
            printf("Try '%s --help' for more information.\n", command_name);
        }
        else if (sscanf(arg_str, "%lf", p_arg_value) != 1)
        {
            printf("%s: invalid value for %s: expected float, got '%s'\n", command_name, input_flag, arg_str);
            printf("Try '%s --help' for more information.\n", command_name);
        }
        else
        {
            parse_status = SDL_TRUE;
        }
    }
    return parse_status;
}

SDL_bool tryParseCharOptionArg(char *command_name, char *input_flag, char *short_option, char *long_option, char *p_arg_value)
{
    if (short_option == NULL)
        short_option = long_option;
    SDL_bool parse_status = SDL_FALSE;
    if (strcasecmp(input_flag, short_option) == 0 || strcasecmp(input_flag, long_option) == 0)
    {
        char *arg_str = strtok(NULL, DELIM);
        if (arg_str == NULL)
        {
            printf("%s: option requires an argument -- '%s'\n", command_name, input_flag);
            printf("Try '%s --help' for more information.\n", command_name);
        }
        else if (sscanf(arg_str, "%c", p_arg_value) != 1)
        {
            printf("%s: invalid value for %s: expected character, got '%s'\n", command_name, input_flag, arg_str);
            printf("Try '%s --help' for more information.\n", command_name);
        }
        else
        {
            parse_status = SDL_TRUE;
        }
    }
    return parse_status;
}

SDL_bool tryParseStrOptionArg(char *command_name, char *input_flag, char *short_option, char *long_option, char *buffer, int buf_size)
{
    if (short_option == NULL)
        short_option = long_option;
    SDL_bool parse_status = SDL_FALSE;
    if (strcasecmp(input_flag, short_option) == 0 || strcasecmp(input_flag, long_option) == 0)
    {
        char *arg = strtok(NULL, DELIM);
        if (arg == NULL)
        {
            printf("%s: option requires an argument -- '%s'\n", command_name, input_flag);
            printf("Try '%s --help' for more information.\n", command_name);
        }
        else if (SDL_strlcpy(buffer, arg, buf_size) > 0)
        {
            parse_status = SDL_TRUE;
        }
    }
    return parse_status;
}