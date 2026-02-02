#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include "colors.h"
#include "vector2d.h"

#define WINDOW_WIDTH 1280
#define WINDOW_HEIGHT 720
#define FRAMES_PER_SEC 30
#define DEFAULT_ARR_CAPACITY 128
#define PIXELS_PER_METER 1024
#define MIN_RADIUS 8
#define MAX_RADIUS 16
#define DENSITY 768
#define LOG_FILE "log.txt"
#define LOG_INTERVAL_SECS 1
#define INPUT_BUFFER_SIZE 256
#define BOUNCE 1
#define MERGE 0
#define DELIM " \t\r\n"
#define DEFAULT_SPEED 196
#define BUFFER_ZONE 128
#define STARTUP_FRAMES 5

enum MODES
{
    ELASTIC = 1,
    GRAVITY = 2,
    MOVE = 4,
    WALLED = 8,
    LOG = 16,
};

typedef struct
{
    double mass;
    VECTOR_2D pos, vel;
} PHYS_BODY;

typedef struct
{
    SDL_bool alive;
    Uint16 id;
    RGB24 color;
    double radius;
    PHYS_BODY phys_comp;
} CIRCLE_OBJ;

const double π = 3.141592653589793;
const double G = 6.6743E-11 * PIXELS_PER_METER * PIXELS_PER_METER * PIXELS_PER_METER;
const double dt = 1.0 / FRAMES_PER_SEC;

FILE *log_file;
SDL_Renderer *renderer;
CIRCLE_OBJ *circle_object_arr;
int arr_cap = DEFAULT_ARR_CAPACITY, arr_size = 0;
SDL_mutex *shared_data_mutex;
SDL_bool is_simulation_paused = SDL_FALSE;
SDL_bool is_collision_elastic;
SDL_bool gravity_enabled;
SDL_bool spawn_moving;
SDL_bool walls_enabled;
SDL_bool logging_enabled;

void setMode(int mode);
SDL_bool isPointInsideCircle(VECTOR_2D point, CIRCLE_OBJ circle_obj);
void logArrInfo();
void logInfoOf(CIRCLE_OBJ circle_obj);
void createNewCircleObj(RGB24 color, double radius, double mass, VECTOR_2D pos, VECTOR_2D vel);
void runSimulation();
void sanitiseObjectArray();
void simulateForces();
void simulateGravitationalForce();
void handleCollision(CIRCLE_OBJ *c1, CIRCLE_OBJ *c2);
void updatePositionsAndCheckBounds();
void RenderFillCircle(CIRCLE_OBJ *circle_obj);
int processUserInput(void *data);
void handleCreateCommand(char *input);
void handleClearCommand(char *input);
void handleSetCommand(char *input);
void handlePauseCommand(char *input);
void handleResumeCommand(char *input);
CIRCLE_OBJ *findCircleById(int id);
SDL_bool tryParseIntOptionArg(char *command_name, char *input_flag, char *short_option, char *long_option, int *p_arg_value);
SDL_bool tryParseFloatOptionArg(char *command_name, char *input_flag, char *short_option, char *long_option, double *p_arg_value);
SDL_bool tryParseCharOptionArg(char *command_name, char *input_flag, char *short_option, char *long_option, char *p_arg_value);
SDL_bool tryParseStrOptionArg(char *command_name, char *input_flag, char *short_option, char *long_option, char *buffer, int buf_size);

int main()
{
    Uint64 engine_start = SDL_GetPerformanceCounter();
    setMode(GRAVITY);
    srand(time(NULL));
    SDL_Init(SDL_INIT_EVERYTHING);
    SDL_Window *window = SDL_CreateWindow(
        "Physics Engine",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        WINDOW_WIDTH,
        WINDOW_HEIGHT,
        0);
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    circle_object_arr = (CIRCLE_OBJ *)calloc(arr_cap, sizeof(CIRCLE_OBJ));
    shared_data_mutex = SDL_CreateMutex();
    SDL_CreateThread(processUserInput, "input thread", NULL);
    if (logging_enabled)
        log_file = fopen(LOG_FILE, "w");

    // for (int i = 0; i < arr_cap; i++)
    // {
    //     int radius = MIN_RADIUS + rand() % (MAX_RADIUS - MIN_RADIUS);
    //     VECTOR_2D pos = {
    //         .x = rand() % WINDOW_WIDTH,
    //         .y = rand() % WINDOW_HEIGHT,
    //     };
    //     VECTOR_2D vel = {
    //         .x = (double)rand() / RAND_MAX * (rand() % 2 ? 1 : -1) * DEFAULT_SPEED * spawn_moving,
    //         .y = (double)rand() / RAND_MAX * (rand() % 2 ? 1 : -1) * DEFAULT_SPEED * spawn_moving,
    //     };
    //     createNewCircleObj(generateVividColor(), radius, π * radius * radius * DENSITY, pos, vel);
    // }

    // Central massive body (like the Sun)
    VECTOR_2D pos1 = {WINDOW_WIDTH / 2.0, WINDOW_HEIGHT / 2.0};
    VECTOR_2D vel1 = {0, 0};
    double radius1 = 40;
    double mass1 = 100000000;
    createNewCircleObj(RGB_YELLOW, radius1, mass1, pos1, vel1);

    // Smaller orbiting body (like a planet)
    double distance = 250; // distance from center
    VECTOR_2D pos2 = {pos1.x + distance, pos1.y};
    double radius2 = 20; // smaller radius
    double mass2 = 1000;

    // Circular orbit velocity perpendicular to radius
    double v = SDL_sqrt(G * mass1 / distance);
    VECTOR_2D vel2 = {0, -v}; // moving upwards for clockwise orbit
    createNewCircleObj(RGB_CYAN, radius2, mass2, pos2, vel2);

    int frames = 0, frames_over_dt = 0;
    double frame_time_sum = 0, max_frame_time = 0, min_frame_time = dt;
    SDL_bool application_running = SDL_TRUE;
    while (application_running)
    {
        Uint64 frame_start = SDL_GetPerformanceCounter();
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            switch (event.type)
            {
            case SDL_QUIT:
                application_running = SDL_FALSE;
                break;
            case SDL_MOUSEBUTTONDOWN:
                switch (event.button.button)
                {
                case SDL_BUTTON_LEFT:
                    VECTOR_2D point = {event.button.x, event.button.y};
                    SDL_LockMutex(shared_data_mutex);
                    for (int i = 0; i < arr_size; i++)
                    {
                        if (isPointInsideCircle(point, circle_object_arr[i]))
                        {
                            printf("ID: %d\n", circle_object_arr[i].id);
                            fflush(stdout);
                            break;
                        }
                    }
                    SDL_UnlockMutex(shared_data_mutex);
                    break;
                }
                break;
            }
        }
        if (is_simulation_paused)
        {
            SDL_Delay(dt * 1000);
            continue;
        }
        SDL_SetRenderDrawColor(renderer, RGB_BLACK.r, RGB_BLACK.g, RGB_BLACK.b, SDL_ALPHA_OPAQUE);
        SDL_RenderClear(renderer);
        runSimulation();
        SDL_RenderPresent(renderer);
        frames++;
        if (logging_enabled && frames % (LOG_INTERVAL_SECS * FRAMES_PER_SEC) == 0)
            logArrInfo();
        Uint64 frame_end = SDL_GetPerformanceCounter();
        double frame_time = (double)(frame_end - frame_start) / SDL_GetPerformanceFrequency();
        if (frames > STARTUP_FRAMES)
        {
            frame_time_sum += frame_time;
            if (frame_time < min_frame_time)
                min_frame_time = frame_time;
            if (frame_time > max_frame_time)
                max_frame_time = frame_time;
        }
        if (frame_time < dt)
            SDL_Delay((dt - frame_time) * 1000);
        else
        {
            printf("%d\n", frames);
            frames_over_dt++;
        }
    }

    Uint64 engine_end = SDL_GetPerformanceCounter();
    printf("Time passed:\t%.2lf s\n", (double)(engine_end - engine_start) / SDL_GetPerformanceFrequency());
    printf("No. of frames:\t%d\n", frames);
    printf("Frames over dt:\t%d\n", frames_over_dt);
    printf("After excluding %d frames during startup:\n", STARTUP_FRAMES);
    printf("Avg. Frame Time: %.2lf ms\n", frame_time_sum / (frames - STARTUP_FRAMES) * 1000);
    printf("Min. Frame Time: %.2lf ms\n", min_frame_time * 1000);
    printf("Max. Frame Time: %.2lf ms\n", max_frame_time * 1000);

    if (logging_enabled)
        fclose(log_file);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyMutex(shared_data_mutex);
    SDL_DestroyWindow(window);
    SDL_Quit();
    free(circle_object_arr);
    return 0;
}

void setMode(int mode)
{
    is_collision_elastic = (mode & ELASTIC) != 0;
    gravity_enabled = (mode & GRAVITY) != 0;
    spawn_moving = (mode & MOVE) != 0;
    walls_enabled = (mode & WALLED) != 0;
    logging_enabled = (mode & LOG) != 0;
}

SDL_bool isPointInsideCircle(VECTOR_2D point, CIRCLE_OBJ circle_obj)
{
    VECTOR_2D dist = Vector2D_Difference(point, circle_obj.phys_comp.pos);
    return Vector2D_Magnitude(dist) <= circle_obj.radius;
}

void logArrInfo()
{
    static int log_count = 1;
    fprintf(log_file, "ENTRY: #%d\n", log_count);
    SDL_LockMutex(shared_data_mutex);
    for (int i = 0; i < arr_size; i++)
    {
        fprintf(log_file, "Circle %d:\n", circle_object_arr[i].id);
        logInfoOf(circle_object_arr[i]);
    }
    SDL_UnlockMutex(shared_data_mutex);
    log_count++;
}

void logInfoOf(CIRCLE_OBJ circle_obj)
{
    if (!circle_obj.alive)
    {
        fprintf(log_file, "is Dead.\n");
        return;
    }
    fprintf(log_file, "Radius = %.2lf\n", circle_obj.radius);
    fprintf(log_file, "Mass = %.2lf\n", circle_obj.phys_comp.mass);
    fprintf(log_file, "Position = (%.2lf, %.2lf)\n", circle_obj.phys_comp.pos.x, circle_obj.phys_comp.pos.y);
    fprintf(log_file, "Velocity = (%.2lf, %.2lf)\n", circle_obj.phys_comp.vel.x, circle_obj.phys_comp.vel.y);
}

void createNewCircleObj(RGB24 color, double radius, double mass, VECTOR_2D pos, VECTOR_2D vel)
{
    static int id = 1;

    CIRCLE_OBJ circle_obj = {
        .alive = SDL_TRUE,
        .id = id++,
        .color = color,
        .radius = radius,
        .phys_comp = (PHYS_BODY){
            .mass = mass,
            .pos = pos,
            .vel = vel,
        },
    };

    SDL_LockMutex(shared_data_mutex);

    if (arr_size >= arr_cap)
    {
        CIRCLE_OBJ *temp = (CIRCLE_OBJ *)realloc(circle_object_arr, arr_cap * 4 * sizeof(CIRCLE_OBJ));
        if (temp)
        {
            arr_cap *= 4;
            circle_object_arr = temp;
        }
        else
            fprintf(stderr, "REALLOCATION FAILED in %s\n", __func__);
    }
    circle_object_arr[arr_size++] = circle_obj;

    SDL_UnlockMutex(shared_data_mutex);
}

void runSimulation()
{
    SDL_LockMutex(shared_data_mutex);

    sanitiseObjectArray();
    simulateForces();
    updatePositionsAndCheckBounds();
    for (int i = 0; i < arr_size; i++)
        RenderFillCircle(circle_object_arr + i);

    SDL_UnlockMutex(shared_data_mutex);
}

void sanitiseObjectArray()
{
    int i = 0, j = arr_size;
    while (i < j)
    {
        if (circle_object_arr[i].alive)
            i++;
        else
            circle_object_arr[i] = circle_object_arr[--j];
    }
    // all elements before i are alive, and all elements from j onwards are dead
    // when i == j, the loop ends and i points to a dead element
    // therefore total num of alive elements is i
    arr_size = i;

    if (arr_size < arr_cap / 8)
    {
        CIRCLE_OBJ *temp = (CIRCLE_OBJ *)realloc(circle_object_arr, arr_cap / 2 * sizeof(CIRCLE_OBJ));
        if (temp)
        {
            arr_cap /= 2;
            circle_object_arr = temp;
        }
        else
            fprintf(stderr, "REALLOCATION FAILED in %s\n", __func__);
    }
}

void simulateForces()
{
    simulateGravitationalForce();
}

void simulateGravitationalForce()
{
    for (int i = 0; i < arr_size - 1; i++)
    {
        if (!circle_object_arr[i].alive)
            continue;
        for (int j = i + 1; j < arr_size; j++)
        {
            if (!circle_object_arr[j].alive)
                continue;
            double m1 = circle_object_arr[i].phys_comp.mass;
            double m2 = circle_object_arr[j].phys_comp.mass;
            VECTOR_2D pos1 = circle_object_arr[i].phys_comp.pos;
            VECTOR_2D pos2 = circle_object_arr[j].phys_comp.pos;
            VECTOR_2D dist_vec = Vector2D_Difference(pos2, pos1);
            double dist = Vector2D_Magnitude(dist_vec);
            if (dist < circle_object_arr[i].radius + circle_object_arr[j].radius)
            {
                handleCollision(circle_object_arr + i, circle_object_arr + j);
                if (is_collision_elastic)
                    // stop calculating with dead circles[j] in case of merge
                    continue;
            }
            if (gravity_enabled)
            {
                double force_magnitude = G * m1 * m2 / SDL_pow(dist, 2);
                VECTOR_2D force = Vector2D_ScalarProduct(
                    Vector2D_Normalised(dist_vec),
                    force_magnitude);

                circle_object_arr[i].phys_comp.vel = Vector2D_Sum(
                    circle_object_arr[i].phys_comp.vel,
                    Vector2D_ScalarProduct(force, dt / m1));

                circle_object_arr[j].phys_comp.vel = Vector2D_Difference(
                    circle_object_arr[j].phys_comp.vel,
                    Vector2D_ScalarProduct(force, dt / m2));
            }
        }
    }
}

void handleCollision(CIRCLE_OBJ *c1, CIRCLE_OBJ *c2)
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

void updatePositionsAndCheckBounds()
{
    for (int i = 0; i < arr_size; i++)
    {
        circle_object_arr[i].phys_comp.pos = Vector2D_Sum(
            circle_object_arr[i].phys_comp.pos,
            Vector2D_ScalarProduct(circle_object_arr[i].phys_comp.vel, dt));

        if (walls_enabled)
        {
            if (circle_object_arr[i].phys_comp.pos.x < circle_object_arr[i].radius ||
                circle_object_arr[i].phys_comp.pos.x > WINDOW_WIDTH - circle_object_arr[i].radius)
            {
                circle_object_arr[i].phys_comp.vel.x *= -1;
                circle_object_arr[i].phys_comp.pos.x = SDL_clamp(
                    circle_object_arr[i].phys_comp.pos.x,
                    circle_object_arr[i].radius,
                    WINDOW_WIDTH - circle_object_arr[i].radius);
            }

            if (circle_object_arr[i].phys_comp.pos.y < circle_object_arr[i].radius ||
                circle_object_arr[i].phys_comp.pos.y > WINDOW_HEIGHT - circle_object_arr[i].radius)
            {
                circle_object_arr[i].phys_comp.vel.y *= -1;
                circle_object_arr[i].phys_comp.pos.y = SDL_clamp(
                    circle_object_arr[i].phys_comp.pos.y,
                    circle_object_arr[i].radius,
                    WINDOW_HEIGHT - circle_object_arr[i].radius);
            }
        }
        else
        {
            if (circle_object_arr[i].phys_comp.pos.x + circle_object_arr[i].radius < 0 - BUFFER_ZONE)
                circle_object_arr[i].alive = 0;
            else if (circle_object_arr[i].phys_comp.pos.y + circle_object_arr[i].radius < 0 - BUFFER_ZONE)
                circle_object_arr[i].alive = 0;
            else if (circle_object_arr[i].phys_comp.pos.x - circle_object_arr[i].radius >= WINDOW_WIDTH + BUFFER_ZONE)
                circle_object_arr[i].alive = 0;
            else if (circle_object_arr[i].phys_comp.pos.y - circle_object_arr[i].radius >= WINDOW_HEIGHT + BUFFER_ZONE)
                circle_object_arr[i].alive = 0;
        }
    }
}

void RenderFillCircle(CIRCLE_OBJ *circle_obj)
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
    printf("Supported Commands: create, clear, set, pause, resume\n");
    while (SDL_TRUE)
    {
        printf("$ ");
        char input[INPUT_BUFFER_SIZE];
        char command[10];
        fgets(input, INPUT_BUFFER_SIZE, stdin);
        sscanf(input, "%s", command);
        if (strcasecmp(command, "create") == 0)
            handleCreateCommand(input);
        else if (strcasecmp(command, "clear") == 0)
            handleClearCommand(input);
        else if (strcasecmp(command, "set") == 0)
            handleSetCommand(input);
        else if (strcasecmp(command, "pause") == 0)
            handlePauseCommand(input);
        else if (strcasecmp(command, "resume") == 0)
            handleResumeCommand(input);
        else
            printf("command not supported: '%s'\n", command);
    }
}

void handleCreateCommand(char *input)
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
    createNewCircleObj(color, radius, mass, pos, vel);
}

void handleClearCommand(char *input)
{
    SDL_LockMutex(shared_data_mutex);
    strtok(input, DELIM); // skip the command
    char *flag = strtok(NULL, DELIM);
    int id;
    if (flag == NULL || strcasecmp(flag, "--all") == 0 || strcasecmp(flag, "-a") == 0)
    {
        // clear the object array before the next frame
        arr_size = 0;
        arr_cap = DEFAULT_ARR_CAPACITY;
    }
    else if (sscanf(flag, "--id=%d", &id) == 1)
    {
        CIRCLE_OBJ *circleFound = findCircleById(id);
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
                CIRCLE_OBJ *circleFound = findCircleById(id);
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

    SDL_UnlockMutex(shared_data_mutex);
}

void handleSetCommand(char *input)
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
            if (num_arg == BOUNCE || num_arg == MERGE)
                is_collision_elastic = num_arg;
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
                if (num_arg == BOUNCE || num_arg == MERGE)
                    is_collision_elastic = num_arg;
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
            {
                SDL_LockMutex(shared_data_mutex);
                gravity_enabled = SDL_TRUE;
                SDL_UnlockMutex(shared_data_mutex);
            }
            else if (strcasecmp(arg_buf, "off") == 0)
            {
                SDL_LockMutex(shared_data_mutex);
                gravity_enabled = SDL_FALSE;
                SDL_UnlockMutex(shared_data_mutex);
            }
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

void handlePauseCommand(char *input)
{
    strtok(input, DELIM); // skip the command
    char *flag = strtok(NULL, DELIM);
    if (flag == NULL)
    {
        is_simulation_paused = SDL_TRUE;
    }
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

void handleResumeCommand(char *input)
{
    strtok(input, DELIM); // skip the command
    char *flag = strtok(NULL, DELIM);
    if (flag == NULL)
    {
        is_simulation_paused = SDL_FALSE;
    }
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

CIRCLE_OBJ *findCircleById(int id)
{
    for (int i = 0; i < arr_size; i++)
    {
        if (circle_object_arr[i].id == id)
            return circle_object_arr + i;
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