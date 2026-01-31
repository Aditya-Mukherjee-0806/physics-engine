#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include "colors.h"

#define WINDOW_WIDTH 1280
#define WINDOW_HEIGHT 720
#define FRAMES_PER_SEC 60
#define DEFAULT_ARR_CAPACITY 64
#define PIXELS_PER_METER 1024
#define MIN_RADIUS 8
#define MAX_RADIUS 16
#define DENSITY 512
#define LOG_FILE "log.txt"
#define LOG_INTERVAL_SECS 1
#define INPUT_BUFFER_SIZE 256
#define PERFECTLY_ELASTIC 1
#define INELASTIC 0
#define DEFAULT_ELASTICITY INELASTIC
#define DELIM " \t\r\n"

typedef struct
{
    double x, y;
} VECTOR_2D;

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
Uint8 elasticity = DEFAULT_ELASTICITY;

SDL_bool isPointInsideCircle(VECTOR_2D point, CIRCLE_OBJ circle_obj);
void logArrInfo();
void logInfoOf(CIRCLE_OBJ circle_obj);
void createNewCircleObj(RGB24 color, double radius, double mass, VECTOR_2D pos, VECTOR_2D vel);
void runSimulation();
void sanitiseObjectArray();
void simulateForces();
void simulateGravitationalForce();
void handleCollision(CIRCLE_OBJ *c1, CIRCLE_OBJ *c2);
void updatePositions();
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

int main()
{
    srand(time(NULL));
    SDL_Init(SDL_INIT_EVERYTHING);
    SDL_Window *window = SDL_CreateWindow("Physics Engine", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WINDOW_WIDTH, WINDOW_HEIGHT, 0);
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    circle_object_arr = (CIRCLE_OBJ *)calloc(arr_cap, sizeof(CIRCLE_OBJ));
    shared_data_mutex = SDL_CreateMutex();
    SDL_CreateThread(processUserInput, "input thread", NULL);
    log_file = fopen(LOG_FILE, "w");

    for (int i = 0; i < arr_cap; i++)
    {
        int radius = MIN_RADIUS + rand() % (MAX_RADIUS - MIN_RADIUS);
        VECTOR_2D pos = {rand() % WINDOW_WIDTH, rand() % WINDOW_HEIGHT};
        VECTOR_2D vel = {
            (double)rand() / RAND_MAX * (rand() % 2 ? 1 : -1),
            (double)rand() / RAND_MAX * (rand() % 2 ? 1 : -1),
        };
        createNewCircleObj(generateVividColor(), radius, π * radius * radius * DENSITY, pos, vel);
    }

    // // Central massive body (like the Sun)
    // VECTOR_2D pos1 = {WINDOW_WIDTH / 2.0, WINDOW_HEIGHT / 2.0};
    // VECTOR_2D vel1 = {0, 0};
    // double radius1 = 40;
    // double mass1 = 100000000;
    // createNewCircleObj(SDL_MapRGB(surface->format, RGB_YELLOW), radius1, mass1, pos1, vel1);

    // // Smaller orbiting body (like a planet)
    // double distance = 300; // distance from center
    // VECTOR_2D pos2 = {pos1.x + distance, pos1.y};
    // double radius2 = 20; // smaller radius
    // double mass2 = 100000;

    // // Circular orbit velocity perpendicular to radius
    // double v = SDL_sqrt(G * mass1 / distance);
    // VECTOR_2D vel2 = {0, -v}; // moving upwards for clockwise orbit
    // createNewCircleObj(SDL_MapRGB(surface->format, RGB_CYAN), radius2, mass2, pos2, vel2);

    SDL_SetRenderDrawColor(renderer, RGB_BLACK.r, RGB_BLACK.g, RGB_BLACK.b, SDL_ALPHA_OPAQUE);
    SDL_RenderClear(renderer);
    SDL_RenderPresent(renderer);

    int frames = 0, frames_over_dt = 0;
    double frame_time_sum = 0, max_frame_time = 0, min_frame_time = dt;
    SDL_bool application_running = SDL_TRUE;
    while (application_running)
    {
        Uint64 start = SDL_GetPerformanceCounter();
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
            SDL_Delay(dt);
            continue;
        }
        SDL_SetRenderDrawColor(renderer, RGB_BLACK.r, RGB_BLACK.g, RGB_BLACK.b, SDL_ALPHA_OPAQUE);
        SDL_RenderClear(renderer);
        runSimulation();
        SDL_RenderPresent(renderer);
        if (++frames % (LOG_INTERVAL_SECS * FRAMES_PER_SEC) == 0)
            logArrInfo();
        Uint64 end = SDL_GetPerformanceCounter();
        double frame_time = (double)(end - start) / SDL_GetPerformanceFrequency();
        if(frames > 1)
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
            frames_over_dt++;
    }

    printf("Time passed:\t\t%.2lf s\n", (double)frames / FRAMES_PER_SEC);
    printf("Number of frames:\t%d\n", frames);
    printf("Frames over dt:\t\t%d\n", frames_over_dt);
    printf("Avg. Frame Time:\t%.2lf ms\n", frame_time_sum / frames * 1000);
    printf("Min. Frame Time:\t%.2lf ms\n", min_frame_time * 1000);
    printf("Max. Frame Time:\t%.2lf ms\n", max_frame_time * 1000);

    SDL_DestroyRenderer(renderer);
    SDL_DestroyMutex(shared_data_mutex);
    SDL_DestroyWindow(window);
    SDL_Quit();
    fclose(log_file);
    free(circle_object_arr);
    return 0;
}

SDL_bool isPointInsideCircle(VECTOR_2D point, CIRCLE_OBJ circle_obj)
{
    double dist_sq = SDL_pow(point.x - circle_obj.phys_comp.pos.x, 2) + SDL_pow(point.y - circle_obj.phys_comp.pos.y, 2);
    return dist_sq <= circle_obj.radius * circle_obj.radius;
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
        fprintf(log_file, "is Null.\n");
        return;
    }
    fprintf(log_file, "Radius = %lf\n", circle_obj.radius);
    fprintf(log_file, "Mass = %lf\n", circle_obj.phys_comp.mass);
    fprintf(log_file, "Position = (%lf, %lf)\n", circle_obj.phys_comp.pos.x, circle_obj.phys_comp.pos.y);
    fprintf(log_file, "Velocity = (%lf, %lf)\n", circle_obj.phys_comp.vel.x, circle_obj.phys_comp.vel.y);
}

void createNewCircleObj(RGB24 color, double radius, double mass, VECTOR_2D pos, VECTOR_2D vel)
{
    static int id = 1;

    PHYS_BODY phys_comp = {
        .mass = mass,
        .pos = pos,
        .vel = vel,
    };

    CIRCLE_OBJ circle_obj = {
        .alive = SDL_TRUE,
        .id = id++,
        .color = color,
        .radius = radius,
        .phys_comp = phys_comp,
    };

    SDL_LockMutex(shared_data_mutex);

    if (arr_size >= arr_cap)
    {
        CIRCLE_OBJ *temp = (CIRCLE_OBJ *)realloc(circle_object_arr, arr_cap * 2 * sizeof(CIRCLE_OBJ));
        if (temp)
        {
            arr_cap *= 2;
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
    simulateForces(elasticity);
    updatePositions();
    for (int i = 0; i < arr_size; i++)
        RenderFillCircle(circle_object_arr + i);

    SDL_UnlockMutex(shared_data_mutex);
}

void sanitiseObjectArray()
{
    for (int i = 0; i < arr_size; i++)
    {
        if (circle_object_arr[i].alive)
            continue;
        for (int j = i + 1; j < arr_size; j++)
            circle_object_arr[j - 1] = circle_object_arr[j];
        arr_size--;
        i--;
    }

    if (arr_size < arr_cap / 4)
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
            double dist = SDL_sqrt(SDL_pow(pos1.x - pos2.x, 2) + SDL_pow(pos1.y - pos2.y, 2));
            if (dist < circle_object_arr[i].radius + circle_object_arr[j].radius)
            {
                handleCollision(circle_object_arr + i, circle_object_arr + j);
                if (elasticity == INELASTIC)
                    // stop calculating with dead circles[j] in case of merge
                    continue;
            }
            double force_magnitude = G * m1 * m2 / SDL_pow(dist, 3);
            VECTOR_2D force;
            force.x = force_magnitude * (pos2.x - pos1.x);
            force.y = force_magnitude * (pos2.y - pos1.y);
            circle_object_arr[i].phys_comp.vel.x += force.x / m1 * dt;
            circle_object_arr[i].phys_comp.vel.y += force.y / m1 * dt;
            circle_object_arr[j].phys_comp.vel.x -= force.x / m2 * dt;
            circle_object_arr[j].phys_comp.vel.y -= force.y / m2 * dt;
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
    if (elasticity == 1)
    {
        // bounce c1 and c2 off each other
        c1->phys_comp.vel.x = ((m1 - m2) * u1.x + 2 * m2 * u2.x) / (m1 + m2);
        c1->phys_comp.vel.y = ((m1 - m2) * u1.y + 2 * m2 * u2.y) / (m1 + m2);

        c2->phys_comp.vel.x = ((m2 - m1) * u2.x + 2 * m1 * u1.x) / (m1 + m2);
        c2->phys_comp.vel.y = ((m2 - m1) * u2.y + 2 * m1 * u1.y) / (m1 + m2);
    }
    else
    {
        // Merge c2 into c1
        // Conservation of Linear Momentum
        c1->phys_comp.vel.x = (m1 * u1.x + m2 * u2.x) / (m1 + m2);
        c1->phys_comp.vel.y = (m1 * u1.y + m2 * u2.y) / (m1 + m2);
        // Conservation of Centre of Mass
        c1->phys_comp.pos.x = (m1 * pos1.x + m2 * pos2.x) / (m1 + m2);
        c1->phys_comp.pos.y = (m1 * pos1.y + m2 * pos2.y) / (m1 + m2);
        // mass of new body is the combined mass of both bodies, and radius is recalculated according to new mass
        c1->phys_comp.mass += c2->phys_comp.mass;
        c1->radius = SDL_sqrt(c1->phys_comp.mass / (π * DENSITY));
        // destroy c2
        c2->alive = 0;
    }
}

void updatePositions()
{
    for (int i = 0; i < arr_size; i++)
    {
        if (circle_object_arr[i].phys_comp.vel.x)
            circle_object_arr[i].phys_comp.pos.x += circle_object_arr[i].phys_comp.vel.x * dt;
        if (circle_object_arr[i].phys_comp.vel.y)
            circle_object_arr[i].phys_comp.pos.y += circle_object_arr[i].phys_comp.vel.y * dt;

        if (circle_object_arr[i].phys_comp.pos.x + circle_object_arr[i].radius <= 0)
            circle_object_arr[i].alive = 0;
        else if (circle_object_arr[i].phys_comp.pos.y + circle_object_arr[i].radius <= 0)
            circle_object_arr[i].alive = 0;
        else if (circle_object_arr[i].phys_comp.pos.x - circle_object_arr[i].radius >= WINDOW_WIDTH)
            circle_object_arr[i].alive = 0;
        else if (circle_object_arr[i].phys_comp.pos.y - circle_object_arr[i].radius >= WINDOW_HEIGHT)
            circle_object_arr[i].alive = 0;
    }
}

void RenderFillCircle(CIRCLE_OBJ *circle_obj)
{
    int x = circle_obj->phys_comp.pos.x;
    int y = circle_obj->phys_comp.pos.y;
    int r = circle_obj->radius;
    SDL_SetRenderDrawColor(renderer, circle_obj->color.r, circle_obj->color.g, circle_obj->color.b, SDL_ALPHA_OPAQUE);
    for (int i = x - r; i < x + r; i++)
    {
        if (i < 0 || i >= WINDOW_WIDTH - 1)
            continue;
        for (int j = y - r; j < y + r; j++)
        {
            if (j < 0 || j >= WINDOW_HEIGHT - 1)
                continue;
            double dist_sqr = SDL_pow(i - x, 2) + SDL_pow(j - y, 2);
            if (dist_sqr <= r * r)
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
    strtok(input, DELIM); // skip the command
    char *flag;
    SDL_bool is_flag_provided = SDL_FALSE;
    while ((flag = strtok(NULL, DELIM)) != NULL)
    {
        is_flag_provided = SDL_TRUE;
        int flag_value;
        if (sscanf(flag, "--elasticity=%d", &flag_value) == 1 || sscanf(flag, "-e=%d", &flag_value) == 1)
        {
            if (flag_value == PERFECTLY_ELASTIC || flag_value == INELASTIC)
                elasticity = flag_value;
            else
            {
                printf("set: elasticity can either be 0 or 1, not %d\n", flag_value);
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
            else if (sscanf(flag_val_str, "%d", &flag_value) != 1)
            {
                printf("set: invalid value for %s: expected integer, got '%s'\n", flag, flag_val_str);
                printf("Try 'set --help' for more information.\n");
            }
            else
            {
                if (flag_value == PERFECTLY_ELASTIC || flag_value == INELASTIC)
                    elasticity = flag_value;
                else
                {
                    printf("set: elasticity can either be 0 or 1, not %d\n", flag_value);
                    printf("Try 'set --help' for more information.\n");
                }
            }
        }
        else if (strcasecmp(flag, "--help") == 0)
        {
            printf("Usage: set OPTION...\n"
                   "Set the value of any supported mathematical variable in the engine.\n"
                   "\n"
                   "Mandatory arguments to long options are mandatory for short options too.\n"
                   "-e, --elasticity[=]{0|1}\tset collisions to be inelastic (0), or perfectly elastic (1)\n"
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
    SDL_bool parse_success = SDL_FALSE;
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
            parse_success = SDL_TRUE;
        }
    }
    return parse_success;
}

SDL_bool tryParseFloatOptionArg(char *command_name, char *input_flag, char *short_option, char *long_option, double *p_arg_value)
{
    if (short_option == NULL)
        short_option = long_option;
    SDL_bool parse_success = SDL_FALSE;
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
            parse_success = SDL_TRUE;
        }
    }
    return parse_success;
}

SDL_bool tryParseCharOptionArg(char *command_name, char *input_flag, char *short_option, char *long_option, char *p_arg_value)
{
    if (short_option == NULL)
        short_option = long_option;
    SDL_bool parse_success = SDL_FALSE;
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
            parse_success = SDL_TRUE;
        }
    }
    return parse_success;
}