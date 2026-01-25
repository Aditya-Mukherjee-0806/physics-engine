#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>

#define WINDOW_WIDTH 1280
#define WINDOW_HEIGHT 720
#define FRAMES_PER_SEC 30
#define DEFAULT_ARR_CAPACITY 8
#define PIXELS_PER_METER 1500
#define MIN_RADIUS 15
#define MAX_RADIUS 30
#define DENSITY 500
#define LOG_FILE "log.txt"
#define LOG_INTERVAL_SECS 1
#define INPUT_BUFFER_SIZE 128

#define RGB_RED 255, 0, 0
#define RGB_GREEN 0, 255, 0
#define RGB_BLUE 0, 0, 255
#define RGB_YELLOW 255, 255, 0
#define RGB_CYAN 0, 255, 255
#define RGB_MAGENTA 255, 0, 255

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
    Uint32 color;
    double radius;
    PHYS_BODY phys_comp;
} CIRCLE_OBJ;

const double π = 3.141592653589793;
const double G = 6.6743E-11 * PIXELS_PER_METER * PIXELS_PER_METER * PIXELS_PER_METER;
const double dt = 1.0 / FRAMES_PER_SEC;

FILE *log_file;
SDL_Surface *surface;
CIRCLE_OBJ *circle_object_arr;
int arr_cap = DEFAULT_ARR_CAPACITY, arr_size = 0;
SDL_mutex *shared_data_mutex;
SDL_bool is_simulation_paused = SDL_FALSE;

SDL_bool isPointInsideCircle(VECTOR_2D point, CIRCLE_OBJ circle_obj);
void logArrInfo();
void logInfoOf(CIRCLE_OBJ circle_obj);
void createNewCircleObj(Uint32 color, double radius, double mass, VECTOR_2D pos, VECTOR_2D vel);
void runSimulation(Uint8 elasticity);
void sanitiseObjectArray();
void simulateForces(Uint8 elasticity);
void simulateGravitationalForce(Uint8 elasticity);
void handleCollision(CIRCLE_OBJ *c1, CIRCLE_OBJ *c2, Uint8 elasticity);
void updatePositions();
void FillCircle(CIRCLE_OBJ circle_obj);
int processUserInput(void *data);
void handleCreateCommand(char *input);
void handleClearCommand(char *input);
void handleSetCommand(char *input);
void handlePauseCommand(char *input);
void handleResumeCommand(char *input);
CIRCLE_OBJ *findCircleById(int id);

int main()
{
    srand(time(NULL));
    log_file = fopen(LOG_FILE, "w");
    SDL_Init(SDL_INIT_EVERYTHING);
    SDL_Window *window = SDL_CreateWindow("Physics Engine", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WINDOW_WIDTH, WINDOW_HEIGHT, 0);
    surface = SDL_GetWindowSurface(window);

    circle_object_arr = (CIRCLE_OBJ *)calloc(arr_cap, sizeof(CIRCLE_OBJ));

    const Uint32 colors[] = {
        SDL_MapRGB(surface->format, RGB_RED),
        SDL_MapRGB(surface->format, RGB_GREEN),
        SDL_MapRGB(surface->format, RGB_BLUE),
        SDL_MapRGB(surface->format, RGB_YELLOW),
        SDL_MapRGB(surface->format, RGB_CYAN),
        SDL_MapRGB(surface->format, RGB_MAGENTA),
    };
    const int num_colors = sizeof(colors) / sizeof(colors[0]);

    shared_data_mutex = SDL_CreateMutex();
    SDL_Thread *input_thread = SDL_CreateThread(processUserInput, "input thread", (void *)colors);

    // for (int i = 0; i < arr_cap; i++)
    // {
    //     int radius = rand() % (MAX_RADIUS - MIN_RADIUS) + MIN_RADIUS;
    //     Uint32 color = colors[i % num_colors];
    //     VECTOR_2D pos = {rand() % WINDOW_WIDTH, rand() % WINDOW_HEIGHT};
    //     VECTOR_2D vel = {
    //         (double)rand() / RAND_MAX * (rand() % 2 ? 1 : -1),
    //         (double)rand() / RAND_MAX * (rand() % 2 ? 1 : -1),
    //     };
    //     createNewCircleObj(color, radius, π * radius * radius * DENSITY, pos, vel);
    // }

    // Central massive body (like the Sun)
    VECTOR_2D pos1 = {WINDOW_WIDTH / 2.0, WINDOW_HEIGHT / 2.0};
    VECTOR_2D vel1 = {0, 0};
    double radius1 = 40;
    double mass1 = 100000000;
    createNewCircleObj(SDL_MapRGB(surface->format, RGB_YELLOW), radius1, mass1, pos1, vel1);

    // Smaller orbiting body (like a planet)
    double distance = 300; // distance from center
    VECTOR_2D pos2 = {pos1.x + distance, pos1.y};
    double radius2 = 20; // smaller radius
    double mass2 = 100000;

    // Circular orbit velocity perpendicular to radius
    double v = SDL_sqrt(G * mass1 / distance);
    VECTOR_2D vel2 = {0, -v}; // moving upwards for clockwise orbit
    createNewCircleObj(SDL_MapRGB(surface->format, RGB_CYAN), radius2, mass2, pos2, vel2);

    Uint8 elasticity = 1;
    int frames = 0;
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
                    for (int i = 0; i < arr_size; i++)
                    {
                        if (isPointInsideCircle(point, circle_object_arr[i]))
                        {
                            printf("ID: %d\n", circle_object_arr[i].id);
                            fflush(stdout);
                            break;
                        }
                    }
                    break;
                }
                break;
            }
        }
        if(is_simulation_paused)
        {
            SDL_Delay(dt);
            continue;
        }
        SDL_FillRect(surface, NULL, 0);
        runSimulation(elasticity);
        SDL_UpdateWindowSurface(window);
        if (++frames % (LOG_INTERVAL_SECS * FRAMES_PER_SEC) == 0)
            logArrInfo();
        Uint64 end = SDL_GetPerformanceCounter();
        double frame_time = (double)(end - start) / SDL_GetPerformanceFrequency();
        frame_time_sum += frame_time;
        if (frame_time < min_frame_time)
            min_frame_time = frame_time;
        if (frame_time > max_frame_time)
            max_frame_time = frame_time;
        if (frame_time < dt)
            SDL_Delay((dt - frame_time) * 1000);
    }

    printf("Number of frames: %d\n", frames);
    printf("Time passed: %lf\n", (double)frames / FRAMES_PER_SEC);
    printf("Avg. Frame Time: %lf\n", frame_time_sum / frames);
    printf("Min. Frame Time: %lf\n", min_frame_time);
    printf("Max. Frame Time: %lf\n", max_frame_time);

    SDL_FreeSurface(surface);
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
    // printf("Log Entry: #%d\n", log_count);
    fprintf(log_file, "ENTRY: #%d\n", log_count);
    for (int i = 0; i < arr_size; i++)
    {
        // printf("Circle %d:\n", circles[i].id);
        fprintf(log_file, "Circle %d:\n", circle_object_arr[i].id);
        logInfoOf(circle_object_arr[i]);
    }
    log_count++;
}

void logInfoOf(CIRCLE_OBJ circle_obj)
{
    if (!circle_obj.alive)
    {
        // printf("is Null.\n");
        fprintf(log_file, "is Null.\n");
        return;
    }

    // printf("Radius = %lf\n", circle_obj.radius);
    // printf("Mass = %lf\n", circle_obj.phys_comp.mass);
    // printf("Position = (%lf, %lf)\n", circle_obj.phys_comp.pos.x, circle_obj.phys_comp.pos.y);
    // printf("Velocity = (%lf, %lf)\n", circle_obj.phys_comp.vel.x, circle_obj.phys_comp.vel.y);

    fprintf(log_file, "Radius = %lf\n", circle_obj.radius);
    fprintf(log_file, "Mass = %lf\n", circle_obj.phys_comp.mass);
    fprintf(log_file, "Position = (%lf, %lf)\n", circle_obj.phys_comp.pos.x, circle_obj.phys_comp.pos.y);
    fprintf(log_file, "Velocity = (%lf, %lf)\n", circle_obj.phys_comp.vel.x, circle_obj.phys_comp.vel.y);
}

void createNewCircleObj(Uint32 color, double radius, double mass, VECTOR_2D pos, VECTOR_2D vel)
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
            // memset(circle_object_arr + arr_size, 0, (arr_cap - arr_size) * sizeof(CIRCLE_OBJ));
        }
        else
            fprintf(stderr, "REALLOCATION FAILED in %s\n", __func__);
    }
    circle_object_arr[arr_size++] = circle_obj;

    SDL_UnlockMutex(shared_data_mutex);
}

void runSimulation(Uint8 elasticity)
{
    SDL_LockMutex(shared_data_mutex);

    sanitiseObjectArray();
    simulateForces(elasticity);
    updatePositions();
    for (int i = 0; i < arr_size; i++)
        FillCircle(circle_object_arr[i]);

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
            // memset(circle_object_arr + arr_size, 0, (arr_cap - arr_size) * sizeof(CIRCLE_OBJ));
        }
        else
            fprintf(stderr, "REALLOCATION FAILED in %s\n", __func__);
    }
}

void simulateForces(Uint8 elasticity)
{
    simulateGravitationalForce(elasticity);
}

void simulateGravitationalForce(Uint8 elasticity)
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
            VECTOR_2D u1 = circle_object_arr[i].phys_comp.vel;
            VECTOR_2D u2 = circle_object_arr[j].phys_comp.vel;
            VECTOR_2D pos1 = circle_object_arr[i].phys_comp.pos;
            VECTOR_2D pos2 = circle_object_arr[j].phys_comp.pos;
            double dist = SDL_sqrt(SDL_pow(pos1.x - pos2.x, 2) + SDL_pow(pos1.y - pos2.y, 2));
            if (dist < circle_object_arr[i].radius + circle_object_arr[j].radius)
            {
                handleCollision(circle_object_arr + i, circle_object_arr + j, elasticity);

                if (elasticity == 0)
                    // stop calculating with circles[i] in current frame in case of merge
                    break;
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

void handleCollision(CIRCLE_OBJ *c1, CIRCLE_OBJ *c2, Uint8 elasticity)
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

void FillCircle(CIRCLE_OBJ circle_obj)
{
    int x = circle_obj.phys_comp.pos.x;
    int y = circle_obj.phys_comp.pos.y;
    int r = circle_obj.radius;
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
            {
                SDL_Rect pixel = {i, j, 1, 1};
                SDL_FillRect(surface, &pixel, circle_obj.color);
            }
        }
    }
}

int SDLCALL processUserInput(void *data)
{
    const Uint32 *colors = (Uint32 *)(data);
    printf("Supported Commands: create, clear, set, pause, resume\n");
    printf("Reading input...\n");
    while (1)
    {
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
            printf("%s is not a supported command\n", command);
        // char *command = strtok(input, " ");
        // if (strcasecmp(command, "create") != 0)
        // {
        //     printf("Usage: create <color> <radius> <mass> <posx> <posy> <velx> <vely>\n");
        //     continue;
        // }

        // char *color_tok = strtok(NULL, " ");
        // int color_index = -1;
        // switch (*color_tok)
        // {
        // case 'r':
        //     color_index = 0;
        //     break;
        // case 'g':
        //     color_index = 1;
        //     break;
        // case 'b':
        //     color_index = 2;
        //     break;
        // case 'y':
        //     color_index = 3;
        //     break;
        // case 'c':
        //     color_index = 4;
        //     break;
        // case 'm':
        //     color_index = 5;
        //     break;
        // }

        // double radius = atof(strtok(NULL, " "));
        // double mass = atof(strtok(NULL, " "));
        // VECTOR_2D pos = {atof(strtok(NULL, " ")), atof(strtok(NULL, " "))};
        // VECTOR_2D vel = {atof(strtok(NULL, " ")), atof(strtok(NULL, " "))};

        // createNewCircleObj(colors[color_index], radius, mass, pos, vel);
    }
}

void handleCreateCommand(char *input)
{
}

void handleClearCommand(char *input)
{
    SDL_LockMutex(shared_data_mutex);
    char *delims = " \t\r\n";
    strtok(input, delims); // skip the command
    char *flag = strtok(NULL, delims);
    if (flag == NULL || strcasecmp(flag, "--all") == 0)
    {
        // clear the object array before the next frame
        arr_size = 0;
        arr_cap = DEFAULT_ARR_CAPACITY;
    }
    else if (strcasecmp(flag, "--id") == 0)
    {
        char *id_str = strtok(NULL, delims);
        if (id_str == NULL)
            printf("ID not provided\n");
        else
        {
            int id;
            if (sscanf(id_str, "%d", &id) != 1)
                printf("ID field is invalid\n");
            else
            {
                CIRCLE_OBJ *circleFound = findCircleById(id);
                if (circleFound != NULL)
                    circleFound->alive = SDL_FALSE;
                else
                    printf("Circle with ID: %d does not exist\n", id);
            }
        }
    }
    else if(strcasecmp(flag, "--help") == 0)
    {
        printf("Usage: clear [OPTION]\n");
        printf("Clears all objects or optionally, a single one specified by its id\n");
        printf("\n");
        printf("\t--id NUM\tclear only the object whose id is NUM\n");
        printf("\t--help\t\tdisplay this help and exit\n");
    }
    else
        printf("Invalid Flag: %s\n", flag);

    SDL_UnlockMutex(shared_data_mutex);
}

void handleSetCommand(char *input)
{
}

void handlePauseCommand(char *input)
{
    char *delims = " \t\r\n";
    strtok(input, delims); //skip the command
    char *flag = strtok(NULL, delims);
    if(flag == NULL)
    {
        is_simulation_paused = SDL_TRUE;
    }
    else if(strcasecmp(flag, "--help") == 0)
    {
        printf("Usage: pause [OPTION]\n");
        printf("Pause the simulation if not already paused, otherwise do nothing\n");
        printf("\n");
        printf("\t--help\tdisplay this help and exit\n");
    }
    else
    {
        printf("Invalid Flag: %s\n", flag);
    }
}

void handleResumeCommand(char *input)
{
    char *delims = " \t\r\n";
    strtok(input, delims); //skip the command
    char *flag = strtok(NULL, delims);
    if(flag == NULL)
    {
        is_simulation_paused = SDL_FALSE;
    }
    else if(strcasecmp(flag, "--help") == 0)
    {
        printf("Usage: resume [OPTION]\n");
        printf("Resume the simulation if paused, otherwise do nothing\n");
        printf("\n");
        printf("\t--help\tdisplay this help and exit\n");
    }
    else
    {
        printf("Invalid Flag: %s\n", flag);
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