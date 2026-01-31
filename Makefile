CC = gcc
CFLAGS = -g -Wall -Wextra $(SDL_CFLAGS)
SDL_CFLAGS = `sdl2-config --cflags`
SDL_LIBS = `sdl2-config --libs`
SRC = physics_engine.c colors.c
OBJ = $(SRC:.c=.o)
TARGET = a.out

.PHONY: all run clean

# make will create/update a.out by default
all: $(TARGET)

# make run will create/update a.out if necessary then execute a.out
run: $(TARGET)
	./$(TARGET)

# create/update a.out from obj files if they are modified
$(TARGET): $(OBJ)
	$(CC) $^ -o $@ $(SDL_LIBS)

# create .o files from .c files with the same filename
%.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

# remove the .o files
clean:
	rm -f $(OBJ)
