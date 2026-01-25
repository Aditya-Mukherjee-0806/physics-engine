CC = gcc
CFLAGS = -g -Wall -Wextra
LDFLAGS = `sdl2-config --cflags --libs`

# make will create/update a.out by default
all: a.out

# make run will create/update a.out if necessary then execute a.out
run: a.out
	./a.out

a.out: physics_engine.c
	$(CC) $(CFLAGS) $< -o $@ $(LDFLAGS)
