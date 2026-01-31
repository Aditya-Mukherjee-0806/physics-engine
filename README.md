# Physics Engine

A program that simulates physics between objects.

## Overview

This a physics engine made entirely in C using the SDL2 library. Currently, it can only simulate gravitational force between objects. In case of a collision the objects either bounce off or merge based on the elasticity of the collision. The user can provide input from the terminal while the application is running to perform certain operations such as:

- Create a new object
- Clear object(s) from the simulation
- Set values of certain mathematical constants
- Pause/Resume the simulation

To learn more about a specific command, type: ```<command> --help```

## Getting Started

### Dependencies

The project uses the SDL2 library to create and draw objects to the simulation window. You can install SDL2 by following the steps from the official [SDL2 Installation Guide](https://wiki.libsdl.org/SDL2/Installation).

### Installation

Clone the repository.

```bash
git clone https://github.com/Aditya-Mukherjee-0806/physics-engine
```

### Execution

- **(Recommended)** To build and run the executable, type ```make run``` in the terminal from the project directory
- To simply build the executable without running it, type ```make```
- To clean the object files after building, type ```make clean ```

<!--
TODO:
## Usage
An example showing a planet-like object orbiting a star-like object.

-->

## Author

Aditya Mukherjee - <adityamukherjee082006@gmail.com>

## Support

<a href="https://www.buymeacoffee.com/ShadowDTR" target="_blank"><img src="https://cdn.buymeacoffee.com/buttons/v2/default-yellow.png" alt="Buy Me A Coffee" style="height: 60px !important;width: 217px !important;" ></a>

## Known Issues

Sometimes if elasticity is set to 1, objects enter inside each other during collision, in which case, they may dart off the screen if their centers overlap.
