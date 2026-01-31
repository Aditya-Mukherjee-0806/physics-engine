#ifndef COLORS_H
#define COLORS_H

typedef struct
{
    unsigned char r, g, b;
} RGB24;

extern const RGB24 RGB_BLACK;
extern const RGB24 RGB_RED;
extern const RGB24 RGB_GREEN;
extern const RGB24 RGB_BLUE;
extern const RGB24 RGB_YELLOW;
extern const RGB24 RGB_CYAN;
extern const RGB24 RGB_MAGENTA;
extern const RGB24 RGB_WHITE;

RGB24 generateVividColor();

#endif