#include "colors.h"
#include <time.h>
#include <stdlib.h>

const RGB24 RGB_BLACK = {0, 0, 0};
const RGB24 RGB_RED = {255, 0, 0};
const RGB24 RGB_GREEN = {0, 255, 0};
const RGB24 RGB_BLUE = {0, 0, 255};
const RGB24 RGB_YELLOW = {255, 255, 0};
const RGB24 RGB_CYAN = {0, 255, 255};
const RGB24 RGB_MAGENTA = {255, 0, 255};
const RGB24 RGB_WHITE = {255, 255, 255};

int getRandNumInRange(int low, int high)
{
    return low + rand() % (high - low);
}

void UCHAR_FisherYatesShuffle(unsigned char *arr, int n)
{
    for (int i = n - 1; i > 0; i--)
    {
        int j = rand() % (i + 1);
        unsigned char temp = arr[i];
        arr[i] = arr[j];
        arr[j] = temp;
    }
}

RGB24 generateVividColor()
{
    unsigned char bands[3];
    bands[0] = getRandNumInRange(0, 64);
    bands[1] = getRandNumInRange(64, 128);
    bands[2] = getRandNumInRange(128, 256);

    UCHAR_FisherYatesShuffle(bands, 3);

    RGB24 color;
    color.r = bands[0];
    color.g = bands[1];
    color.b = bands[2];
    return color;
}

RGB24 mixTwoColors(RGB24 color1, RGB24 color2)
{
    return (RGB24){
        .r = (color1.r + color2.r) / 2,
        .g = (color1.g + color2.g) / 2,
        .b = (color1.b + color2.b) / 2,
    };
}