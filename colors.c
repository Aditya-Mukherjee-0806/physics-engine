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

/**
 * @brief Get a random number between low (inclusive) and high (exclusive)
 *
 * @param low the lower limit (inclusive)
 * @param high the higher limit (exclusive)
 * @return the random number generated in the given range
 */
int getNumInRange(int low, int high)
{
    return low + rand() % (high - low);
}

/**
 * @brief shuffle an unsigned char array based on the Fisher-Yates algorithm
 *
 * @param arr the array to shuffle
 * @param n the size of the array
 */
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
    bands[0] = getNumInRange(0, 64);
    bands[1] = getNumInRange(64, 128);
    bands[2] = getNumInRange(128, 256);

    UCHAR_FisherYatesShuffle(bands, 3);

    RGB24 color;
    color.r = bands[0];
    color.g = bands[1];
    color.b = bands[2];
    return color;
}