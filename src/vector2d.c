#include "vector2d.h"
#include <math.h>

double Vector2D_Magnitude(VECTOR_2D v)
{
    return sqrt(v.x * v.x + v.y * v.y);
}

VECTOR_2D Vector2D_Normalised(VECTOR_2D v)
{
    double mag = Vector2D_Magnitude(v);
    if (mag == 0)
        return (VECTOR_2D){0, 0};
    return (VECTOR_2D){
        .x = v.x / mag,
        .y = v.y / mag,
    };
}

VECTOR_2D Vector2D_Sum(VECTOR_2D v1, VECTOR_2D v2)
{
    return (VECTOR_2D){
        .x = v1.x + v2.x,
        .y = v1.y + v2.y,
    };
}

VECTOR_2D Vector2D_Difference(VECTOR_2D v1, VECTOR_2D v2)
{
    return (VECTOR_2D){
        .x = v1.x - v2.x,
        .y = v1.y - v2.y,
    };
}

VECTOR_2D Vector2D_ScalarProduct(VECTOR_2D v, double c)
{
    return (VECTOR_2D){
        .x = v.x * c,
        .y = v.y * c,
    };
}

double Vector2D_DotProduct(VECTOR_2D v1, VECTOR_2D v2)
{
    return v1.x * v2.x + v1.y * v2.y;
}
