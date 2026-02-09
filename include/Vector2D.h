#ifndef VECTOR2D_H
#define VECTOR2D_H

typedef struct
{
    double x, y;
} VECTOR_2D;

double Vector2D_Magnitude(VECTOR_2D);
VECTOR_2D Vector2D_Normalised(VECTOR_2D);
VECTOR_2D Vector2D_Sum(VECTOR_2D, VECTOR_2D);
VECTOR_2D Vector2D_Difference(VECTOR_2D, VECTOR_2D);
VECTOR_2D Vector2D_ScalarProduct(VECTOR_2D, double c);
double Vector2D_DotProduct(VECTOR_2D, VECTOR_2D);

#endif