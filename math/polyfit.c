#include "polyfit.h"
#include "vector3f.h"

#define POLY_ORDER 4

void update(__attribute__((unused))double x,
            __attribute__((unused))vector3f_t *y)
{
/*     double temp = 1; */
/*     for (int8_t i = 2 * (POLY_ORDER - 1); i >= 0; i--) { */
/*         int8_t k = (i < POLY_ORDER)?0:i - POLY_ORDER + 1; */
/*         for (int8_t j = i - k; j >= k; j--) { */
/*             mat[j][i - j] += temp; */
/*         } */
/*         temp *= x; */
/*     } */
/*     temp = 1; */
/*     for (int8_t i = POLY_ORDER - 1; i >= 0; i--) { */
/*         vec[i] += y * temp; */
/*         temp *= x; */
/*     } */
}

bool get_polynomial(__attribute__((unused))vector3f_t *res)
{
    //double *inv_mat = malloc(sizeof(double) * POLY_ORDER * POLY_ORDER);
    return true;
}
