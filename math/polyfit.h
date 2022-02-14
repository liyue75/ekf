#ifndef POLYFIT_H_
#define POLYFIT_H_

#include <stdint.h>
#include <stdbool.h>
#include "vector3f.h"

#define POLY_ORDER 4

typedef struct polyfit {
    double mat[POLY_ORDER][POLY_ORDER];
    vector3f_t vec[POLY_ORDER];
} polyfit_t;

void update(double x, vector3f_t *y);
bool get_polynomial(vector3f_t *res);

#endif // POLYFIT_H_
