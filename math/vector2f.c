#include "vector2f.h"
#include "fusion_math.h"

vector2f_t v2f_normalized(const vector2f_t *v)
{
    float tmp = norm_2f(v->x, v->y);
    vector2f_t tmp2 = {v->x / tmp, v->y / tmp};
    return tmp2;
}

float v2f_length(const vector2f_t *v)
{
    return norm_2f(v->x, v->y);
}

float v2f_cross_product(const vector2f_t *v1, const vector2f_t *v2)
{
    return v1->x * v2->y - v1->y * v2->x;
}

void v2f_zero(vector2f_t *v)
{
    v->x = v->y = 0;
}
