#include "vector2f.h"
#include "fusion_math.h"

vector2f_t v2f_normalized(const vector2f_t *v)
{
    float tmp = norm_2f(v->x, v->y);
    vector2f_t tmp2 = {v->x / tmp, v->y / tmp};
    return tmp2;
}
