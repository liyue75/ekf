#include <stdbool.h>
#include "geodesic_grid.h"
#include "matrix3f.h"
#include "fusion_math.h"

//static const int NUM_SUBTRIANGLES = 4;

static const neighbor_umbrella_t neighbor_umbrellas[3] = {
{{9, 8, 7, 12, 14}, 1, 2, 0, 0, 2},
{{1, 2, 4, 5, 3}, 0, 0, 2, 2, 0},
{{16, 15, 13, 18, 17}, 2, 2, 0, 2, 1},
};

static const matrix3f_t inverses[10] = {
    {{-0.309017f,  0.500000f,  0.190983f},
     { 0.000000f,  0.000000f, -0.618034f},
     {-0.309017f, -0.500000f,  0.190983f}},
    {{-0.190983f,  0.309017f, -0.500000f},
     {-0.500000f, -0.190983f,  0.309017f},
     { 0.309017f, -0.500000f, -0.190983f}},
    {{-0.618034f,  0.000000f,  0.000000f},
     { 0.190983f, -0.309017f, -0.500000f},
     { 0.190983f, -0.309017f,  0.500000f}},
    {{-0.500000f,  0.190983f, -0.309017f},
     { 0.000000f, -0.618034f,  0.000000f},
     { 0.500000f,  0.190983f, -0.309017f}},
    {{-0.190983f, -0.309017f, -0.500000f},
     {-0.190983f, -0.309017f,  0.500000f},
     { 0.618034f,  0.000000f,  0.000000f}},
    {{-0.309017f, -0.500000f, -0.190983f},
     { 0.190983f,  0.309017f, -0.500000f},
     { 0.500000f, -0.190983f,  0.309017f}},
    {{ 0.309017f, -0.500000f,  0.190983f},
     { 0.000000f,  0.000000f, -0.618034f},
     { 0.309017f,  0.500000f,  0.190983f}},
    {{ 0.190983f, -0.309017f, -0.500000f},
     { 0.500000f,  0.190983f,  0.309017f},
     {-0.309017f,  0.500000f, -0.190983f}},
    {{ 0.500000f, -0.190983f, -0.309017f},
     { 0.000000f,  0.618034f,  0.000000f},
     {-0.500000f, -0.190983f, -0.309017f}},
    {{ 0.309017f,  0.500000f, -0.190983f},
     {-0.500000f,  0.190983f,  0.309017f},
     {-0.190983f, -0.309017f, -0.500000f}},
};

static const matrix3f_t mid_inverses[10] = {
    {{-0.000000f,  1.000000f, -0.618034f},
     { 0.000000f, -1.000000f, -0.618034f},
     {-0.618034f,  0.000000f,  1.000000f}},
    {{-1.000000f,  0.618034f, -0.000000f},
     {-0.000000f, -1.000000f,  0.618034f},
     { 0.618034f, -0.000000f, -1.000000f}},
    {{-0.618034f, -0.000000f, -1.000000f},
     { 1.000000f, -0.618034f, -0.000000f},
     {-0.618034f,  0.000000f,  1.000000f}},
    {{-1.000000f, -0.618034f, -0.000000f},
     { 1.000000f, -0.618034f,  0.000000f},
     {-0.000000f,  1.000000f, -0.618034f}},
    {{-1.000000f, -0.618034f,  0.000000f},
     { 0.618034f,  0.000000f,  1.000000f},
     { 0.618034f,  0.000000f, -1.000000f}},
    {{-0.618034f, -0.000000f, -1.000000f},
     { 1.000000f,  0.618034f, -0.000000f},
     { 0.000000f, -1.000000f,  0.618034f}},
    {{ 0.000000f, -1.000000f, -0.618034f},
     { 0.000000f,  1.000000f, -0.618034f},
     { 0.618034f, -0.000000f,  1.000000f}},
    {{ 1.000000f, -0.618034f, -0.000000f},
     { 0.000000f,  1.000000f,  0.618034f},
     {-0.618034f,  0.000000f, -1.000000f}},
    {{ 1.000000f,  0.618034f, -0.000000f},
     {-1.000000f,  0.618034f,  0.000000f},
     { 0.000000f, -1.000000f, -0.618034f}},
    {{-0.000000f,  1.000000f,  0.618034f},
     {-1.000000f, -0.618034f, -0.000000f},
     { 0.618034f,  0.000000f, -1.000000f}},
};

static int neighbor_umbrella_component(int idx, int comp_idx)
{
    if (idx < 3) {
        return neighbor_umbrellas[idx].components[comp_idx];
    }
    return (neighbor_umbrellas[idx % 3].components[comp_idx] + 10) % 20;
}

static int from_neighbor_umbrella(int idx,
                                  const vector3f_t *v,
                                  const vector3f_t *u,
                                  bool inclusive)
{
    if (float_is_equal(u->x, u->y)) {
        int comp = neighbor_umbrella_component(idx, 0);
        vector3f_t w = m3f_multi_v(&inverses[comp % 10], v);
        if (comp > 9) {
            w.x = -w.x;
            w.y = -w.y;
            w.z = -w.z;
        }
        float x0 = ((float *)&w)[neighbor_umbrellas[idx % 3].v0_c0];
        if (float_is_zero(x0)) {
            if (!inclusive) {
                return -1;
            }
            return comp;
        } else if (x0 < 0) {
            if (!inclusive) {
                return -1;
            }
            return neighbor_umbrella_component(idx, u->x < u->y ? 3 : 2);
        }
        return comp;
    }
    if (u->y > u->x) {
        int comp  = neighbor_umbrella_component(idx, 1);
        vector3f_t w = m3f_multi_v(&inverses[comp % 10], v);
        if (comp > 9) {
            w.x = -w.x;w.y = -w.y;w.z = -w.z;
        }
        float x1 = ((float *)&w)[neighbor_umbrellas[idx % 3].v1_c1];
        float x2 = ((float *)&w)[neighbor_umbrellas[idx % 3].v2_c1];
        if (float_is_zero(x1)) {
            if (!inclusive) {
                return -1;
            }
            return neighbor_umbrella_component(idx, x1 < 0 ? 2 : 1);
        } else if (x1 < 0) {
            return neighbor_umbrella_component(idx, 2);
        }

        if (float_is_zero(x2)) {
            if (!inclusive) {
                return -1;
            }
            return neighbor_umbrella_component(idx, x2 > 0 ? 1 : 0);
        } else if (x2 < 0) {
            return neighbor_umbrella_component(idx, 0);
        }
        return comp;
    } else {
        int comp = neighbor_umbrella_component(idx, 4);
        vector3f_t w = m3f_multi_v(&inverses[comp % 10], v);
        if (comp > 9) {
            w.x = -w.x; w.y = -w.y; w.z = -w.z;
        }
        float x4 = ((float *)&w)[neighbor_umbrellas[idx % 3].v4_c4];
        float x0 = ((float *)&w)[neighbor_umbrellas[idx % 3].v0_c4];
        if (float_is_zero(x4)) {
            if (!inclusive) {
                return -1;
            }
            return neighbor_umbrella_component(idx, x4 < 0 ? 0 : 4);
        } else if (x4 < 0) {
            return neighbor_umbrella_component(idx, 0);
        }
        if (float_is_zero(x0)) {
            if (!inclusive) {
                return -1;
            }
            return neighbor_umbrella_component(idx, x0 > 0 ? 4 : 3);
        } else if (x0 < 0) {
            return neighbor_umbrella_component(idx, 3);
        }
        return comp;
    }
}

static int triangle_index(const vector3f_t *v, bool inclusive)
{
    vector3f_t w = m3f_multi_v(&inverses[0], v);
    int zero_count = 0;
    int balance = 0;
    int umbrella = -1;
    if (float_is_zero(w.x)) {
        zero_count++;
    } else if (w.x > 0) {
        balance++;
    } else {
        balance--;
    }

    if (float_is_zero(w.y)) {
        zero_count++;
    } else if (w.y > 0) {
        balance++;
    } else {
        balance--;
    }
    if (float_is_zero(w.z)) {
        zero_count++;
    } else if (w.z > 0) {
        balance++;
    } else {
        balance--;
    }
    switch (balance) {
        case 3:
            return 0;
        case -3:
            return 10;
        case 2:
            return inclusive ? 0 : -1;
        case -2:
            return inclusive ? 10 : -1;
        case 1:
            if (zero_count == 2) {
                return inclusive ? 0 : -1;
            }
            if (!float_is_zero(w.x) && w.x < 0) {
                umbrella = 1;
            } else if (!float_is_zero(w.y) && w.y < 0) {
                umbrella = 2;
            } else {
                umbrella = 0;
            }
            break;
        case -1:
            if (zero_count == 2) {
                return inclusive ? 10 : -1;
            }
            if (!float_is_zero(w.x) && w.x > 0) {
                umbrella = 4;
            } else if (!float_is_zero(w.y) && w.y > 0) {
                umbrella = 5;
            } else {
                umbrella = 3;
            }
            w.x = -w.x; w.y = -w.y; w.z = -w.z;
            break;
        case 0:
            if (zero_count == 3) {
                return -1;
            }
            if (!float_is_zero(w.x) && w.x < 0) {
                umbrella = 1;
            } else if (!float_is_zero(w.y) && w.y < 0) {
                umbrella = 2;
            } else {
                umbrella = 0;
            }
            break;
    }
    vector3f_t tmp1 = {w.y, w.z, -w.x};
    vector3f_t tmp2 = {w.z, w.x, -w.y};
    switch (umbrella % 3) {
        case 0:
            w.z = -w.z;
            break;
        case 1:
            w = tmp1;
            break;
        case 2:
            w = tmp2;
            break;
    }
    return from_neighbor_umbrella(umbrella, v, &w, inclusive);
}

static int subtriangle_index(const unsigned int triangle_index,
                             const vector3f_t *v,
                             bool inclusive)
{
    vector3f_t w = m3f_multi_v(&mid_inverses[triangle_index % 10], v);
    if (triangle_index > 9) {
        w.x = -w.x; w.y = -w.y; w.z = -w.z;
    }
    if ((float_is_zero(w.x) || float_is_zero(w.y) || float_is_zero(w.z)) && !inclusive) {
        return -1;
    }
    if (!float_is_zero(w.x) && w.x < 0) {
        return 3;
    }
    if (!float_is_zero(w.y) && w.y < 0) {
        return 1;
    }
    if (!float_is_zero(w.z) && w.z < 0) {
        return 2;
    }
    return 0;
}

int geodesic_grid_section(const vector3f_t *v, bool inclusive)
{
    int i = triangle_index(v, inclusive);
    if (i < 0) {
        return -1;
    }
    int j = subtriangle_index(i, v, inclusive);
    if (j < 0) {
        return -1;
    }
    return 4 * i + j;
}
