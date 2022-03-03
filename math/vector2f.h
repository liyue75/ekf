#ifndef VECTOR2F_H_
#define VECTOR2F_H_

typedef struct vector2f {
    float x;
    float y;
} vector2f_t;

vector2f_t v2f_normalized(const vector2f_t *v);

#endif // VECTOR2F_H_
