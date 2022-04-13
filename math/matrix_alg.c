#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "matrix_alg.h"
#include "fusion_math.h"

static float* matrix_multiply(const float *A, const float *B, uint16_t n)
{
    float *ret = malloc(n*n*sizeof(float));
    memset(ret, 0, n*n*sizeof(float));
    for (uint16_t i = 0; i < n; i++) {
        for (uint16_t j = 0; j < n; j++) {
            for (uint16_t k = 0; k < n; k++) {
                ret[i*n + j] += A[i*n + k] * B[k*n +j];
            }
        }
    }
    return ret;
}

static inline void swap(float *a, float *b)
{
    float c;
    c = *a;
    *a = *b;
    *b = c;
}

static void mat_pivot(const float *A, float*pivot, uint16_t n)
{
    for (uint16_t i = 0; i < n; i++) {
        for (uint16_t j = 0; j < n; j++) {
            pivot[i*n+j] = (float)(i == j);
        }
    }
    for (uint16_t i = 0; i < n; i++) {
        uint16_t max_j = i;
        for (uint16_t j=i; j < n; j++) {
            if (fabsf(A[j*n + i]) > fabsf(A[max_j*n + i])) {
                max_j = j;
            }
        }
        if (max_j != i) {
            for (uint16_t k = 0; k < n; k++) {
                swap(&pivot[i*n + k], &pivot[max_j*n + k]);
            }
        }
    }
}

static void mat_forward_sub(const float *L, float *out, uint16_t n)
{
// Forward substitution solve LY = I
    for(int i = 0; i < n; i++) {
        out[i*n + i] = 1/L[i*n + i];
        for (int j = i+1; j < n; j++) {
            for (int k = i; k < j; k++) {
                out[j*n + i] -= L[j*n + k] * out[k*n + i];
            }
            out[j*n + i] /= L[j*n + j];
        }
    }
}

static void mat_back_sub(const float *U, float *out, uint16_t n)
{
    // Backward Substitution solve UY = I
    for(int i = n-1; i >= 0; i--) {
        out[i*n + i] = 1/U[i*n + i];
        for (int j = i - 1; j >= 0; j--) {
            for (int k = i; k > j; k--) {
                out[j*n + i] -= U[j*n + k] * out[k*n + i];
            }
            out[j*n + i] /= U[j*n + j];
        }
    }
}

static void mat_LU_decompose(const float* A, float* L, float* U, float *P, uint16_t n)
{
    memset(L,0,n*n*sizeof(float));
    memset(U,0,n*n*sizeof(float));
    memset(P,0,n*n*sizeof(float));
    mat_pivot(A,P,n);

    float *APrime = matrix_multiply(P,A,n);
    for(uint16_t i = 0; i < n; i++) {
        L[i*n + i] = 1;
    }
    for(uint16_t i = 0; i < n; i++) {
        for(uint16_t j = 0; j < n; j++) {
            if(j <= i) {
                U[j*n + i] = APrime[j*n + i];
                for(uint16_t k = 0; k < j; k++) {
                    U[j*n + i] -= L[j*n + k] * U[k*n + i];
                }
            }
            if(j >= i) {
                L[j*n + i] = APrime[j*n + i];
                for(uint16_t k = 0; k < i; k++) {
                    L[j*n + i] -= L[j*n + k] * U[k*n + i];
                }
                L[j*n + i] /= U[i*n + i];
            }
        }
    }
    free(APrime);
}

static bool mat_inverseN(const float *A, float *inv, uint16_t n)
{
    float *L, *U, *P;
    bool ret = true;
    L = malloc(n * n * sizeof(float));
    U = malloc(n * n * sizeof(float));
    P = malloc(n * n * sizeof(float));
    mat_LU_decompose(A,L,U,P,n);

    float *L_inv = malloc(n*n*sizeof(float));
    float *U_inv = malloc(n*n*sizeof(float));

    memset(L_inv,0,n*n*sizeof(float));
    mat_forward_sub(L,L_inv,n);

    memset(U_inv,0,n*n*sizeof(float));
    mat_back_sub(U,U_inv,n);

    // decomposed matrices no longer required
    free(L);
    free(U);

    float *inv_unpivoted = matrix_multiply(U_inv,L_inv,n);
    float *inv_pivoted = matrix_multiply(inv_unpivoted, P, n);

    //check sanity of results
    for(uint16_t i = 0; i < n; i++) {
        for(uint16_t j = 0; j < n; j++) {
            if(isnan(inv_pivoted[i*n+j]) || isinf(inv_pivoted[i*n+j])){
                ret = false;
            }
        }
    }
    memcpy(inv,inv_pivoted,n*n*sizeof(float));

    //free memory
    free(inv_pivoted);
    free(inv_unpivoted);
    free(P);
    free(U_inv);
    free(L_inv);
    return ret;
}

static bool inverse3x3(const float m[], float invOut[])
{
    float inv[9];
    // computes the inverse of a matrix m
    float  det = m[0] * (m[4] * m[8] - m[7] * m[5]) -
    m[1] * (m[3] * m[8] - m[5] * m[6]) +
    m[2] * (m[3] * m[7] - m[4] * m[6]);
    if (float_is_zero(det) || isinf(det)) {
        return false;
    }

    float invdet = 1 / det;

    inv[0] = (m[4] * m[8] - m[7] * m[5]) * invdet;
    inv[1] = (m[2] * m[7] - m[1] * m[8]) * invdet;
    inv[2] = (m[1] * m[5] - m[2] * m[4]) * invdet;
    inv[3] = (m[5] * m[6] - m[3] * m[8]) * invdet;
    inv[4] = (m[0] * m[8] - m[2] * m[6]) * invdet;
    inv[5] = (m[3] * m[2] - m[0] * m[5]) * invdet;
    inv[6] = (m[3] * m[7] - m[6] * m[4]) * invdet;
    inv[7] = (m[6] * m[1] - m[0] * m[7]) * invdet;
    inv[8] = (m[0] * m[4] - m[3] * m[1]) * invdet;

    for(uint16_t i = 0; i < 9; i++){
        invOut[i] = inv[i];
    }

    return true;
}

static bool inverse4x4(const float m[], float invOut[])
{
    float inv[16], det;
    uint16_t i;
    inv[0] = m[5]  * m[10] * m[15] -
    m[5]  * m[11] * m[14] -
    m[9]  * m[6]  * m[15] +
    m[9]  * m[7]  * m[14] +
    m[13] * m[6]  * m[11] -
    m[13] * m[7]  * m[10];

    inv[4] = -m[4]  * m[10] * m[15] +
    m[4]  * m[11] * m[14] +
    m[8]  * m[6]  * m[15] -
    m[8]  * m[7]  * m[14] -
    m[12] * m[6]  * m[11] +
    m[12] * m[7]  * m[10];

    inv[8] = m[4]  * m[9] * m[15] -
    m[4]  * m[11] * m[13] -
    m[8]  * m[5] * m[15] +
    m[8]  * m[7] * m[13] +
    m[12] * m[5] * m[11] -
    m[12] * m[7] * m[9];

    inv[12] = -m[4]  * m[9] * m[14] +
    m[4]  * m[10] * m[13] +
    m[8]  * m[5] * m[14] -
    m[8]  * m[6] * m[13] -
    m[12] * m[5] * m[10] +
    m[12] * m[6] * m[9];

    inv[1] = -m[1]  * m[10] * m[15] +
    m[1]  * m[11] * m[14] +
    m[9]  * m[2] * m[15] -
    m[9]  * m[3] * m[14] -
    m[13] * m[2] * m[11] +
    m[13] * m[3] * m[10];

    inv[5] = m[0]  * m[10] * m[15] -
    m[0]  * m[11] * m[14] -
    m[8]  * m[2] * m[15] +
    m[8]  * m[3] * m[14] +
    m[12] * m[2] * m[11] -
    m[12] * m[3] * m[10];

    inv[9] = -m[0]  * m[9] * m[15] +
    m[0]  * m[11] * m[13] +
    m[8]  * m[1] * m[15] -
    m[8]  * m[3] * m[13] -
    m[12] * m[1] * m[11] +
    m[12] * m[3] * m[9];

    inv[13] = m[0]  * m[9] * m[14] -
    m[0]  * m[10] * m[13] -
    m[8]  * m[1] * m[14] +
    m[8]  * m[2] * m[13] +
    m[12] * m[1] * m[10] -
    m[12] * m[2] * m[9];

    inv[2] = m[1]  * m[6] * m[15] -
    m[1]  * m[7] * m[14] -
    m[5]  * m[2] * m[15] +
    m[5]  * m[3] * m[14] +
    m[13] * m[2] * m[7] -
    m[13] * m[3] * m[6];

    inv[6] = -m[0]  * m[6] * m[15] +
    m[0]  * m[7] * m[14] +
    m[4]  * m[2] * m[15] -
    m[4]  * m[3] * m[14] -
    m[12] * m[2] * m[7] +
    m[12] * m[3] * m[6];

    inv[10] = m[0]  * m[5] * m[15] -
    m[0]  * m[7] * m[13] -
    m[4]  * m[1] * m[15] +
    m[4]  * m[3] * m[13] +
    m[12] * m[1] * m[7] -
    m[12] * m[3] * m[5];

    inv[14] = -m[0]  * m[5] * m[14] +
    m[0]  * m[6] * m[13] +
    m[4]  * m[1] * m[14] -
    m[4]  * m[2] * m[13] -
    m[12] * m[1] * m[6] +
    m[12] * m[2] * m[5];

    inv[3] = -m[1] * m[6] * m[11] +
    m[1] * m[7] * m[10] +
    m[5] * m[2] * m[11] -
    m[5] * m[3] * m[10] -
    m[9] * m[2] * m[7] +
    m[9] * m[3] * m[6];

    inv[7] = m[0] * m[6] * m[11] -
    m[0] * m[7] * m[10] -
    m[4] * m[2] * m[11] +
    m[4] * m[3] * m[10] +
    m[8] * m[2] * m[7] -
    m[8] * m[3] * m[6];

    inv[11] = -m[0] * m[5] * m[11] +
    m[0] * m[7] * m[9] +
    m[4] * m[1] * m[11] -
    m[4] * m[3] * m[9] -
    m[8] * m[1] * m[7] +
    m[8] * m[3] * m[5];

    inv[15] = m[0] * m[5] * m[10] -
    m[0] * m[6] * m[9] -
    m[4] * m[1] * m[10] +
    m[4] * m[2] * m[9] +
    m[8] * m[1] * m[6] -
    m[8] * m[2] * m[5];

    det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

    if (float_is_zero(det) || isinf(det)){
        return false;
    }

    det = 1.0f / det;

    for (i = 0; i < 16; i++)
        invOut[i] = inv[i] * det;

    return true;
}

bool mat_inverse(const float x[], float y[], uint16_t dim)
{
    switch(dim){
    case 3: return inverse3x3(x,y);
    case 4: return inverse4x4(x,y);
    default: return mat_inverseN(x,y,dim);
    }
}

void mat_mul(const float *A, const float *B, float *C, uint16_t n)
{
    memset(C, 0, sizeof(float)*n*n);
    for(uint16_t i = 0; i < n; i++) {
        for(uint16_t j = 0; j < n; j++) {
            for(uint16_t k = 0;k < n; k++) {
                C[i*n + j] += A[i*n + k] * B[k*n + j];
            }
        }
    }
}

void mat_identity(float *A, uint16_t n)
{
    memset(A, 0, sizeof(float)*n*n);
    for (uint16_t i=0; i<n; i++) {
        A[i*n+i] = 1;
    }
}
