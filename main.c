#include <stdio.h>
#include "xtimer.h"
#include "timex.h"
#include "periph/spi.h"
#include "thread.h"

#include "inertial_sensor.h"
#include "uart_device.h"
#include "vehicle.h"

#include "stdlib.h"
#include "math.h"
#include "arm_math.h"
#include "lowpass_filter2p.h"
#include "board_led.h"

#include "matrix3f.h"

#define ENABLE_DEBUG 1
#include "debug.h"

/* set interval to 1 second */
#define INTERVAL (1U * US_PER_SEC)
#define ERROR_INIT -1

extern kernel_pid_t spi_pid;
extern char device_periodic_stack[];

int main(void)
{
//    dma_init();
    xtimer_ticks32_t last_wakeup = xtimer_now();
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    DEBUG("order little endian\n");
#else
    DEBUG("order big endian\n");
#endif
#ifdef MODULE_PERIPH_INIT_SPI
    DEBUG("auto init spi\n");
#endif

#ifdef MODULE_PERIPH_UART_NONBLOCKING
    DEBUG("uart nonblocking\n");
#endif
#ifdef ENABLE_DEBUG
    DEBUG("enable debug\n");
#endif
#if defined(ARM_MATH_MVEF) && !defined(ARM_MATH_AUTOVECTORIZE)
    DEBUG("ARM_MATH_MVEF && ! AUTOVECTORIZE");
#else
#if defined(ARM_MATH_NEON) && !defined(ARM_MATH_AUTOVECTORIZE)
    DEBUG("ARM_MATH_NEON && !ARM_MATH_AUTOVECTORIZE");
#else
    DEBUG("!defined ARM_MATH MVEF NEON\n");
#endif
#endif
#if defined ARM_MATH_LOOPUNROLL
    DEBUG("defined ARM_MATH loopunroll\n");
#endif
#if defined ARM_MATH_AUTOVECTORIZE
    DEBUG("DEFINED ARM_MATH_AUTOVECTORIZE\n");
#endif
    bool init_stat = true;
    board_led_init();
    init_stat = serial_init();
    if (init_stat == false) {
        DEBUG("console init error\n");
        return ERROR_INIT;
    }
    MY_LOG("console init success\n");
    init_stat = vehicle_init();
    if (init_stat == false) {
        DEBUG("vehicle init error\n");
        return ERROR_INIT;
    }
    //float32_t pcoeffs[4] = {1.1, 2.2, 3.3, 4.4};
    //float32_t x3 = 1;
    //float32*4_t coeffs;
    //coeffs = vld1q(pcoeffs);
    /* biquad_t lpf; */
    /* set_cutoff_frequency(4000, 188, &lpf); */
    /* init_lowpass_filter(&lpf); */
    /* vector3f_t sample = {1.0, 2.0, 3.0}; */
    /* uint32_t start = xtimer_now().ticks32; */
    /* for(int i = 0; i < 1000; i++) { */
    /*     lowpass_filter_apply(&sample, &lpf); */
    /* } */
    /* uint32_t end = xtimer_now().ticks32; */
    /* DEBUG("1000 filter time = %ld\n", end - start); */
    /* vector3f_t * sample_p = &sample; */
    /* DEBUG("samplex = %d\n", (int)sample_p->x); */
    /* double mat[4][4] = {0}; */
    /* uint32_t start = xtimer_now().ticks32; */
    /* for (int i = 0; i < 1000; i++) { */
    /*     double temp = 1; */
    /*     for (int i = 6; i >= 0; i--) { */
    /*         int k = i<4?0:i - 3; */
    /*         for (int j = i - k; j>=k; j--) { */
    /*             mat[j][i - j] += temp; */
    /*         } */
    /*         temp *= 2.0; */
    /*     } */
    /* } */
    /* uint32_t end = xtimer_now().ticks32; */
    /* DEBUG(" double process time = %ld\n", end - start); */
    /* for (int i = 0; i < 4; i++) { */
    /*     for (int j = 0; i < 4; j++) { */
    /*         DEBUG(" %d", (int)mat[i][j]); */
    /*             } */
    /*     DEBUG("\n"); */
    /* } */
    matrix3f_t a = {{1.0, 1.5, 0.7}, {0.8, 0.8, 0.65}, {3.5, 4.34, 2.35}};
    matrix3f_t b = {{46.2, 349.2, 435.23}, {353.2,3.324,64.89}, {39,4.2,6.79}};
    matrix3f_t c;
    uint32_t start = xtimer_now().ticks32;
    for (int i = 0; i < 100; i++) {
        c = m3f_multi_m(&a, &b);
    }
    uint32_t end = xtimer_now().ticks32;
    DEBUG("100 matrix multi time = %ld\n", end - start);
    DEBUG("%d %d %d\n%d %d %d\n%d %d %d\n", (int)c.a.x, (int)c.a.y, (int)c.a.z,
          (int)c.b.x, (int)c.b.y, (int)c.b.z,
          (int)c.c.x, (int)c.c.y, (int)c.c.z);
    /* float av[9] = {1, 1.5, 0.7, 0.8, 0.8, 0.65, 3.5, 4.34, 2.35}; */
    /* float bv[9] = {46.2, 349.2, 435.23, 353.2, 3.324, 64.89, 39, 4.2, 6.79}; */
    /* float cv[9] = {0}; */
    /* arm_matrix_instance_f32 aa = {3, 3, av}; */
    /* arm_matrix_instance_f32 bb = {3, 3, bv}; */
    /* arm_matrix_instance_f32 cc = {3, 3, cv}; */
    /* uint32_t start2 = xtimer_now().ticks32; */
    /* for (int i = 0; i < 100; i++) { */
    /*     arm_mat_mult_f32_3*3_mve(&aa, &bb, &cc); */
    /* } */
    /* uint32_t end2 = xtimer_now().ticks32; */
    /* printf("arm matrix multi 100 time = %ld\n", end2 - start2); */
    /* for (int i = 0; i < 9; i++) { */
    /*     printf("%d ", (int)cv[i]); */
    /* } */
    /* printf("\n"); */
    /* vector3f_t test = {2, 2, 1}; */
    /* vector3f_t test2 = v3f_normalized(&test); */
    /* printf("test length = %d\n", (int)v3f_length(&test)); */
    /* printf("normalize {2, 2, 1} = {%d, %d, %d}\n", (int)(test2.x * 10), (int)test2.y * 10, */
    /*        (int)(test2.z * 10)); */
    uint32_t start3 = xtimer_now().ticks32;
    float ab = 1.345, cb = 7.958;
    float *abd=  &ab, *ccb = &cb;
    for (int i = 0; i < 100; i++) {
        *ccb = *ccb * *abd;
    }
    uint32_t end3 = xtimer_now().ticks32;
    DEBUG("mul time =  %ld, c = %f\n", end3 - start3, *ccb);
    DEBUG("float = %f\n", 4.321567);
    //thread_stack_print();
    //uint32_t size = thread_measure_stack_free(device_periodic_stack);
    //DEBUG("spi periodic thread free size = %ld\n", size);
    while(1) {
        xtimer_periodic_wakeup(&last_wakeup, INTERVAL);
        //uint32_t start = xtimer_now().ticks32;
        //__attribute__((unused)) int n;
        //MY_LOGN(n, "slept until %" PRIu32 ", xtimer=%lu\n",
        //xtimer_usec_from_ticks(xtimer_now()), xtimer_now().ticks32);
        //uint32_t end = xtimer_now().ticks32;
        //int n2;
        //MY_LOGN(n2, "last_write = %d, print time = %lu\n", n, end - start);
        //DEBUG("n2 = %d\n", n2);

        float a = -3.1;
        __attribute__((unused))float b;
        int i;
        __attribute__((unused))uint32_t start = xtimer_now().ticks32;
        for (i = 0; i < 100; i++) {
            b = fabs(a);
        }
        __attribute__((unused))uint32_t end = xtimer_now().ticks32;
        //DEBUG("fabs 100 time = %ld, time = %ld\n", end - start, end);
        /* uint32_t start2 = xtimer_now().ticks32; */
        /* for (i = 0; i < 100; i++) { */
        /*     arm_abs_f32(&a, &b, 1); */
        /* } */
        /* uint32_t end2 = xtimer_now().ticks32; */
        /* DEBUG("arm abs 100 = %ld, b = %f\n", end2 - start2, b); */

        /* uint32_t start = xtimer_now().ticks32; */
        /* uint32_t val = xtimer_now().ticks32; */
        /* uint32_t end = xtimer_now().ticks32; */
        /* DEBUG("32 time = %ld, val = %ld\n", end - start, val); */
        /* uint32_t start2 = xtimer_now().ticks32; */
        /* uint64_t val2 = xtimer_now64().ticks64; */
        /* uint32_t end2 = xtimer_now().ticks32; */
        /* DEBUG("64 time = 0x%lx, val2 = 0x%ld %ld\n", end2 - start2, (uint32_t)(val2 >> 32), */
        /*       (uint32_t)(val2 & 0xffffffff)); */

    }

    return 0;
}
