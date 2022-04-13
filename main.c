#include <stdio.h>
#include "xtimer.h"
#include "timex.h"
#include "periph/spi.h"
#include "thread.h"

#include "inertial_sensor.h"
#include "uart_device.h"
#include "vehicle.h"
#include "mutex.h"
#include "ringbuffer.h"
#include "common_nmea.h"
#include "ak8963.h"
#include "ekf3_core.h"
#include "nav_common.h"
#include "microdds_user.h"

#include "stdlib.h"
#include "math.h"
#include "gps.h"
//#include "arm_math.h"
#include "lowpass_filter2p.h"
#include "board_led.h"
#include "matrix3f.h"
#include "vector3f.h"
#include "fusion_math.h"

#define ENABLE_DEBUG 1
#include "debug.h"

#define ERROR_INIT -1

extern ringbuffer_t gps_buffer;

int main(void)
{
    //DEBUG("main thread priority = %d\n", thread_get_active()->priority);
    bool init_stat = true;
    board_led_init();
    init_stat = serial_init();
    if (init_stat == false) {
        DEBUG("console init error\n");
        return ERROR_INIT;
    }
    //MY_LOG("console init success\n");
    init_stat = vehicle_init();
    if (init_stat == false) {
        DEBUG("vehicle init error\n");
        return ERROR_INIT;
    }
    //microdds_user_init();
    //ringbuffer_remove(&gps_buffer, 512);
    //float alpha = calc_lowpass_alpha_dt(0.02, 20);
    //MY_LOG("50hz sample rate, 20hz cutoff lowpass filter alpha = %f\n", alpha);
    while(1) {
        vehicle_loop();
        /* int16_t c; */
        /* while ((c = ringbuffer_get_one(&gps_buffer)) != -1) { */
        /*     DEBUG("%c", c); */
        /* } */
        /* xtimer_usleep(1000); */
    }

    return 0;
}
