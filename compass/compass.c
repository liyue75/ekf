#include "compass.h"
#include "vector3f.h"
#include "vector2f.h"
#include "xtimer.h"
#include "ak8963.h"
#include "compass_backend.h"
#include "compass_calibrator.h"
#include "location.h"
#include "ahrs.h"
#include "declination.h"
#include "uart_device.h"

#ifndef COMPASS_LEARN_DEFAULT
#define COMPASS_LEARN_DEFAULT COMPASS_LEARN_NONE
#endif

#ifndef COMPASS_OFFSETS_MAX_DEFAULT
#define COMPASS_OFFSETS_MAX_DEFAULT 1800
#endif

#ifndef COMPASS_FILTER_DEFAULT
#define COMPASS_FILTER_DEFAULT 0
#endif

#ifndef COMPASS_AUTO_ROT_DEFAULT
#define COMPASS_AUTO_ROT_DEFAULT 2
#endif

#if COMPASS_CAL_ENABLED
bool cal_saved[1];
bool cal_autosave;
#endif

static int8_t enabled = 1;
bool compass_cal_autoreboot;
bool cal_requires_reboot;
bool cal_has_run;
uint8_t compass_count = 0;
uint8_t backend_count;
Rotation_t board_orientation = ROTATION_NONE;
matrix3f_t *custom_rotation;
static float declination = 0;
static int8_t auto_declination = 1;
uint32_t log_bit = -1;
int8_t motor_comp_type = COMPASS_MOT_COMP_DISABLED;
int8_t rotate_auto = COMPASS_AUTO_ROT_DEFAULT;
float custom_roll = 0, custom_pitch = 0, custom_yaw = 0;
float _thr;

mag_state_t _compass_state[COMPASS_MAX_INSTANCES + 1] = {{.external = 0, .offset = {0, 0, 0},
    .motor_compensation = {0, 0, 0}, .orientation = ROTATION_NONE, .dev_id = 0,
    .diagonals = {0, 0, 0}, .offdiagonals = {0, 0, 0}, .scale_factor = 0}};
static int8_t use_for_yaw[COMPASS_MAX_INSTANCES] = {1};
int16_t offset_max = COMPASS_OFFSETS_MAX_DEFAULT;
int options = 0;
float calibration_threshold = COMPASS_CALIBRATION_FITNESS_DEFAULT;
int32_t driver_type_mask = 0;

#if COMPASS_MAX_UNREG_DEV
int32_t extra_dev_id[COMPASS_MAX_UNREG_DEV] = {0, 0, 0, 0, 0};
uint32_t previously_unreg_mag[COMPASS_MAX_UNREG_DEV];
#endif
int8_t _filter_range = COMPASS_FILTER_DEFAULT;
bool learn_allocated;
int8_t _compass_learn = COMPASS_LEARN_NONE;
static bool init_done = false;
__attribute__((unused))static uint8_t first_usable;
static bool initial_location_set = false;

bool compass_available(void)
{
    return enabled && init_done;
}

bool compass_healthy(void)
{
    return _compass_state[0].healthy;
}

static bool add_backend(void)
{
    /*  use mpu9250 auxiliary i2c instead of spi to connect to compass ak8963 */
    //ak8963_probe_mpu9250();
    ak8963_probe();
    backend_count++;
    return true;
}

static void detect_backends(void)
{
    //ADD_BACKEND(DRIVER_MSP, MSP);
    add_backend();
}

static void try_set_initial_location(void)
{
    if (!auto_declination) {
        return;
    }
    if (!enabled) {
        return;
    }
    location_t loc;
    if (!ahrs_get_position(&loc)) {
        return;
    }
    initial_location_set = true;
    declination = radians(get_declination((float)loc.lat / 10000000,
                                          (float)loc.lng / 10000000));
    //MY_LOG("init loc declination= %d\n", (int)(get_declination(loc.lat / 10000000,
    //                                                                   loc.lng / 10000000)));
}

bool compass_read(void)
{
    if (!initial_location_set) {
        try_set_initial_location();
    }
    compass_drain_accumlated_sample();
    uint32_t time = xtimer_now().ticks32 / 1000;
    _compass_state[0].healthy = (time - _compass_state[0].last_update_ms < 500);
    //first_usable = 0;
    return _compass_state[0].healthy;
}

void compass_init(void)
{
    if(!enabled) {
        return;
    }
    compass_backend_init();
    for (uint8_t i = 0; i < COMPASS_MAX_INSTANCES; i++) {
        _compass_state[i].expected_dev_id = _compass_state[i].dev_id;
    }
    if (compass_count == 0) {
        detect_backends();
    }
    xtimer_usleep(100000);
    compass_read();
    init_done = true;
}

bool compass_have_scale_factor(void)
{
    if (!compass_available()) {
        return false;
    }
    if (_compass_state[0].scale_factor < COMPASS_MIN_SCALE_FACTOR ||
    _compass_state[0].scale_factor > COMPASS_MAX_SCALE_FACTOR) {
        return false;
    }
    return true;
}


float compass_get_declination(void)
{
    return declination;
}

vector3f_t *compass_get_field(void)
{
    return &_compass_state[0].field;
}

bool compass_auto_declination_enabled(void)
{
    return auto_declination != 0;
}

bool compass_consistent(void)
{
    /* const vector3f_t *primary_mag_field = &_compass_state[0].field; */
    /* const vector2f_t primary_mag_field_xy = {primary_mag_field->x, primary_mag_field->y}; */
    /* if (float_is_zero(primary_mag_field_xy.x) && float_is_zero(primary_mag_field_xy.y)) { */
    /*     return false; */
    /* } */
    /* const vector3f_t primary_mag_field_norm = v3f_normalized(primary_mag_field); */
    /* const vector2f_t primary_mag_field_xy_norm = v2f_normalized(&primary_mag_field_xy); */
    /* if (!compass_use_for_yaw()) { */
    /*     return true; */
    /* } */
    /* vector3f_t mag_field = _compass_state[0].field; */
    /* vector2f_t mag_field_xy = {mag_field.x, mag_field.y}; */
    return true;
}

uint32_t compass_last_update_usec(void)
{
    return _compass_state[0].last_update_usec;
}


bool compass_use_for_yaw(void)
{
    if (!compass_available()) {
        return false;
    }
    return use_for_yaw[0] && (_compass_learn != COMPASS_LEARN_INFLIGHT);
}
