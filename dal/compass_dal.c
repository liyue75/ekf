#include "compass_dal.h"
#include "dal.h"
#include "compass.h"

static log_rmgh_t rmgh = {.count = 1, .num_enabled = 1, .consistent = true,
.first_usable = 0};
static log_rmgi_t rmgi = {.offsets = {0, 0, 0}};

void dal_compass_start_frame(void)
{
    rmgh.available = compass_available();
    rmgh.auto_declination_enabled = compass_auto_declination_enabled();
    rmgh.declination = compass_get_declination();
    //rmgh.consistent = compass_consistent();
    rmgi.use_for_yaw = compass_use_for_yaw();
    rmgi.healthy = compass_healthy();
    rmgi.have_scale_factor = compass_have_scale_factor();
    rmgi.last_update_usec = compass_last_update_usec();
    rmgi.field = *compass_get_field();
}

bool dal_compass_healthy(void)
{
    return rmgi.healthy;
}

float dal_compass_get_declination(void)
{
    return rmgh.declination;
}

bool dal_compass_use_for_yaw(void)
{
    return rmgi.use_for_yaw;
}

bool dal_compass_have_scale_factor(void)
{
    return rmgi.have_scale_factor;
}

bool dal_compass_auto_declination_enabled(void)
{
    return rmgh.auto_declination_enabled;
}
