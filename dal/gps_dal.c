#include "gps_dal.h"
#include "dal.h"
#include "gps.h"
#include "location.h"

__attribute__((unused))static log_rgph_t rgph = {.primary_sensor = 0, .num_sensors = 1,};
static log_rgpi_t rgpi = {.antenna_offset = {0, 0, 0}};
static log_rgpj_t rgpj;

static location_t tmp_location;

location_t *dal_gps_location(void)
{
    tmp_location.lat = rgpj.lat;
    tmp_location.lng = rgpj.lng;
    tmp_location.alt = rgpj.alt;
    return &tmp_location;
}

void dal_gps_start_frame(void)
{
    rgpi.status = gps_status();
    location_t *loc = gps_location();
    rgpj.last_message_time_ms = gps_last_message_time_ms();
    rgpj.lat = loc->lat;
    rgpj.lng = loc->lng;
    rgpj.alt = loc->alt;
    rgpi.have_vertical_velocity = gps_have_vertical_velocity();
    rgpi.horizontal_accuracy_returncode = gps_horizontal_accuracy(&rgpj.hacc);
    rgpi.vertical_accuracy_returncode = gps_vertical_accuracy(&rgpj.vacc);
    rgpj.hdop = gps_get_hdop();
    rgpi.num_sats = gps_num_sats();
    rgpi.get_lag_returncode = gps_get_lag(&rgpi.lag_sec);
    rgpj.velocity = gps_velocity();
    rgpi.speed_accuracy_returncode = gps_speed_accuracy(&rgpj.sacc);
}

bool dal_gps_have_vertical_velocity(void)
{
    return rgpi.have_vertical_velocity;
}

bool dal_gps_get_lag(float *lag_sec)
{
    *lag_sec = rgpi.lag_sec;
    return rgpi.get_lag_returncode;
}

uint16_t dal_gps_get_hdop(void)
{
    return rgpj.hdop;
}

bool dal_gps_speed_accuracy(float *sacc)
{
    *sacc = rgpj.sacc;
    return rgpi.speed_accuracy_returncode;
}

dal_gps_status_enum_t dal_gps_status(void)
{
    return rgpi.status;
}

bool dal_gps_horizontal_accuracy(float *hacc)
{
    *hacc = rgpj.hacc;
    return rgpi.horizontal_accuracy_returncode;
}

uint32_t dal_gps_last_message_time_ms(void)
{
    return rgpj.last_message_time_ms;
}

bool dal_gps_vertical_accuracy(float *vacc)
{
    *vacc = rgpj.vacc;
    return rgpi.vertical_accuracy_returncode;
}

uint8_t dal_gps_num_sats(void)
{
    return rgpi.num_sats;
}

vector3f_t dal_gps_velocity(void)
{
    return rgpj.velocity;
}
