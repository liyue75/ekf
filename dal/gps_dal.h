#ifndef GPS_DAL_H_
#define GPS_DAL_H_

#include <stdbool.h>
#include "location.h"
#include "vector3f.h"

typedef enum {
DAL_NO_GPS,
DAL_NO_FIX,
DAL_GPS_OK_FIX_2D,
DAL_GPS_OK_FIX_3D,
DAL_GPS_OK_FIX_3D_DGPS,
DAL_GPS_OK_FIX_3D_RTK_FLOAT,
DAL_GPS_OK_FIX_3D_RTK_FIXED
} dal_gps_status_enum_t;

void dal_gps_start_frame(void);
location_t *dal_gps_location(void);
dal_gps_status_enum_t dal_gps_status(void);
bool dal_gps_speed_accuracy(float *sacc);
bool dal_gps_horizontal_accuracy(float *hacc);
bool dal_gps_vertical_accuracy(float *vacc);
uint16_t dal_gps_get_hdop(void);
uint8_t dal_gps_num_sats(void);
uint32_t dal_gps_last_message_time_ms(void);
bool dal_gps_get_lag(float *lag_sec);
vector3f_t dal_gps_velocity(void);
bool dal_gps_have_vertical_velocity(void);

#endif // GPS_DAL_H_
