#ifndef GPS_BACKEND_H_
#define GPS_BACKEND_H_

#include <stdint.h>
void make_gps_time(uint32_t bcd_date, uint32_t bcd_milliseconds);

void set_uart_timestamp(uint16_t nbytes);
void fill_3d_velocity(void);
#endif // GPS_BACKEND_H_
