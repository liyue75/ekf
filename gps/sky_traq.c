#include "sky_traq.h"
#include "gps.h"
#include "xtimer.h"
#include "ringbuffer.h"
#include "periph/uart.h"

extern ringbuffer_t gps_buffer;

bool skytra_cfg_ack_check(void)
{
    bool rval = false;
    uint32_t start = xtimer_now().ticks32 / 1000;
    while (xtimer_now().ticks32 / 1000 - start < 500) {
        int16_t c;
        while ((c = ringbuffer_get_one(&gps_buffer)) != -1) {
            if ((uint8_t)c == 0x83) {
                rval = true;
                break;
            }
        }
        if (rval == true) {
            break;
        }
        xtimer_usleep(5000);
    }
    return rval;
}

static void skytra_send_data(uint8_t *dbuf, uint16_t len)
{
    uart_write(GPS_UART_DEV, dbuf, len);
}

bool skytra_cfg_rate(uint8_t frep)
{
    uint8_t tx_buf[sizeof(skytra_pos_rate_t)];
    skytra_pos_rate_t *cfg_rate = (skytra_pos_rate_t *)tx_buf;
    cfg_rate->sos = 0xA1A0;
    cfg_rate->PL = 0x0300;
    cfg_rate->id = 0x0E;
    cfg_rate->rate = frep;
    cfg_rate->Attributes = 0x01;
    cfg_rate->CS = cfg_rate->id ^ cfg_rate->rate ^ cfg_rate->Attributes;
    cfg_rate->end = 0x0A0D;
    skytra_send_data((uint8_t *)cfg_rate, sizeof(skytra_pos_rate_t));
    return skytra_cfg_ack_check();
}

bool skytra_cfg_msg_type(void)
{
    uint8_t tx_buf[sizeof(skytra_outmsg_t)];
    skytra_outmsg_t *cfg_msg = (skytra_outmsg_t *)tx_buf;
    cfg_msg->sos = 0xA1A0;
    cfg_msg->PL = 0x0900;
    cfg_msg->id = 0x08;
    cfg_msg->GGA = 0x01;
    cfg_msg->GSA = 0;
    cfg_msg->GSV = 0;
    cfg_msg->GLL = 0;
    cfg_msg->RMC = 0x01;
    cfg_msg->VTG = 0x01;
    cfg_msg->ZDA = 0;
    cfg_msg->Attributes = 0x01;
    cfg_msg->CS = cfg_msg->id ^ cfg_msg->GGA ^ cfg_msg->GSA ^ cfg_msg->GSV ^
        cfg_msg->GLL ^ cfg_msg->RMC ^ cfg_msg->VTG ^ cfg_msg->ZDA ^ cfg_msg->Attributes;
    cfg_msg->end = 0x0A0D;
    skytra_send_data((uint8_t *)cfg_msg, sizeof(skytra_outmsg_t));
    return skytra_cfg_ack_check();
}
