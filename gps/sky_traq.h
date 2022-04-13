#ifndef SKY_TRAQ_H_
#define SKY_TRAQ_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint16_t sos;
    uint16_t PL;
    uint8_t id;
    uint8_t com_port;
    uint8_t Baud_id;
    uint8_t Attributes;
    uint8_t CS;
    uint8_t end;
} skytra_baudrate_t;

typedef struct {
    uint16_t sos;
    uint16_t PL;
    uint8_t id;
    uint8_t GGA;
    uint8_t GSA;
    uint8_t GSV;
    uint8_t GLL;
    uint8_t RMC;
    uint8_t VTG;
    uint8_t ZDA;
    uint8_t Attributes;
    uint8_t CS;
    uint16_t end;
} skytra_outmsg_t;

typedef struct {
    uint16_t sos;
    uint16_t PL;
    uint8_t id;
    uint8_t rate;
    uint8_t Attributes;
    uint8_t CS;
    uint16_t end;
} skytra_pos_rate_t;

typedef struct {
    uint16_t sos;
    uint16_t PL;
    uint8_t id;
    uint8_t ACK_ID;
    uint8_t CS;
    uint16_t end;
} skytra_ack_t;

typedef struct {
    uint16_t sos;
    uint16_t PL;
    uint8_t id;
    uint8_t NACK_ID;
    uint8_t CS;
    uint16_t end;
} skytra_nack_t;

bool skytrag_cfg_ack_check(void);
bool skytra_cfg_rate(uint8_t frep);
bool skytra_cfg_msg_type(void);
#endif // SKY_TRAQ_H_
