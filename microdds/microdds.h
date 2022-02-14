#ifndef MICRO_DDS_H
#define MICRO_DDS_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "uxr/client/client.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uxrUDPTransport transport;
    uxrUDPPlatform udp_platform;
    uxrSession session;
    uint8_t best_effort_buffer_out[2048];
    uint8_t reliable_buffer_out[2048];
    uint8_t reliable_buffer_in[2048];
    uxrStreamId best_effort_out;
    uxrStreamId best_effort_in;
    uxrStreamId reliable_out;
    uxrStreamId reliable_in;
    uxrObjectId participant_id;
    uxrObjectId publisher_id;
    uxrObjectId subscriber_id;
    bool have_init;
    uint16_t topic_count;
} microdds_t;

extern microdds_t *microdds;

typedef struct {
    const char *agent_ip;
    const char *agent_port;
    //uint16_t participant_domain;
} microdds_init_arg_t;

typedef struct {
    const char *topic_name;
    const char *topic_type;
} microdds_topic_t;

typedef struct {
    uxrOnTopicFunc fun;
    void *args;
} microdds_cb_t;

enum {
    MICRODDS_OK             = 0,
    MICRODDS_NOT_INIT       = -1,
    MICRODDS_ER_INIT_UDP    = -2,
    MICRODDS_ER_CRE_SES     = -3,
    MICRODDS_ER_CRE_PAR     = -4,
    MICRODDS_ER_CRE_TOP     = -5,
    MICRODDS_ER_CRE_DW      = -6,
    MICRODDS_ER_CRE_DR      = -7,
    MICRODDS_NOT_COM        = -8,
};


int microdds_init(const microdds_init_arg_t *args);
int microdds_create_topic(const microdds_topic_t *topic);
int microdds_create_writer(uint16_t writer_id, const microdds_topic_t *topic);
int microdds_create_reader(uint16_t reader_id, const microdds_topic_t *topic, const microdds_cb_t *cb);
int microdds_pub(void);
int microdds_listen(uint32_t timeout, bool mode);
int microdds_close(void);


#ifdef __cplusplus
}
#endif

#endif
