#include <string.h>
#include <stdio.h>
#include "net/netif.h"
#include "microdds.h"

#define ENABLE_DEBUG        (0)
#include "debug.h"

static uint32_t get_mac_key(void)
{
    netif_t * netif = NULL;
    netif = netif_iter(netif);
    uint8_t mac[6] = {0};
    netif_get_opt(netif, NETOPT_ADDRESS, 0, mac, 6);
    uint32_t key = (mac[2]<<24) + (mac[3]<<16) + (mac[4]<<8) + mac[5];

    return key;
}

static microdds_t ses;
microdds_t *microdds = &ses;

int microdds_init(const microdds_init_arg_t *args)
{
    char ip[150] = {0};
    char port[10] = {0};
//    const char *ip = args->agent_ip;
//    const char *port = args->agent_port;
    strcpy(ip, args->agent_ip);
    strcpy(port, args->agent_port);
    //uint16_t domain = args->participant_domain;
    uint16_t domain = 1;
    if(!uxr_init_udp_transport(&ses.transport, UXR_IPv6, ip, port))
    {
        DEBUG("Error at create transport.\n");
        uxr_close_udp_transport(&ses.transport);
        return MICRODDS_ER_INIT_UDP;
    }

    uint32_t key = get_mac_key();
    // Session
    uxr_init_session(&ses.session, &ses.transport.comm, key);
    if(!uxr_create_session(&ses.session))
    {
        DEBUG("Error at create session.\n");
        uxr_close_udp_transport(&ses.transport);
        uxr_delete_session(&ses.session);
        return MICRODDS_ER_CRE_SES;
    }

    ses.best_effort_out = uxr_create_output_best_effort_stream(&ses.session, ses.best_effort_buffer_out, 2048);
    ses.best_effort_in = uxr_create_input_best_effort_stream(&ses.session);
    ses.reliable_out = uxr_create_output_reliable_stream(&ses.session, ses.reliable_buffer_out, 2048, 2);
    ses.reliable_in = uxr_create_input_reliable_stream(&ses.session, ses.reliable_buffer_in, 2048, 2);

    ses.participant_id = uxr_object_id(1, UXR_PARTICIPANT_ID);
    const char* participant_xml = "<dds>"
                                  "<profiles>"
                                    "<transport_descriptors>"
                                            "<transport_descriptor>"
                                                "<transport_id>udp</transport_id>"
                                                "<type>UDPv4</type>"
                                                "<non_blocking_send>true</non_blocking_send>"
                                            "</transport_descriptor>"
                                        "</transport_descriptors>"
                                "</profiles>"
                                      "<participant>"
                                          "<rtps>"
                                              "<name>default_xrce_participant</name>"
                                              "<useBuiltinTransports>false</useBuiltinTransports>"
                                              "<userTransports>"
                                                    "<transport_id>udp</transport_id>"
                                              "</userTransports>"
                                          "</rtps>"
                                      "</participant>"
                                  "</dds>";
    uint16_t participant_req = uxr_buffer_create_participant_xml(&ses.session, ses.reliable_out, ses.participant_id, domain, participant_xml, UXR_REPLACE);
    ses.publisher_id = uxr_object_id(0x01, UXR_PUBLISHER_ID);
    const char* publisher_xml = "";

    uint16_t publisher_req = uxr_buffer_create_publisher_xml(&ses.session, ses.reliable_out, ses.publisher_id, ses.participant_id, publisher_xml, UXR_REPLACE);

    ses.subscriber_id = uxr_object_id(0x01, UXR_SUBSCRIBER_ID);
    const char* subscriber_xml = "";

    uint16_t subscriber_req = uxr_buffer_create_subscriber_xml(&ses.session, ses.reliable_out, ses.subscriber_id, ses.participant_id, subscriber_xml, UXR_REPLACE);

    uint8_t status[3];
    uint16_t requests[3] = {participant_req, publisher_req, subscriber_req};


    if(!uxr_run_session_until_all_status(&ses.session, 1000, requests, status, 3))
    {
        DEBUG("Error at create entities: participant: %i publisher: %i subscriber: %i\n", status[0], status[1], status[2]);
        return MICRODDS_ER_CRE_PAR;
    }
    ses.topic_count = 0;
    ses.have_init = true;
    return MICRODDS_OK;
}

int microdds_create_topic(const microdds_topic_t *topic)
{
    if(ses.have_init == false) {
        DEBUG("dds_ses haven't been inited\n");
        return MICRODDS_NOT_INIT;
    }
    uxrObjectId topic_id = uxr_object_id(++ses.topic_count, UXR_TOPIC_ID);
    char xml_buf[300] = {0};
    sprintf(xml_buf, "<dds>"
                                "<topic>"
                                    "<name>%s</name>"
                                    "<dataType>%s</dataType>"
                                "</topic>"
                            "</dds>", topic->topic_name, topic->topic_type);
    uint16_t topic_req = uxr_buffer_create_topic_xml(&ses.session, ses.reliable_out, topic_id, ses.participant_id, xml_buf, UXR_REPLACE);
     uint8_t status[1];
    uint16_t requests[1] = {topic_req};
    if(!uxr_run_session_until_all_status(&ses.session, 1000, requests, status, 1))
    {
        DEBUG("Error at create entities: topic: %i\n", status[0]);
        return MICRODDS_ER_CRE_TOP;
    }
    return MICRODDS_OK;
}


int microdds_create_writer(uint16_t writer_id, const microdds_topic_t *topic)
{
     if(ses.have_init == false) {
        DEBUG("dds_ses haven't init");
        return MICRODDS_NOT_INIT;
    }
    uxrObjectId datawriter_id = uxr_object_id(writer_id, UXR_DATAWRITER_ID);
    char xml_buf[500] = {0};
    sprintf(xml_buf, "<dds>"
                        "<data_writer>"
                            "<qos>"
                                "<reliability>"
                                    "<kind>BEST_EFFORT</kind>"
                                "</reliability>"
                                "<durability>"
                                    "<kind>VOLATILE</kind>"
                                "</durability>"
                                "<publishMode>"
                                    "<kind>ASYNCHRONOUS</kind>"
                                "</publishMode>"
                            "</qos>"
                            "<topic>"
                                "<kind>NO_KEY</kind>"
                                "<name>%s</name>"
                                "<dataType>%s</dataType>"
                            "</topic>"
                        "</data_writer>"
                    "</dds>",
            topic->topic_name, topic->topic_type);

    uint16_t datawriter_req = uxr_buffer_create_datawriter_xml(&ses.session, ses.reliable_out, datawriter_id, ses.publisher_id, xml_buf, UXR_REPLACE);
    uint8_t status[1];
    uint16_t requests[1] = {datawriter_req};
    if(!uxr_run_session_until_all_status(&ses.session, 1000, requests, status, 1))
    {
        DEBUG("Error at create entities: darawriter: %i\n", status[0]);
        return MICRODDS_ER_CRE_DW;
    }

    return MICRODDS_OK;
}

int microdds_create_reader(uint16_t reader_id, const microdds_topic_t * topic, const microdds_cb_t *cb)
{
     if(ses.have_init == false) {
        DEBUG("dds_ses haven't init");
        return MICRODDS_NOT_INIT;
    }
    uxr_set_topic_callback(&ses.session, cb->fun, cb->args);
    uxrObjectId datareader_id = uxr_object_id(reader_id, UXR_DATAREADER_ID);
    char xml_buf[500] = {0};
    sprintf(xml_buf, "<dds>"
                        "<data_reader>"
                            "<qos>"
                                "<reliability>"
                                    "<kind>BEST_EFFORT</kind>"
                                "</reliability>"
                                "<durability>"
                                    "<kind>VOLATILE</kind>"
                                "</durability>"
                            "</qos>"
                            "<topic>"
                                "<kind>NO_KEY</kind>"
                                "<name>%s</name>"
                                "<dataType>%s</dataType>"
                            "</topic>"
                        "</data_reader>"
                    "</dds>", topic->topic_name, topic->topic_type);
    uint16_t datareader_req = uxr_buffer_create_datareader_xml(&ses.session, ses.reliable_out, datareader_id, ses.subscriber_id, xml_buf, UXR_REPLACE);
    uint8_t status[1];
    uint16_t requests[1] = {datareader_req};
    if(!uxr_run_session_until_all_status(&ses.session, 1000, requests, status, 1))
    {
        DEBUG("Error at create entities: datareader: %i\n", status[0]);
        return MICRODDS_ER_CRE_DR;
    }
    uxrDeliveryControl delivery_control = {0};
    delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
    uxr_buffer_request_data(&ses.session, ses.reliable_out, datareader_id, ses.best_effort_in, &delivery_control);

    return MICRODDS_OK;
}

int microdds_pub(void)
{
     if(ses.have_init == false) {
        DEBUG("dds_ses haven't init");
        return MICRODDS_NOT_INIT;
    } else {
        uxr_flash_output_streams(&ses.session);
        return MICRODDS_OK;
    }
}

//arg mode = 1, until a message is received or the timeout exceeded
//    mode = 0, until the waiting timeout for a new message
//arg timeout:  0~4294000mills
int microdds_listen(uint32_t timeout, bool mode)
{
    if(ses.have_init == false) {
        DEBUG("dds_ses haven't init");
        return MICRODDS_NOT_INIT;
    }
    bool ret = false;
    if (mode) {
        ret = uxr_run_session_until_timeout(&ses.session, timeout);
    }
    else {
        ret = uxr_run_session_time(&ses.session, timeout);
    }
    if (ret == true) {
        return MICRODDS_OK;
    }
    return MICRODDS_NOT_COM;
}

int microdds_close(void)
{
     if(ses.have_init == false) {
        DEBUG("dds_ses haven't init");
        return MICRODDS_NOT_INIT;
    } else {
        uxr_delete_session(&ses.session);
        uxr_close_udp_transport(&ses.transport);
        ses.have_init = false;
        return MICRODDS_OK;
    }
}

