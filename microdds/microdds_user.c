#include "microdds_user.h"
#include "uart_device.h"
#include "xtimer.h"
#include "GpsImuFusionData2.h"
#include "ahrs.h"
#include "fusion_math.h"

#define ENABLE_DEBUG 1
#include "debug.h"

#define LINK_ADDR "fe80::f58c:2fb7:adad:1%4"
#define LINK_PORT "2021"
#define TOPIC_PUBNAME_DEFAULT "rt/gps_imu_fusion/data2"
#define TOPIC_PUB_TYPE "mymsg::msg::dds_::GpsImuFusionData2_"
#define WRITER_ID_1 1

extern bool _ekf3_start_using_gps;
extern location_t _ahrs_current_loc;
extern vector3f_t _ahrs_velocity;
extern float _ahrs_roll;
extern float _ahrs_pitch;
extern float _ahrs_yaw;

void microdds_user_init(void)
{
    __attribute__((unused))microdds_init_arg_t microdds_args = {
        .agent_ip = LINK_ADDR,
        .agent_port = LINK_PORT,
    };
    DEBUG("microdds connecting ...\n");
    while (microdds_init(&microdds_args)) {
        DEBUG("microdds retry connect");
        xtimer_usleep(100000);
    }
    microdds_topic_t gps_imu_fusion_data_topic = {TOPIC_PUBNAME_DEFAULT, TOPIC_PUB_TYPE};
    microdds_create_topic(&gps_imu_fusion_data_topic);
    microdds_create_writer(WRITER_ID_1, &gps_imu_fusion_data_topic);
}

void pub_absolute_position(GpsImuFusionData2 *data)
{
    ucdrBuffer ub;
    uint32_t topic_size = GpsImuFusionData2_size_of_topic(data, 0);
    uxrObjectId datawriter_id = uxr_object_id(WRITER_ID_1, UXR_DATAWRITER_ID);
    uxr_prepare_output_stream(&microdds->session, microdds->best_effort_out, datawriter_id,
                              &ub, topic_size);
    GpsImuFusionData2_serialize_topic(&ub, data);
    microdds_pub();
}

static uint32_t pub_time = 0;
void microdds_send_data(void)
{
    if (_ekf3_start_using_gps) {
        __attribute__((unused))GpsImuFusionData2 pub_data;
        pub_data.latitude = _ahrs_current_loc.lat;
        pub_data.longitude = _ahrs_current_loc.lng;
        pub_data.altitude = _ahrs_current_loc.alt;
        pub_data.velocity_n = _ahrs_velocity.x;
        pub_data.velocity_e = _ahrs_velocity.y;
        pub_data.velocity_d = _ahrs_velocity.z;
        pub_data.roll = degrees(_ahrs_roll);
        pub_data.pitch = degrees(_ahrs_pitch);
        pub_data.yaw = degrees(_ahrs_yaw);
        pub_absolute_position(&pub_data);
        /* MY_LOG("current loc %ld %ld %ld, vel : %f %f %f, att: %f %f %f\n", */
        /*        pub_data.longitude, pub_data.latitude, pub_data.altitude, */
        /*        pub_data.velocity_n, pub_data.velocity_e, pub_data.velocity_d, */
        /*        pub_data.roll, pub_data.pitch, pub_data.yaw); */
       //MY_LOG("send microdds data\n");
        uint32_t now = xtimer_now().ticks32 / 1000;
        if (now - pub_time >= 1000) {
         MY_LOG("current loc %ld %ld %ld, vel : %f %f %f, att: %f %f %f\n",
               pub_data.longitude, pub_data.latitude, pub_data.altitude,
               pub_data.velocity_n, pub_data.velocity_e, pub_data.velocity_d,
               pub_data.roll, pub_data.pitch, pub_data.yaw);
           pub_time = now;
        }
    }
}
