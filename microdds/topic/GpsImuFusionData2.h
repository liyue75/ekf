// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*! 
 * @file GpsImuFusionData2.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _GpsImuFusionData2_H_
#define _GpsImuFusionData2_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

/*!
 * @brief This struct represents the structure GpsImuFusionData2 defined by the user in the IDL file.
 * @ingroup GPSIMUFUSIONDATA2
 */
typedef struct GpsImuFusionData2
{
    int32_t latitude;
    int32_t longitude;
    int32_t altitude;
    float velocity_n;
    float velocity_e;
    float velocity_d;
    float roll;
    float pitch;
    float yaw;
} GpsImuFusionData2;

struct ucdrBuffer;

bool GpsImuFusionData2_serialize_topic(struct ucdrBuffer* writer, const GpsImuFusionData2* topic);
bool GpsImuFusionData2_deserialize_topic(struct ucdrBuffer* reader, GpsImuFusionData2* topic);
uint32_t GpsImuFusionData2_size_of_topic(const GpsImuFusionData2* topic, uint32_t size);


#ifdef __cplusplus
}
#endif

#endif // _GpsImuFusionData2_H_