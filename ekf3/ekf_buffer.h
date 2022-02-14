#ifndef EKF_BUFFER_H_
#define EKF_BUFFER_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t time_ms;
} ekf_obs_element_t;

typedef struct {
    uint8_t elsize;
    void *buffer;
    uint8_t _size, _head, _tail, _new_data;
} ekf_ring_buffer_t;

typedef ekf_ring_buffer_t ekf_obs_buffer_t;

typedef struct {
    uint8_t elsize;
    void *buffer;
    uint8_t _size, _oldest, _youngest;
    bool _filled;
} ekf_imu_buffer_t;

bool init_ekf_ring_buffer(ekf_ring_buffer_t *b, uint8_t elsize, uint8_t size);
bool init_ekf_imu_buffer(ekf_imu_buffer_t *b, uint8_t elsize, uint8_t size);
void *ekf_ring_buffer_get_offset(ekf_ring_buffer_t *b, uint8_t idx);
void *ekf_imu_buffer_get_offset(ekf_imu_buffer_t *b, uint8_t idx);
uint32_t *ekf_ring_buffer_time_ms(ekf_ring_buffer_t *b, uint8_t idx);
void ekf_ring_buffer_push(ekf_ring_buffer_t *b, const void *element);
void ekf_ring_buffer_reset(ekf_ring_buffer_t *b);
bool ekf_ring_buffer_recalll(ekf_ring_buffer_t *b, void *element, uint32_t sample_time);
void ekf_imu_buffer_push_youngest_element(ekf_imu_buffer_t *b, const void *element);
void ekf_imu_buffer_get_oldest_element(ekf_imu_buffer_t *b, void *element);
void ekf_imu_buffer_reset_history(ekf_imu_buffer_t *b, const void *element);
void ekf_imu_buffer_reset(ekf_imu_buffer_t *b);
void *ekf_imu_buffer_get(ekf_imu_buffer_t *b, uint8_t index);
#endif // EKF_BUFFER_H_
