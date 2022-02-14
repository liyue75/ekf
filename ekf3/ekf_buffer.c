#include <stdlib.h>
#include <string.h>
#include "ekf_buffer.h"

bool init_ekf_ring_buffer(ekf_ring_buffer_t *b, uint8_t elsize, uint8_t size)
{
    b->elsize = elsize;
    if (b->buffer) {
        free(b->buffer);
    }
    b->buffer = calloc(size, b->elsize);
    if (b->buffer == NULL) {
        return false;
    }
    b->_size = size;
    b->_head = 0;
    b->_tail = 0;
    b->_new_data = false;
    return true;
}

bool init_ekf_imu_buffer(ekf_imu_buffer_t *b, uint8_t elsize, uint8_t size)
{
    b->elsize = elsize;
    if (b->buffer) {
        free(b->buffer);
    }
    b->buffer = calloc(size, elsize);
    if (b->buffer == NULL) {
        return false;
    }
    b->_size = size;
    b->_youngest = 0;
    b->_oldest = 0;
    return true;
}

void *ekf_ring_buffer_get_offset(ekf_ring_buffer_t *b, uint8_t idx)
{
    return (void *)(((uint8_t *)b->buffer) + idx * (uint32_t)b->elsize);
}

void *ekf_imu_buffer_get_offset(ekf_imu_buffer_t *b, uint8_t idx)
{
    return (void *)(((uint8_t *)b->buffer) + idx * (uint32_t)b->elsize);
}

uint32_t *ekf_ring_buffer_time_ms(ekf_ring_buffer_t *b, uint8_t idx)
{
    ekf_obs_element_t *el = (ekf_obs_element_t *)ekf_ring_buffer_get_offset(b, idx);
    return &el->time_ms;
}

void ekf_ring_buffer_push(ekf_ring_buffer_t *b, const void *element)
{
    if (b->buffer == NULL) {
        return;
    }
    b->_head = (b->_head + 1) % b->_size;
    memcpy(ekf_ring_buffer_get_offset(b, b->_head), element, b->elsize);
    b->_new_data = true;
}

void ekf_ring_buffer_reset(ekf_ring_buffer_t *b)
{
    b->_head = 0;
    b->_tail = 0;
    b->_new_data = false;
    memset(b->buffer, 0, b->_size * (uint32_t)b->elsize);
}

bool ekf_ring_buffer_recalll(ekf_ring_buffer_t *b, void *element, uint32_t sample_time)
{
    if (!b->_new_data) {
        return false;
    }
    bool success = false;
    uint8_t tail = b->_tail;
    uint8_t best_index;
    if (b->_head == tail) {
        if (*ekf_ring_buffer_time_ms(b, tail) != 0 &&
            *ekf_ring_buffer_time_ms(b, tail) <= sample_time) {
            if ((sample_time - *ekf_ring_buffer_time_ms(b, tail)) < 100) {
                best_index = tail;
                success = true;
                b->_new_data = false;
            }
        }
    } else {
        while (b->_head != tail) {
            if (*ekf_ring_buffer_time_ms(b, tail) != 0 &&
                *ekf_ring_buffer_time_ms(b, tail) <= sample_time) {
                if ((sample_time - *ekf_ring_buffer_time_ms(b, tail)) < 100) {
                    best_index = tail;
                    success = true;
                }
            } else if (*ekf_ring_buffer_time_ms(b, tail) > sample_time) {
                break;
            }
            tail = (tail + 1) % b->_size;
        }
    }
    if (!success) {
        return false;
    }
    memcpy(element, ekf_ring_buffer_get_offset(b, best_index), b->elsize);
    b->_tail = (best_index + 1) % b->_size;
    *ekf_ring_buffer_time_ms(b, best_index) = 0;
    return true;
}

void ekf_imu_buffer_push_youngest_element(ekf_imu_buffer_t *b, const void *element)
{
    if (b->buffer == NULL) {
        return;
    }
    b->_youngest = (b->_youngest + 1) % b->_size;
    memcpy(ekf_imu_buffer_get_offset(b, b->_youngest), element, b->elsize);
    b->_oldest = (b->_youngest + 1) % b->_size;
    if (b->_oldest == 0) {
        b->_filled = true;
    }
}

void ekf_imu_buffer_get_oldest_element(ekf_imu_buffer_t *b, void *element)
{
    if (b->buffer == NULL) {
        memset(element, 0, b->elsize);
    } else {
        memcpy(element, ekf_imu_buffer_get_offset(b, b->_oldest), b->elsize);
    }
}

void ekf_imu_buffer_reset_history(ekf_imu_buffer_t *b, const void *element)
{
    for (uint8_t index=0; index < b->_size; index++) {
        memcpy(ekf_imu_buffer_get_offset(b, index), element, b->elsize);
    }
}

void ekf_imu_buffer_reset(ekf_imu_buffer_t *b)
{
    b->_youngest = 0;
    b->_oldest = 0;
    memset(b->buffer, 0, b->_size * (uint32_t)b->elsize);
}

void *ekf_imu_buffer_get(ekf_imu_buffer_t *b, uint8_t index)
{
    return ekf_imu_buffer_get_offset(b, index);
}
