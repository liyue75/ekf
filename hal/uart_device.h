#ifndef UART_DEVICE_H_
#define UART_DEVICE_H_

#include <stdio.h>
#include "cond.h"
#include "mutex.h"
#include "tsrb.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HAL_UART_TX_PIPE_MAX_ADD_LENGTH 256

typedef struct uart_tx_pipe {
    tsrb_t rb;
    //cond_t read_cond;
    cond_t cond;
    mutex_t lock;
} uart_tx_pipe_t;
extern uart_tx_pipe_t _console_tx_pipe;

// add string to console pipe, string length must less than HAL_UART_TX_PIPE_MAX_ADD_LENGTH
#define MY_LOGN(total_write, ...) do { \
    uint32_t free_space = tsrb_free(&_console_tx_pipe.rb); \
    if (free_space == 0) { \
        total_write = 0;   \
        break;             \
    } \
    char str[HAL_UART_TX_PIPE_MAX_ADD_LENGTH ]; \
    size_t size = HAL_UART_TX_PIPE_MAX_ADD_LENGTH  <= free_space? \
        HAL_UART_TX_PIPE_MAX_ADD_LENGTH  : free_space; \
    int total = snprintf(str, size, __VA_ARGS__); \
    if (total >= (int)size) { \
        total = size - 1; \
    }                     \
    total_write = tsrb_add(&_console_tx_pipe.rb, (uint8_t *)str, total + 1); \
    cond_signal(&_console_tx_pipe.cond); \
} while(0)

#define MY_LOG(...) do { \
    uint32_t free_space = tsrb_free(&_console_tx_pipe.rb); \
    if (free_space == 0) { \
        break;             \
    } \
    char str[HAL_UART_TX_PIPE_MAX_ADD_LENGTH ]; \
    size_t size = HAL_UART_TX_PIPE_MAX_ADD_LENGTH  <= free_space? \
    HAL_UART_TX_PIPE_MAX_ADD_LENGTH  : free_space;            \
    int total = snprintf(str, size, __VA_ARGS__);   \
    if (total >= (int)size) {                               \
        total = size - 1;                                   \
    }                                                       \
    tsrb_add(&_console_tx_pipe.rb, (uint8_t *)str, total + 1); \
    cond_signal(&_console_tx_pipe.cond); \
} while(0)

bool serial_init(void);

#ifdef __cplusplus
}
#endif

#endif // UART_DEVICE_H_
