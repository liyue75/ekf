#include "periph/uart.h"
#include "thread.h"
#include "uart_device.h"
#include "tsrb.h"
#include "xtimer.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

#define HAL_UART_TX_STACK_SIZE 2048
#define HAL_UART_DEV UART_DEV(0)
//#define HAL_UART_BAUD 115200
#define HAL_UART_BAUD 921600
#define HAL_UART_TX_THREAD_PRIORITY 11
#define UART_TX_THREAD_STACKSIZE 3096

static uint8_t console_tx_rb_buf[HAL_UART_TX_STACK_SIZE];
uart_tx_pipe_t _console_tx_pipe;
static char console_tx_thread_stack[UART_TX_THREAD_STACKSIZE /* THREAD_STACKSIZE_DEFAULT */];

static void *uart_tx_thread(__attribute__((unused))void *arg)
{
    assert(_console_tx_pipe.rb.buf);
    uint8_t tmp_buf[HAL_UART_TX_STACK_SIZE];
    //__attribute__((unused))uint32_t last_thread_run_us = 0;
    //uint16_t rb_read_pos = 0;
    //uint16_t rb_write_pos = 0;
    while (true) {
        if (tsrb_empty(&_console_tx_pipe.rb)) {
            //DEBUG("rb empty\n");
            mutex_lock(&_console_tx_pipe.lock);
            cond_wait(&_console_tx_pipe.cond, &_console_tx_pipe.lock);
            mutex_unlock(&_console_tx_pipe.lock);
        }
        //DEBUG("rb not empty\n");
        uint32_t ava_bytes = tsrb_avail(&_console_tx_pipe.rb);
        int got_bytes = tsrb_get(&_console_tx_pipe.rb, tmp_buf, ava_bytes);
        uart_write(HAL_UART_DEV, tmp_buf, got_bytes);
    }
    return NULL;
}

/* static bool thread_rx_init(void) */
/* { */
/*     return true; */
/* } */

static bool uart_pipe_init(void)
{
    tsrb_init(&_console_tx_pipe.rb, console_tx_rb_buf, HAL_UART_TX_STACK_SIZE);
    cond_init(&_console_tx_pipe.cond);
    mutex_init(&_console_tx_pipe.lock);

    return true;
}

static bool thread_tx_init(void)
{
    //tsrb_init(&_console_tx_rb, console_tx_rb_buf, HAL_UART_TX_STACK_SIZE);
    if (uart_pipe_init() == false) {
        DEBUG("Could not init uart pipe\n");
        return false;
    }
    __attribute__((unused))kernel_pid_t pid = thread_create(console_tx_thread_stack,
                                     sizeof(console_tx_thread_stack),
                                     HAL_UART_TX_THREAD_PRIORITY,
                                     THREAD_CREATE_WOUT_YIELD,
                                     uart_tx_thread,
                                     NULL,
                                     "console_tx_thread");
    DEBUG("\nuart tx pid = %d, prio = %d\n", pid, HAL_UART_TX_THREAD_PRIORITY);
    if (pid < 0) {
        DEBUG("Could not create UART TX thread\n");
        return false;
    }

    return true;
}

/* static void rx_timer_tick(void) */
/* { */

/* } */

/* static void *uart_rx_thread(__attribute((unused))void *arg) */
/* { */
/*     while (true) { */
/*         xtimer_usleep(1000); */
/*         rx_timer_tick(); */
/*     } */
/*     return NULL; */
/* } */

/* static bool thread_rx_init(void) */
/* { */
/*     kernel_pid_t pid = thread_create(uart_rx_thread_stack, */
/*                                      sizeof(uart_rx_thread_stack), */
/*                                      UART_RX_THREAD_PRIORITY, */
/*                                      THREAD_CREATE_WOUT_YIELD, */
/*                                      uart_rx_thread, */
/*                                      NULL, */
/*                                      "uart rx thread"); */
/*     if (pid > 0) { */
/*         DEBUG("gps uart rx thread pid = %d, priority = %d\n", pid, */
/*               UART_RX_THREAD_PRIORITY); */
/*         return true; */
/*     } */
/*     return false; */
/* } */

bool serial_init(void)
{
    bool ret = true;
    uart_init(HAL_UART_DEV, HAL_UART_BAUD, NULL, NULL);
    //ret = thread_rx_init();
    if (ret == false) {
        return false;
    }
    ret = thread_tx_init();
    return ret;
}
