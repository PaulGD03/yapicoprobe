/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef CDC_UART_SWD_H
#define CDC_UART_SWD_H

#include <stdbool.h>
#include <stdint.h>
#include "tusb.h"

#if OPT_TARGET_UART_SWD

typedef struct
{
    bool host_connected;
    bool target_attached;
    bool swd_lock_held;
} cdc_uartswd_status_t;

void cdc_uartswd_init(uint32_t task_prio);

void cdc_uartswd_line_state_cb(bool dtr, bool rts);
void cdc_uartswd_line_coding_cb(cdc_line_coding_t const *line_coding);
void cdc_uartswd_tx_complete_cb(void);
void cdc_uartswd_rx_cb(void);

void cdc_uartswd_get_status(cdc_uartswd_status_t *status);
void cdc_uartswd_print_status(void);
bool cdc_uartswd_debug_logging_enabled(void);
void cdc_uartswd_set_debug_logging(bool enable);
bool cdc_uartswd_manual_connect(void);
/**
 * Attempt a manual connect. If `reset_first` is true the function will request the
 * target to RESET_RUN before trying to attach (useful for RAM-loaded helpers).
 */
bool cdc_uartswd_manual_connect_force(bool reset_first);

/* deep debug: FR/DR register dumps */
bool cdc_uartswd_deep_logging_enabled(void);
void cdc_uartswd_set_deep_logging(bool enable);

#endif

#endif
