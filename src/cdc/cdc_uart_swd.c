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

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "event_groups.h"
#include "stream_buffer.h"
#include "task.h"

#include "tusb.h"

#include "picoprobe_config.h"
#include "led.h"
#include "sw_lock.h"
#include "cdc_uart_swd.h"

#include "target_board.h"
#include "swd_host.h"

#if OPT_TARGET_UART_SWD

#ifndef PROBE_SWDUART_BASE_ADDR
#define PROBE_SWDUART_BASE_ADDR (0x40038000u) // RP2040 UART1 base
#endif

#define RP2040_UART_DR_OFFSET 0x000u
#define RP2040_UART_FR_OFFSET 0x018u

#define UART_FR_TXFF (1u << 5)
#define UART_FR_RXFE (1u << 4)

#define STREAM_TO_HOST_SIZE 4096
#define STREAM_TO_HOST_TRIGGER 32
#define STREAM_TO_TARGET_SIZE 1024
#define STREAM_TO_TARGET_TRIGGER 1

#define SWD_UART_RX_BURST 260
#define SWD_UART_TX_BURST 260

#define EV_TX_COMPLETE 0x01
#define EV_STREAM 0x02
#define EV_RX 0x04
#define EV_STATE 0x08

typedef enum
{
    UART_SWD_RESULT_NO_DATA = 0,
    UART_SWD_RESULT_DATA,
    UART_SWD_RESULT_ERROR
} uart_swd_result_t;

static inline uint32_t swd_uart_reg(uint32_t offset)
{
    return PROBE_SWDUART_BASE_ADDR + offset;
}

static TaskHandle_t task_cdc = NULL;
static TaskHandle_t task_swd = NULL;
static StreamBufferHandle_t stream_to_host;
static StreamBufferHandle_t stream_to_target;
static EventGroupHandle_t events;
static volatile bool m_connected = false;
static volatile bool m_target_attached = false;
static volatile bool m_swd_lock_held = false;
static volatile bool m_debug_logging = false;
/* deep logging: FR/DR register dumps (very noisy) */
static volatile bool m_deep_logging = false;

static bool tx_hold_valid = false;
static uint8_t tx_hold_byte = 0;
static const char swd_uart_banner[] =
    "\r\n[YAPicoprobe SWD UART bridge]\r\n"
    "Target UART via SWD ready.\r\n";

#define UART_SWD_DEBUG_PREVIEW 32

static void uartswd_debug_dump(const char *prefix, const uint8_t *data, size_t len)
{
    if (!m_debug_logging || data == NULL || len == 0)
    {
        return;
    }

    char buf[(UART_SWD_DEBUG_PREVIEW * 3) + 20];
    size_t preview = MIN(len, (size_t)UART_SWD_DEBUG_PREVIEW);
    size_t pos = 0;
    for (size_t i = 0; i < preview && pos < sizeof(buf); ++i)
    {
        pos += snprintf(buf + pos, sizeof(buf) - pos, "%02x ", data[i]);
    }
    if (len > preview && pos < sizeof(buf))
    {
        snprintf(buf + pos, sizeof(buf) - pos, "... (+%u)", (unsigned)(len - preview));
    }

    picoprobe_info("SWD-UART %s (%u byte%s): %s\n",
                   prefix,
                   (unsigned)len,
                   (len == 1) ? "" : "s",
                   buf);
}

static uart_swd_result_t swd_uart_try_read(uint8_t *value)
{
    uint32_t flags = 0;
    if (!swd_read_word(swd_uart_reg(RP2040_UART_FR_OFFSET), &flags))
    {
        return UART_SWD_RESULT_ERROR;
    }
    if (m_deep_logging)
    {
        picoprobe_info("SWD-UART: FR=0x%08lx\n", (unsigned long)flags);
    }
    if (flags & UART_FR_RXFE)
    {
        return UART_SWD_RESULT_NO_DATA;
    }

    uint32_t data = 0;
    if (!swd_read_word(swd_uart_reg(RP2040_UART_DR_OFFSET), &data))
    {
        return UART_SWD_RESULT_ERROR;
    }

    if (m_deep_logging && (data == 0))
    {
        picoprobe_info("SWD-UART: DR==0x00 (read zero)\n");
    }

    *value = (uint8_t)(data & 0xffu);
    return UART_SWD_RESULT_DATA;
}

static uart_swd_result_t swd_uart_try_write(uint8_t value)
{
    uint32_t flags = 0;
    if (!swd_read_word(swd_uart_reg(RP2040_UART_FR_OFFSET), &flags))
    {
        return UART_SWD_RESULT_ERROR;
    }
    if (flags & UART_FR_TXFF)
    {
        return UART_SWD_RESULT_NO_DATA;
    }

    /* RP2040 UART DR is a 32-bit register; prefer write_word for efficiency. */
    uint32_t write_value = (uint32_t)value;
    if (!swd_write_word(swd_uart_reg(RP2040_UART_DR_OFFSET), write_value))
    {
        /* fallback: write memory as 4 bytes */
        if (!swd_write_memory(swd_uart_reg(RP2040_UART_DR_OFFSET), (uint8_t *)&write_value, sizeof(write_value)))
        {
            return UART_SWD_RESULT_ERROR;
        }
    }

    return UART_SWD_RESULT_DATA;
}

static uint32_t stream_push_to_host(const uint8_t *data, size_t len)
{
    if (stream_to_host == NULL)
    {
        return 0;
    }

    if (!m_connected)
    {
        size_t available = xStreamBufferSpacesAvailable(stream_to_host);
        while (available < len)
        {
            uint8_t dummy[32];
            size_t removed = xStreamBufferReceive(stream_to_host, dummy, sizeof(dummy), 0);
            if (removed == 0)
            {
                break;
            }
            available += removed;
        }
    }

    return xStreamBufferSend(stream_to_host, data, len, 0);
}

static bool swd_uart_receive_into_stream(bool *fatal_error)
{
    bool progress = false;
    *fatal_error = false;
    uint8_t debug_buf[SWD_UART_RX_BURST];
    size_t debug_cnt = 0;

    for (uint32_t n = 0; n < SWD_UART_RX_BURST; ++n)
    {
        uint8_t ch = 0;
        uart_swd_result_t res = swd_uart_try_read(&ch);
        if (res == UART_SWD_RESULT_NO_DATA)
        {
            break;
        }
        if (res == UART_SWD_RESULT_ERROR)
        {
            *fatal_error = true;
            break;
        }

        stream_push_to_host(&ch, sizeof(ch));
        if (m_debug_logging && debug_cnt < sizeof(debug_buf))
        {
            debug_buf[debug_cnt++] = ch;
        }
        progress = true;
    }

    if (debug_cnt != 0)
    {
        uartswd_debug_dump("RX target->host", debug_buf, debug_cnt);
    }

    if (progress)
    {
        led_state(LS_UART_RX_DATA);
        xEventGroupSetBits(events, EV_STREAM);
    }

    return progress;
}

static bool swd_uart_transmit_from_stream(bool *fatal_error)
{
    bool progress = false;
    *fatal_error = false;

    if (stream_to_target == NULL)
    {
        return false;
    }

    for (uint32_t n = 0; n < SWD_UART_TX_BURST; ++n)
    {
        if (!tx_hold_valid)
        {
            if (xStreamBufferReceive(stream_to_target, &tx_hold_byte, sizeof(tx_hold_byte), 0) != sizeof(tx_hold_byte))
            {
                break;
            }
            tx_hold_valid = true;
        }

        uart_swd_result_t res = swd_uart_try_write(tx_hold_byte);
        if (res == UART_SWD_RESULT_NO_DATA)
        {
            break;
        }
        if (res == UART_SWD_RESULT_ERROR)
        {
            *fatal_error = true;
            break;
        }

        tx_hold_valid = false;
        progress = true;
        led_state(LS_UART_TX_DATA);
    }

    return progress;
}

static void swd_thread(void *ptr)
{
    (void)ptr;
    m_target_attached = false;

    for (;;)
    {
        if (!m_connected)
        {
            m_target_attached = false;
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (!sw_lock("UART-SWD", false))
        {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        m_swd_lock_held = true;

        if (!m_target_attached)
        {
            m_target_attached = target_set_state(ATTACH);
            if (!m_target_attached)
            {
                sw_unlock("UART-SWD");
                m_swd_lock_held = false;
                vTaskDelay(pdMS_TO_TICKS(200));
                continue;
            }
        }

        for (;;)
        {
            if (!m_connected || sw_unlock_requested())
            {
                break;
            }

            bool fatal_rx = false;
            bool fatal_tx = false;
            bool rx_work = swd_uart_receive_into_stream(&fatal_rx);
            bool tx_work = swd_uart_transmit_from_stream(&fatal_tx);

            if (fatal_rx || fatal_tx)
            {
                m_target_attached = false;
                break;
            }

            if (!(rx_work || tx_work))
            {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }

        m_swd_lock_held = false;
        sw_unlock("UART-SWD");
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void cdc_uartswd_get_status(cdc_uartswd_status_t *status)
{
    if (status == NULL)
    {
        return;
    }

    status->host_connected = m_connected;
    status->target_attached = m_target_attached;
    status->swd_lock_held = m_swd_lock_held;
}

void cdc_uartswd_print_status(void)
{
    cdc_uartswd_status_t status;
    cdc_uartswd_get_status(&status);

    picoprobe_info("SWD-UART bridge: host %s, target %s, swd lock %s\n",
                   status.host_connected ? "connected" : "idle",
                   status.target_attached ? "attached" : "detached",
                   status.swd_lock_held ? "held" : "released");

    if (!status.host_connected)
    {
        picoprobe_info("  Open the CDC-SWD-UART COM port (assert DTR/RTS) to activate the bridge.\n");
    }
    else if (!status.target_attached)
    {
        picoprobe_info("  Waiting for SWD access / target UART init.\n");
    }

    picoprobe_info("  Debug logging: %s\n", m_debug_logging ? "on" : "off");
}

bool cdc_uartswd_debug_logging_enabled(void)
{
    return m_debug_logging;
}

void cdc_uartswd_set_debug_logging(bool enable)
{
    m_debug_logging = enable;
    picoprobe_info("SWD-UART debug logging %s\n", enable ? "enabled" : "disabled");
}

void cdc_uartswd_set_deep_logging(bool enable)
{
    m_deep_logging = enable;
    picoprobe_info("SWD-UART deep logging %s\n", enable ? "enabled" : "disabled");
}

bool cdc_uartswd_deep_logging_enabled(void)
{
    return m_deep_logging;
}

bool cdc_uartswd_manual_connect(void)
{
    if (!sw_lock("UART-SWD-MANUAL", true))
    {
        const char *owner = sw_lock_owner();
        if (owner != NULL)
        {
            picoprobe_error("SWD-UART: unable to grab SWD lock for manual connect (held by %s)\n", owner);
        }
        else
        {
            picoprobe_error("SWD-UART: unable to grab SWD lock for manual connect\n");
        }
        return false;
    }

    m_swd_lock_held = true;
    bool ok = target_set_state(ATTACH);
    m_target_attached = ok;
    sw_unlock("UART-SWD-MANUAL");
    m_swd_lock_held = false;

    picoprobe_info("SWD-UART manual connect %s\n", ok ? "succeeded" : "failed");
    return ok;
}

bool cdc_uartswd_manual_connect_force(bool reset_first)
{
    if (!sw_lock("UART-SWD-MANUAL", true))
    {
        const char *owner = sw_lock_owner();
        if (owner != NULL)
        {
            picoprobe_error("SWD-UART: unable to grab SWD lock for manual connect (held by %s)\n", owner);
        }
        else
        {
            picoprobe_error("SWD-UART: unable to grab SWD lock for manual connect\n");
        }
        return false;
    }

    m_swd_lock_held = true;

    if (reset_first)
    {
        picoprobe_info("SWD-UART: requesting RESET_RUN to wake target\n");
        (void)target_set_state(RESET_RUN);
        /* small delay to allow CPU to start and peripherals to initialize */
        /* allow more time for RAM-loaded helpers or slow attach sequences */
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    /* try attach multiple times; RAM-loaded helpers may need a bit longer */
    bool ok = false;
    /* progressive retry strategy: more attempts with modest backoff */
    const int max_attempts = 12;
    for (int attempt = 1; attempt <= max_attempts; ++attempt)
    {
        picoprobe_info("SWD-UART: attach attempt %d/%d\n", attempt, max_attempts);
        ok = target_set_state(ATTACH);
        if (ok)
        {
            picoprobe_info("SWD-UART: attach succeeded on attempt %d\n", attempt);
            break;
        }
        /* if requested, pulse reset/run again to nudge the target */
        if (reset_first)
        {
            (void)target_set_state(RESET_RUN);
        }
        /* backoff: 100ms then gradually up to 1000ms */
        int backoff_ms = 100 + (attempt * 100);
        if (backoff_ms > 1000)
        {
            backoff_ms = 1000;
        }
        vTaskDelay(pdMS_TO_TICKS(backoff_ms));
    }

    m_target_attached = ok;

    sw_unlock("UART-SWD-MANUAL");
    m_swd_lock_held = false;

    picoprobe_info("SWD-UART manual connect %s\n", ok ? "succeeded" : "failed");
    return ok;
}

static void cdc_thread(void *ptr)
{
    (void)ptr;
    static uint8_t cdc_tx_buf[CFG_TUD_CDC_TX_BUFSIZE];

    for (;;)
    {
        if (stream_to_host == NULL || stream_to_target == NULL)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (!m_connected)
        {
            xEventGroupWaitBits(events, EV_STATE, pdTRUE, pdFALSE, pdMS_TO_TICKS(1000));
            continue;
        }

        uint32_t cdc_rx_chars = tud_cdc_n_available(CDC_UART_SWD_N);
        if (cdc_rx_chars == 0 && xStreamBufferIsEmpty(stream_to_host))
        {
            tud_cdc_n_write_flush(CDC_UART_SWD_N);
            xEventGroupWaitBits(events, EV_TX_COMPLETE | EV_STREAM | EV_RX | EV_STATE, pdTRUE, pdFALSE, pdMS_TO_TICKS(100));
        }

        if (!xStreamBufferIsEmpty(stream_to_host))
        {
            size_t max_cnt = tud_cdc_n_write_available(CDC_UART_SWD_N);
            if (max_cnt != 0)
            {
                max_cnt = MIN(sizeof(cdc_tx_buf), max_cnt);
                size_t cnt = xStreamBufferReceive(stream_to_host, cdc_tx_buf, max_cnt, 0);
                if (cnt != 0)
                {
                    tud_cdc_n_write(CDC_UART_SWD_N, cdc_tx_buf, cnt);
                }
            }
        }
        else
        {
            tud_cdc_n_write_flush(CDC_UART_SWD_N);
        }

        if (cdc_rx_chars != 0)
        {
            uint8_t rx_buf[64];
            size_t chunk = tud_cdc_n_read(CDC_UART_SWD_N, rx_buf, MIN(sizeof(rx_buf), cdc_rx_chars));
            if (chunk != 0)
            {
                if (stream_to_target != NULL)
                {
                    size_t sent = xStreamBufferSend(stream_to_target, rx_buf, chunk, pdMS_TO_TICKS(20));
                    if (sent != 0)
                    {
                        uartswd_debug_dump("TX host->target", rx_buf, sent);
                    }
                }
                xEventGroupSetBits(events, EV_RX);
            }
        }
    }
}

void cdc_uartswd_line_state_cb(bool dtr, bool rts)
{
    tud_cdc_n_write_clear(CDC_UART_SWD_N);
    tud_cdc_n_read_flush(CDC_UART_SWD_N);

    bool was_connected = m_connected;
    m_connected = (dtr || rts);
    if (!m_connected)
    {
        tx_hold_valid = false;
        xStreamBufferReset(stream_to_target);
    }
    else if (!was_connected && stream_to_host != NULL)
    {
        stream_push_to_host((const uint8_t *)swd_uart_banner, sizeof(swd_uart_banner) - 1);
        xEventGroupSetBits(events, EV_STREAM);
    }

    if (was_connected != m_connected)
    {
        xEventGroupSetBits(events, EV_STATE);
    }

    xEventGroupSetBits(events, EV_TX_COMPLETE);
}

void cdc_uartswd_line_coding_cb(cdc_line_coding_t const *line_coding)
{
    (void)line_coding;
}

void cdc_uartswd_tx_complete_cb(void)
{
    xEventGroupSetBits(events, EV_TX_COMPLETE);
}

void cdc_uartswd_rx_cb(void)
{
    xEventGroupSetBits(events, EV_RX);
}

void cdc_uartswd_init(uint32_t task_prio)
{
    events = xEventGroupCreate();

    stream_to_host = xStreamBufferCreate(STREAM_TO_HOST_SIZE, STREAM_TO_HOST_TRIGGER);
    if (stream_to_host == NULL)
    {
        picoprobe_error("cdc_uartswd_init: cannot create stream_to_host\n");
    }

    stream_to_target = xStreamBufferCreate(STREAM_TO_TARGET_SIZE, STREAM_TO_TARGET_TRIGGER);
    if (stream_to_target == NULL)
    {
        picoprobe_error("cdc_uartswd_init: cannot create stream_to_target\n");
    }

    UBaseType_t swd_task_prio = (task_prio > (tskIDLE_PRIORITY + 1)) ? (task_prio - 1) : task_prio;

    xTaskCreate(cdc_thread, "CDC-UARTSWD", configMINIMAL_STACK_SIZE, NULL, task_prio, &task_cdc);
    xTaskCreate(swd_thread, "SWD-UART", configMINIMAL_STACK_SIZE, NULL, swd_task_prio, &task_swd);

    cdc_uartswd_line_state_cb(false, false);
}

#endif // OPT_TARGET_UART_SWD
