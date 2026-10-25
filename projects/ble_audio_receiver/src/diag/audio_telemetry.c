#include "audio_telemetry.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(audio_telemetry, LOG_LEVEL_INF);

#define TELEMETRY_INTERVAL_SEC 30

static atomic_t cnt_valid_sdu   = ATOMIC_INIT(0);
static atomic_t cnt_plc_sdu     = ATOMIC_INIT(0);
static atomic_t cnt_silence_sdu = ATOMIC_INIT(0);
static atomic_t cnt_seq_gaps    = ATOMIC_INIT(0);
static atomic_t cnt_underruns   = ATOMIC_INIT(0);
static atomic_t cnt_drift_slips = ATOMIC_INIT(0);
static atomic_t cnt_drift_pads  = ATOMIC_INIT(0);

static atomic_t q_min_blocks = ATOMIC_INIT(999);
static atomic_t q_max_blocks = ATOMIC_INIT(0);
static bool     is_active    = false;

static struct k_work_delayable report_work;

static void
reset_window_counters(void)
{
    atomic_clear(&cnt_valid_sdu);
    atomic_clear(&cnt_plc_sdu);
    atomic_clear(&cnt_silence_sdu);
    atomic_clear(&cnt_seq_gaps);
    atomic_clear(&cnt_underruns);
    atomic_clear(&cnt_drift_slips);
    atomic_clear(&cnt_drift_pads);
    atomic_set(&q_min_blocks, 999);
    atomic_set(&q_max_blocks, 0);
}

static void
report_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (!is_active)
    {
        return;
    }

    long valid   = atomic_set(&cnt_valid_sdu, 0);
    long plc     = atomic_set(&cnt_plc_sdu, 0);
    long silence = atomic_set(&cnt_silence_sdu, 0);
    long gaps    = atomic_set(&cnt_seq_gaps, 0);
    long under   = atomic_set(&cnt_underruns, 0);
    long slips   = atomic_set(&cnt_drift_slips, 0);
    long pads    = atomic_set(&cnt_drift_pads, 0);
    long q_min   = atomic_set(&q_min_blocks, 999);
    long q_max   = atomic_set(&q_max_blocks, 0);

    if (q_min == 999)
    {
        q_min = 0;
    }

    long total        = valid + plc + silence;
    long plc_permille = (total > 0) ? ((plc * 1000L) / total) : 0;

    LOG_INF(
        "[STREAM HEALTH %us] SDUs: %ld valid, %ld silence, %ld PLC (%ld.%01ld%%, gaps: %ld) | "
        "DMA Q: min %ld, max %ld | Drift: -%ld/+%ld samples | Underruns: %ld",
        TELEMETRY_INTERVAL_SEC,
        valid,
        silence,
        plc,
        plc_permille / 10L,
        plc_permille % 10L,
        gaps,
        q_min,
        q_max,
        slips,
        pads,
        under);

    k_work_reschedule(&report_work, K_SECONDS(TELEMETRY_INTERVAL_SEC));
}

void
audio_telemetry_init(void)
{
    reset_window_counters();
    k_work_init_delayable(&report_work, report_work_handler);
}

void
audio_telemetry_set_active(bool streaming_active)
{
    if (streaming_active && !is_active)
    {
        is_active = true;
        reset_window_counters();
        k_work_reschedule(&report_work, K_SECONDS(TELEMETRY_INTERVAL_SEC));
    }
    else if (!streaming_active && is_active)
    {
        is_active = false;
        k_work_cancel_delayable(&report_work);
    }
}

void
audio_telemetry_record_sdu(bool valid_data, bool is_plc, uint16_t seq_gap)
{
    if (seq_gap > 0U)
    {
        atomic_inc(&cnt_seq_gaps);
    }

    if (is_plc)
    {
        atomic_inc(&cnt_plc_sdu);
    }
    else if (valid_data)
    {
        atomic_inc(&cnt_valid_sdu);
    }
    else
    {
        atomic_inc(&cnt_silence_sdu);
    }
}

void
audio_telemetry_record_queue_depth(uint32_t used_blocks)
{
    long cur_min = atomic_get(&q_min_blocks);
    if ((long)used_blocks < cur_min)
    {
        atomic_set(&q_min_blocks, (atomic_val_t)used_blocks);
    }

    long cur_max = atomic_get(&q_max_blocks);
    if ((long)used_blocks > cur_max)
    {
        atomic_set(&q_max_blocks, (atomic_val_t)used_blocks);
    }
}

void
audio_telemetry_record_underrun(void)
{
    atomic_inc(&cnt_underruns);
}

void
audio_telemetry_record_drift_correction(bool is_slip)
{
    if (is_slip)
    {
        atomic_inc(&cnt_drift_slips);
    }
    else
    {
        atomic_inc(&cnt_drift_pads);
    }
}