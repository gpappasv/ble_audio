#include "ble_audio_stream.h"
#include "ble_conn_control.h"
#include "ble_bap_broadcast_sink.h"
#include "../dac/audio_i2s.h"
#include "../diag/audio_telemetry.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include "lc3.h"

LOG_MODULE_REGISTER(ble_audio_stream, LOG_LEVEL_INF);

#define DAC_SAMPLE_RATE       48000
#define MAX_FRAME_DURATION_US 10000
#define MAX_NUM_SAMPLES       ((MAX_FRAME_DURATION_US * DAC_SAMPLE_RATE) / USEC_PER_SEC)
#define MAX_AUDIO_CHANNELS    2
#define MAX_SDU_BUF_SIZE      248

/* Slots 0..1: Unicast Server | Slots 2..3: Broadcast Sink */
#define TOTAL_STREAM_SLOTS (BLE_AUDIO_MAX_UNICAST_STREAMS + BLE_AUDIO_MAX_BROADCAST_STREAMS)

#define AUDIO_WORKER_STACK_SIZE 4096
#define AUDIO_WORKER_PRIORITY   K_PRIO_PREEMPT(2)

enum sdu_frame_status
{
    SDU_STATUS_VALID_DATA = 0,
    SDU_STATUS_SILENCE    = 1,
    SDU_STATUS_LOST_PLC   = 2,
};

struct lc3_sdu_packet
{
    uint8_t  stream_idx;
    uint8_t  status;
    uint16_t seq_num;
    uint16_t len;
    uint8_t  data[MAX_SDU_BUF_SIZE];
};

K_MSGQ_DEFINE(lc3_rx_msgq, sizeof(struct lc3_sdu_packet), 16, 4);

static struct bt_bap_stream  sink_streams[TOTAL_STREAM_SLOTS];
static lc3_decoder_t         lc3_decoders[MAX_AUDIO_CHANNELS];
static lc3_decoder_mem_48k_t lc3_decoder_mems[MAX_AUDIO_CHANNELS];
static int16_t               pcm_left_buf[MAX_NUM_SAMPLES];
static int16_t               pcm_right_buf[MAX_NUM_SAMPLES];

/* Sequence-locked staging for dual-stream stereo alignment */
static bool     left_staged      = false;
static bool     right_staged     = false;
static uint16_t staged_left_seq  = 0;
static uint16_t staged_right_seq = 0;

static bool     stream_configured[TOTAL_STREAM_SLOTS];
static bool     stream_streaming[TOTAL_STREAM_SLOTS];
static int      frames_per_sdu[TOTAL_STREAM_SLOTS];
static int      octets_per_frame[TOTAL_STREAM_SLOTS];
static int      chan_cnt_per_stream[TOTAL_STREAM_SLOTS];
static bool     is_right_stream[TOTAL_STREAM_SLOTS];
static size_t   samples_per_frame[TOTAL_STREAM_SLOTS];
static uint16_t expected_seq_num[TOTAL_STREAM_SLOTS];
static bool     seq_initialized[TOTAL_STREAM_SLOTS];

static int
get_stream_idx(const struct bt_bap_stream *stream)
{
    for (int i = 0; i < TOTAL_STREAM_SLOTS; i++)
    {
        if (stream == &sink_streams[i])
        {
            return i;
        }
    }
    return -1;
}

static bool
is_broadcast_slot(int idx)
{
    return (idx >= BLE_AUDIO_MAX_UNICAST_STREAMS && idx < TOTAL_STREAM_SLOTS);
}

static int
count_configured_streams(void)
{
    int cnt = 0;
    for (int i = 0; i < TOTAL_STREAM_SLOTS; i++)
    {
        if (stream_configured[i])
        {
            cnt++;
        }
    }
    return cnt;
}

static int
count_streaming_streams(void)
{
    int cnt = 0;
    for (int i = 0; i < TOTAL_STREAM_SLOTS; i++)
    {
        if (stream_streaming[i])
        {
            cnt++;
        }
    }
    return cnt;
}

bool
ble_audio_stream_has_active_unicast(void)
{
    for (int i = 0; i < BLE_AUDIO_MAX_UNICAST_STREAMS; i++)
    {
        if (stream_configured[i] || stream_streaming[i])
        {
            return true;
        }
    }
    return false;
}

bool
ble_audio_stream_has_active_broadcast(void)
{
    for (int i = BLE_AUDIO_MAX_UNICAST_STREAMS; i < TOTAL_STREAM_SLOTS; i++)
    {
        if (stream_configured[i] || stream_streaming[i])
        {
            return true;
        }
    }
    return false;
}

static void
decode_single_channel(
    lc3_decoder_t dec, uint8_t status, const uint8_t *frame_ptr, int octets, int16_t *pcm_out, size_t n_samples)
{
    if (status == SDU_STATUS_SILENCE || dec == NULL)
    {
        memset(pcm_out, 0, n_samples * sizeof(int16_t));
        return;
    }

    const uint8_t *in_ptr = (status == SDU_STATUS_VALID_DATA) ? frame_ptr : NULL;
    if (lc3_decode(dec, in_ptr, octets, LC3_PCM_FORMAT_S16, pcm_out, 1) < 0)
    {
        memset(pcm_out, 0, n_samples * sizeof(int16_t));
    }
}

static void
lc3_worker_thread_fn(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    struct lc3_sdu_packet pkt;

    while (1)
    {
        if (k_msgq_get(&lc3_rx_msgq, &pkt, K_FOREVER) != 0)
        {
            continue;
        }

        uint8_t idx = pkt.stream_idx;
        if (idx >= TOTAL_STREAM_SLOTS || !stream_streaming[idx])
        {
            continue;
        }

        int    f_blocks  = frames_per_sdu[idx];
        int    octets    = octets_per_frame[idx];
        int    chans     = chan_cnt_per_stream[idx];
        size_t n_samples = samples_per_frame[idx];

        if (f_blocks <= 0 || octets <= 0 || n_samples == 0)
        {
            continue;
        }

        int offset = 0;

        /* CASE 1: Single-CIS / Single-BIS Multiplexed Stereo (2 channels in 1 SDU) */
        if (chans == 2 && lc3_decoders[0] != NULL && lc3_decoders[1] != NULL)
        {
            for (int b = 0; b < f_blocks; b++)
            {
                const uint8_t *l_ptr = pkt.data + offset;
                const uint8_t *r_ptr = pkt.data + offset + octets;
                offset += (2 * octets);

                decode_single_channel(lc3_decoders[0], pkt.status, l_ptr, octets, pcm_left_buf, n_samples);
                decode_single_channel(lc3_decoders[1], pkt.status, r_ptr, octets, pcm_right_buf, n_samples);

                audio_i2s_write_stereo_frame(pcm_left_buf, pcm_right_buf, n_samples);
            }
            continue;
        }

        /* CASE 2: Dual-CIS / Dual-BIS Split Stereo (Sequence-Number Locked) */
        if (count_streaming_streams() >= 2 && lc3_decoders[0] != NULL && lc3_decoders[1] != NULL)
        {
            bool right_ch = is_right_stream[idx];

            for (int b = 0; b < f_blocks; b++)
            {
                const uint8_t *frame_ptr = pkt.data + offset;
                offset += octets;

                if (!right_ch)
                {
                    /* Incoming Left channel packet */
                    if (left_staged)
                    {
                        /* A previous Left frame waited but its Right counterpart never arrived.
                         * Synthesize PLC for Right on the older sequence so audio does not stall. */
                        decode_single_channel(
                            lc3_decoders[1], SDU_STATUS_LOST_PLC, NULL, octets, pcm_right_buf, n_samples);
                        audio_i2s_write_stereo_frame(pcm_left_buf, pcm_right_buf, n_samples);
                        left_staged = false;
                    }

                    if (right_staged)
                    {
                        int16_t diff = (int16_t)(pkt.seq_num - staged_right_seq);
                        if (diff > 0)
                        {
                            /* The staged Right packet is older than this new Left packet.
                             * Flush the stale Right packet with PLC Left and clear it. */
                            decode_single_channel(
                                lc3_decoders[0], SDU_STATUS_LOST_PLC, NULL, octets, pcm_left_buf, n_samples);
                            audio_i2s_write_stereo_frame(pcm_left_buf, pcm_right_buf, n_samples);
                            right_staged = false;
                        }
                    }

                    decode_single_channel(lc3_decoders[0], pkt.status, frame_ptr, octets, pcm_left_buf, n_samples);
                    staged_left_seq = pkt.seq_num;
                    left_staged     = true;
                }
                else
                {
                    /* Incoming Right channel packet */
                    if (right_staged)
                    {
                        decode_single_channel(
                            lc3_decoders[0], SDU_STATUS_LOST_PLC, NULL, octets, pcm_left_buf, n_samples);
                        audio_i2s_write_stereo_frame(pcm_left_buf, pcm_right_buf, n_samples);
                        right_staged = false;
                    }

                    if (left_staged)
                    {
                        int16_t diff = (int16_t)(pkt.seq_num - staged_left_seq);
                        if (diff < 0)
                        {
                            /* This Right packet arrived late: its matching Left frame was already
                             * rendered. Discard it immediately so it never skews stereo timing. */
                            continue;
                        }
                        if (diff > 0)
                        {
                            /* Staged Left packet is older: flush it with PLC Right first */
                            decode_single_channel(
                                lc3_decoders[1], SDU_STATUS_LOST_PLC, NULL, octets, pcm_right_buf, n_samples);
                            audio_i2s_write_stereo_frame(pcm_left_buf, pcm_right_buf, n_samples);
                            left_staged = false;
                        }
                    }

                    decode_single_channel(lc3_decoders[1], pkt.status, frame_ptr, octets, pcm_right_buf, n_samples);
                    staged_right_seq = pkt.seq_num;
                    right_staged     = true;
                }

                /* Strict pairing: only output when sequence numbers match */
                if (left_staged && right_staged)
                {
                    if (staged_left_seq == staged_right_seq)
                    {
                        audio_i2s_write_stereo_frame(pcm_left_buf, pcm_right_buf, n_samples);
                        left_staged  = false;
                        right_staged = false;
                    }
                }
            }
            continue;
        }

        /* CASE 3: Single-Stream Mono Fallback */
        int dec_idx = is_right_stream[idx] ? 1 : 0;
        if (lc3_decoders[dec_idx] != NULL)
        {
            for (int b = 0; b < f_blocks; b++)
            {
                const uint8_t *frame_ptr = pkt.data + offset;
                offset += octets;

                decode_single_channel(lc3_decoders[dec_idx], pkt.status, frame_ptr, octets, pcm_left_buf, n_samples);
                audio_i2s_write_stereo_frame(pcm_left_buf, pcm_left_buf, n_samples);
            }
        }
    }
}

K_THREAD_DEFINE(
    lc3_worker_tid, AUDIO_WORKER_STACK_SIZE, lc3_worker_thread_fn, NULL, NULL, NULL, AUDIO_WORKER_PRIORITY, 0, 0);

int
ble_audio_stream_configure(struct bt_bap_stream *stream, const struct bt_audio_codec_cfg *codec_cfg)
{
    int idx = get_stream_idx(stream);
    if (idx < 0 || codec_cfg == NULL)
    {
        return -EINVAL;
    }

    int freq_ret = bt_audio_codec_cfg_get_freq(codec_cfg);
    int dur_ret  = bt_audio_codec_cfg_get_frame_dur(codec_cfg);
    int len_ret  = bt_audio_codec_cfg_get_octets_per_frame(codec_cfg);

    if (freq_ret <= 0 || dur_ret <= 0 || len_ret <= 0)
    {
        LOG_ERR("Invalid codec parameters on stream idx %d", idx);
        return -EINVAL;
    }

    int freq              = bt_audio_codec_cfg_freq_to_freq_hz(freq_ret);
    int frame_duration_us = bt_audio_codec_cfg_frame_dur_to_frame_dur_us(dur_ret);

    enum bt_audio_location chan_alloc = BT_AUDIO_LOCATION_MONO_AUDIO;
    bt_audio_codec_cfg_get_chan_allocation(codec_cfg, &chan_alloc, true);

    bool has_left  = (chan_alloc & BT_AUDIO_LOCATION_FRONT_LEFT) != 0;
    bool has_right = (chan_alloc & BT_AUDIO_LOCATION_FRONT_RIGHT) != 0;

    if (has_left && has_right)
    {
        chan_cnt_per_stream[idx] = 2;
        is_right_stream[idx]     = false;
    }
    else
    {
        chan_cnt_per_stream[idx] = 1;
        is_right_stream[idx]     = has_right ? true : (has_left ? false : ((idx % 2) == 1));
    }

    int blocks             = bt_audio_codec_cfg_get_frame_blocks_per_sdu(codec_cfg, true);
    frames_per_sdu[idx]    = (blocks > 0) ? blocks : 1;
    octets_per_frame[idx]  = len_ret;
    samples_per_frame[idx] = (size_t)((frame_duration_us * DAC_SAMPLE_RATE) / USEC_PER_SEC);

    if (samples_per_frame[idx] > MAX_NUM_SAMPLES)
    {
        samples_per_frame[idx] = MAX_NUM_SAMPLES;
    }

    if (chan_cnt_per_stream[idx] == 2)
    {
        lc3_decoders[0] = lc3_setup_decoder(frame_duration_us, freq, DAC_SAMPLE_RATE, &lc3_decoder_mems[0]);
        lc3_decoders[1] = lc3_setup_decoder(frame_duration_us, freq, DAC_SAMPLE_RATE, &lc3_decoder_mems[1]);
    }
    else
    {
        int dec_idx           = is_right_stream[idx] ? 1 : 0;
        lc3_decoders[dec_idx] = lc3_setup_decoder(frame_duration_us, freq, DAC_SAMPLE_RATE, &lc3_decoder_mems[dec_idx]);
    }

    seq_initialized[idx]   = false;
    stream_configured[idx] = true;

    LOG_INF("Configured %s Stream [%d]: %d Hz, %d us, %d Octets/Frame, %d Chans (alloc 0x%04X, %s)",
            is_broadcast_slot(idx) ? "Broadcast" : "Unicast",
            idx,
            freq,
            frame_duration_us,
            octets_per_frame[idx],
            chan_cnt_per_stream[idx],
            (unsigned int)chan_alloc,
            (chan_cnt_per_stream[idx] == 2) ? "STEREO-1STREAM" : (is_right_stream[idx] ? "RIGHT" : "LEFT"));

    return 0;
}

void
ble_audio_stream_cleanup(struct bt_bap_stream *stream)
{
    int idx = get_stream_idx(stream);
    if (idx < 0)
    {
        return;
    }

    bool was_active = stream_configured[idx] || stream_streaming[idx];

    stream_configured[idx]   = false;
    stream_streaming[idx]    = false;
    frames_per_sdu[idx]      = 0;
    octets_per_frame[idx]    = 0;
    chan_cnt_per_stream[idx] = 0;
    samples_per_frame[idx]   = 0;
    seq_initialized[idx]     = false;

    left_staged      = false;
    right_staged     = false;
    staged_left_seq  = 0;
    staged_right_seq = 0;

    if (count_configured_streams() == 0 && count_streaming_streams() == 0)
    {
        lc3_decoders[0] = NULL;
        lc3_decoders[1] = NULL;
        k_msgq_purge(&lc3_rx_msgq);
    }

    if (was_active)
    {
        LOG_INF("Reset decoder context for stream index %d", idx);
    }
}

void
ble_audio_stream_evict_conn(struct bt_conn *conn)
{
    if (conn == NULL)
    {
        return;
    }

    for (int i = 0; i < BLE_AUDIO_MAX_UNICAST_STREAMS; i++)
    {
        if (sink_streams[i].conn == conn)
        {
            ble_audio_stream_cleanup(&sink_streams[i]);
        }
    }
}

void
ble_audio_stream_reset_broadcast(void)
{
    for (int i = BLE_AUDIO_MAX_UNICAST_STREAMS; i < TOTAL_STREAM_SLOTS; i++)
    {
        ble_audio_stream_cleanup(&sink_streams[i]);
    }
}

static void
stream_recv(struct bt_bap_stream *stream, const struct bt_iso_recv_info *info, struct net_buf *buf)
{
    if (stream == NULL || info == NULL)
    {
        return;
    }

    int idx = get_stream_idx(stream);
    if (idx < 0 || idx >= TOTAL_STREAM_SLOTS || !stream_streaming[idx])
    {
        return;
    }

    /* Unicast guard: packet must belong to the active connection */
    if (!is_broadcast_slot(idx))
    {
        if (!ble_conn_control_is_active_conn(stream->conn))
        {
            return;
        }
    }
    else
    {
        /* Broadcast guard: ignore BIS packets if Unicast took over */
        if (ble_audio_stream_has_active_unicast())
        {
            return;
        }
    }

    uint16_t gap = 0U;
    if (seq_initialized[idx])
    {
        gap = (uint16_t)(info->seq_num - expected_seq_num[idx]);
        if (gap > 0 && gap <= 4)
        {
            struct lc3_sdu_packet plc_pkt = {
                .stream_idx = (uint8_t)idx,
                .status     = SDU_STATUS_LOST_PLC,
                .seq_num    = (uint16_t)(expected_seq_num[idx]),
                .len        = 0,
            };
            for (uint16_t g = 0; g < gap; g++)
            {
                plc_pkt.seq_num = (uint16_t)(expected_seq_num[idx] + g);
                k_msgq_put(&lc3_rx_msgq, &plc_pkt, K_NO_WAIT);
                audio_telemetry_record_sdu(false, true, (g == 0U) ? gap : 0U);
            }
        }
    }
    expected_seq_num[idx] = (uint16_t)(info->seq_num + 1U);
    seq_initialized[idx]  = true;

    size_t required_len = (size_t)(octets_per_frame[idx] * frames_per_sdu[idx] * chan_cnt_per_stream[idx]);
    bool   flag_valid   = (info->flags & BT_ISO_FLAGS_VALID) != 0;

    struct lc3_sdu_packet pkt;
    pkt.stream_idx = (uint8_t)idx;
    pkt.seq_num    = (uint16_t)info->seq_num;

    if (flag_valid && buf != NULL && buf->len >= required_len && required_len > 0 && required_len <= MAX_SDU_BUF_SIZE)
    {
        pkt.status = SDU_STATUS_VALID_DATA;
        pkt.len    = (uint16_t)required_len;
        memcpy(pkt.data, buf->data, required_len);
        audio_telemetry_record_sdu(true, false, 0U);
    }
    else if (flag_valid && (buf == NULL || buf->len == 0))
    {
        pkt.status = SDU_STATUS_SILENCE;
        pkt.len    = 0;
        audio_telemetry_record_sdu(false, false, 0U);
    }
    else
    {
        pkt.status = SDU_STATUS_LOST_PLC;
        pkt.len    = 0;
        audio_telemetry_record_sdu(false, true, 0U);
    }

    k_msgq_put(&lc3_rx_msgq, &pkt, K_NO_WAIT);
}

static void
stream_started(struct bt_bap_stream *stream)
{
    int idx = get_stream_idx(stream);
    if (idx < 0)
    {
        return;
    }

    if (!stream_configured[idx] && stream->codec_cfg != NULL)
    {
        if (ble_audio_stream_configure(stream, stream->codec_cfg) != 0)
        {
            LOG_ERR("Failed configuring stream %d on start", idx);
            return;
        }
    }

    seq_initialized[idx]  = false;
    stream_streaming[idx] = true;
    left_staged           = false;
    right_staged          = false;
    staged_left_seq       = 0;
    staged_right_seq      = 0;

    LOG_INF("Audio Stream %p (idx %d, %s) streaming link active",
            stream,
            idx,
            is_broadcast_slot(idx) ? "Broadcast" : "Unicast");
}

static void
stream_stopped(struct bt_bap_stream *stream, uint8_t reason)
{
    int idx = get_stream_idx(stream);
    LOG_INF("Audio stream %p (idx %d) stopped (reason 0x%02X)", stream, idx, reason);

    ble_audio_stream_cleanup(stream);

    if (count_streaming_streams() == 0)
    {
        audio_i2s_stop();
    }

    if (idx >= 0)
    {
        if (is_broadcast_slot(idx))
        {
            if (!ble_audio_stream_has_active_broadcast())
            {
                ble_bap_broadcast_sink_on_big_stopped();
            }
        }
        else if (!ble_audio_stream_has_active_unicast())
        {
            ble_bap_broadcast_sink_resume();
        }
    }
}

static void
stream_enabled_cb(struct bt_bap_stream *stream)
{
    int idx = get_stream_idx(stream);
    if (idx >= 0 && !is_broadcast_slot(idx))
    {
        const int err = bt_bap_stream_start(stream);
        if (err != 0)
        {
            LOG_WRN("Failed auto-initiating unicast stream %p: %d", stream, err);
        }
    }
}

static struct bt_bap_stream_ops shared_stream_ops = {
    .recv    = stream_recv,
    .started = stream_started,
    .stopped = stream_stopped,
    .enabled = stream_enabled_cb,
};

struct bt_bap_stream *
ble_audio_stream_alloc_unicast(void)
{
    for (int i = 0; i < BLE_AUDIO_MAX_UNICAST_STREAMS; i++)
    {
        if (!sink_streams[i].conn)
        {
            stream_configured[i] = false;
            stream_streaming[i]  = false;
            seq_initialized[i]   = false;
            return &sink_streams[i];
        }
    }
    return NULL;
}

void
ble_audio_stream_get_broadcast_streams(struct bt_bap_stream **streams, size_t max_streams)
{
    size_t count = MIN(max_streams, (size_t)BLE_AUDIO_MAX_BROADCAST_STREAMS);
    for (size_t i = 0; i < count; i++)
    {
        int slot                = BLE_AUDIO_MAX_UNICAST_STREAMS + (int)i;
        stream_configured[slot] = false;
        stream_streaming[slot]  = false;
        seq_initialized[slot]   = false;
        streams[i]              = &sink_streams[slot];
    }
}

void
ble_audio_stream_init(void)
{
    for (int i = 0; i < TOTAL_STREAM_SLOTS; i++)
    {
        bt_bap_stream_cb_register(&sink_streams[i], &shared_stream_ops);
    }
}