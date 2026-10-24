// --- includes ----------------------------------------------------------------
#include "ble_bap_unicast_server.h"

#include "../dac/audio_i2s.h"

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/bap_lc3_preset.h>
#include <zephyr/bluetooth/audio/pacs.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include "lc3.h"

// --- logging settings --------------------------------------------------------
LOG_MODULE_REGISTER(ble_unicast_server, LOG_LEVEL_INF);

// --- defines -----------------------------------------------------------------
#define MAX_SAMPLE_RATE       48000
#define MAX_FRAME_DURATION_US 10000
#define MAX_NUM_SAMPLES       ((MAX_FRAME_DURATION_US * MAX_SAMPLE_RATE) / USEC_PER_SEC)
#define MAX_AUDIO_CHANNELS    2
#define MAX_SDU_BUF_SIZE      248

#define AUDIO_WORKER_STACK_SIZE 4096
#define AUDIO_WORKER_PRIORITY   K_PRIO_PREEMPT(2)

struct lc3_sdu_packet
{
    uint8_t  stream_idx;
    bool     valid;
    uint16_t len;
    uint8_t  data[MAX_SDU_BUF_SIZE];
};

K_MSGQ_DEFINE(lc3_rx_msgq, sizeof(struct lc3_sdu_packet), 16, 4);

/* Advertise support for both 1 and 2 channels per CIS, up to 2 frames per SDU */
static const struct bt_audio_codec_cap lc3_codec_cap = BT_AUDIO_CODEC_CAP_LC3(
    BT_AUDIO_CODEC_CAP_FREQ_48KHZ,
    BT_AUDIO_CODEC_CAP_DURATION_10,
    BT_AUDIO_CODEC_CAP_CHAN_COUNT_SUPPORT(1, 2),
    40u,
    120u,
    2u,
    (BT_AUDIO_CONTEXT_TYPE_MEDIA | BT_AUDIO_CONTEXT_TYPE_CONVERSATIONAL | BT_AUDIO_CONTEXT_TYPE_GAME));

static const struct bt_audio_codec_qos_pref qos_pref
    = BT_AUDIO_CODEC_QOS_PREF(true, BT_GAP_LE_PHY_2M, 0x02, 10, 10000, 60000, 10000, 60000);

// --- static variables definitions --------------------------------------------
static struct bt_bap_stream sink_streams[CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT];

static struct bt_bap_unicast_server_register_param param
    = { .snk_cnt = CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT, .src_cnt = 0 };

static struct bt_pacs_cap cap_sink = {
    .codec_cap = &lc3_codec_cap,
};

// Decoder instances indexed by physical audio channel: [0] = Left, [1] = Right
static lc3_decoder_t         lc3_decoders[MAX_AUDIO_CHANNELS];
static lc3_decoder_mem_48k_t lc3_decoder_mems[MAX_AUDIO_CHANNELS];
static int16_t               pcm_left_buf[MAX_NUM_SAMPLES];
static int16_t               pcm_right_buf[MAX_NUM_SAMPLES];
static bool                  left_staged  = false;
static bool                  right_staged = false;

// Per-ASE stream configuration state
static bool     stream_active[CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT];
static int      frames_per_sdu[CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT];
static int      octets_per_frame[CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT];
static int      chan_cnt_per_stream[CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT];
static bool     is_right_stream[CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT];
static size_t   samples_per_frame[CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT];
static uint16_t expected_seq_num[CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT];
static bool     seq_initialized[CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT];

// --- forward declarations ----------------------------------------------------
static int                   get_stream_idx(struct bt_bap_stream *stream);
static struct bt_bap_stream *stream_alloc(enum bt_audio_dir dir);
static enum bt_audio_dir     stream_dir(const struct bt_bap_stream *stream);

static int
count_active_streams(void)
{
    int cnt = 0;
    for (int i = 0; i < CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT; i++)
    {
        if (stream_active[i])
        {
            cnt++;
        }
    }
    return cnt;
}

// --- Dedicated Audio Decode & Stereo Pairing Worker Thread -------------------
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
        if (idx >= CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT || !stream_active[idx])
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

        const uint8_t *in_buf = pkt.valid ? pkt.data : NULL;
        int            offset = 0;

        /* CASE 1: Single-CIS Multiplexed Stereo (2 channels packed in 1 SDU) */
        if (chans == 2 && lc3_decoders[0] != NULL && lc3_decoders[1] != NULL)
        {
            for (int b = 0; b < f_blocks; b++)
            {
                const uint8_t *l_ptr = in_buf ? (in_buf + offset) : NULL;
                const uint8_t *r_ptr = in_buf ? (in_buf + offset + octets) : NULL;
                offset += (2 * octets);

                if (lc3_decode(lc3_decoders[0], l_ptr, octets, LC3_PCM_FORMAT_S16, pcm_left_buf, 1) < 0)
                {
                    memset(pcm_left_buf, 0, n_samples * sizeof(int16_t));
                }
                if (lc3_decode(lc3_decoders[1], r_ptr, octets, LC3_PCM_FORMAT_S16, pcm_right_buf, 1) < 0)
                {
                    memset(pcm_right_buf, 0, n_samples * sizeof(int16_t));
                }

                audio_i2s_write_stereo_frame(pcm_left_buf, pcm_right_buf, n_samples);
            }
            continue;
        }

        /* CASE 2: Dual-CIS Split Stereo (2 separate ASEs active, 1 channel each) */
        if (count_active_streams() >= 2 && lc3_decoders[0] != NULL && lc3_decoders[1] != NULL)
        {
            bool right_ch = is_right_stream[idx];

            for (int b = 0; b < f_blocks; b++)
            {
                const uint8_t *frame_ptr = in_buf ? (in_buf + offset) : NULL;
                offset += octets;

                if (!right_ch)
                {
                    /* If previous Left wasn't paired yet (Right dropped), conceal Right and flush */
                    if (left_staged)
                    {
                        lc3_decode(lc3_decoders[1], NULL, octets, LC3_PCM_FORMAT_S16, pcm_right_buf, 1);
                        audio_i2s_write_stereo_frame(pcm_left_buf, pcm_right_buf, n_samples);
                        left_staged  = false;
                        right_staged = false;
                    }

                    if (lc3_decode(lc3_decoders[0], frame_ptr, octets, LC3_PCM_FORMAT_S16, pcm_left_buf, 1) < 0)
                    {
                        memset(pcm_left_buf, 0, n_samples * sizeof(int16_t));
                    }
                    left_staged = true;
                }
                else
                {
                    /* If previous Right wasn't paired yet (Left dropped), conceal Left and flush */
                    if (right_staged)
                    {
                        lc3_decode(lc3_decoders[0], NULL, octets, LC3_PCM_FORMAT_S16, pcm_left_buf, 1);
                        audio_i2s_write_stereo_frame(pcm_left_buf, pcm_right_buf, n_samples);
                        left_staged  = false;
                        right_staged = false;
                    }

                    if (lc3_decode(lc3_decoders[1], frame_ptr, octets, LC3_PCM_FORMAT_S16, pcm_right_buf, 1) < 0)
                    {
                        memset(pcm_right_buf, 0, n_samples * sizeof(int16_t));
                    }
                    right_staged = true;
                }

                /* Both Left and Right decoded for this interval -> output interleaved stereo */
                if (left_staged && right_staged)
                {
                    audio_i2s_write_stereo_frame(pcm_left_buf, pcm_right_buf, n_samples);
                    left_staged  = false;
                    right_staged = false;
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
                const uint8_t *frame_ptr = in_buf ? (in_buf + offset) : NULL;
                offset += octets;

                if (lc3_decode(lc3_decoders[dec_idx], frame_ptr, octets, LC3_PCM_FORMAT_S16, pcm_left_buf, 1) < 0)
                {
                    memset(pcm_left_buf, 0, n_samples * sizeof(int16_t));
                }

                audio_i2s_write_stereo_frame(pcm_left_buf, pcm_left_buf, n_samples);
            }
        }
    }
}

K_THREAD_DEFINE(
    lc3_worker_tid, AUDIO_WORKER_STACK_SIZE, lc3_worker_thread_fn, NULL, NULL, NULL, AUDIO_WORKER_PRIORITY, 0, 0);

// --- BAP Unicast Server Callbacks --------------------------------------------
static int
lc3_config(struct bt_conn                        *conn,
           const struct bt_bap_ep                *ep,
           enum bt_audio_dir                      dir,
           const struct bt_audio_codec_cfg       *codec_cfg,
           struct bt_bap_stream                 **stream,
           struct bt_audio_codec_qos_pref * const pref,
           struct bt_bap_ascs_rsp                *rsp)
{
    *stream = stream_alloc(dir);
    if (*stream == NULL)
    {
        LOG_ERR("No structural streams available");
        *rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_NO_MEM, BT_BAP_ASCS_REASON_NONE);
        return -ENOMEM;
    }

    int idx = get_stream_idx(*stream);
    LOG_INF("ASE Configured: stream %p (idx %d)", *stream, idx);
    *pref = qos_pref;

    if (idx >= 0)
    {
        stream_active[idx]   = false;
        seq_initialized[idx] = false;
    }

    return 0;
}

static int
lc3_reconfig(struct bt_bap_stream                  *stream,
             enum bt_audio_dir                      dir,
             const struct bt_audio_codec_cfg       *codec_cfg,
             struct bt_audio_codec_qos_pref * const pref,
             struct bt_bap_ascs_rsp                *rsp)
{
    LOG_WRN("Dynamic Reconfig rejected");
    return -ENOEXEC;
}

static int
lc3_qos(struct bt_bap_stream *stream, const struct bt_audio_codec_qos *qos, struct bt_bap_ascs_rsp *rsp)
{
    LOG_INF("QoS Confirmed: stream %p, interval %u us, SDU %u", stream, qos->interval, qos->sdu);
    return 0;
}

static int
lc3_enable(struct bt_bap_stream *stream, const uint8_t meta[], size_t meta_len, struct bt_bap_ascs_rsp *rsp)
{
    int idx = get_stream_idx(stream);
    if (idx < 0)
    {
        return -EINVAL;
    }

    int freq_ret = bt_audio_codec_cfg_get_freq(stream->codec_cfg);
    int dur_ret  = bt_audio_codec_cfg_get_frame_dur(stream->codec_cfg);
    int len_ret  = bt_audio_codec_cfg_get_octets_per_frame(stream->codec_cfg);

    if (freq_ret <= 0 || dur_ret <= 0 || len_ret <= 0)
    {
        LOG_ERR("Invalid codec initialization parameters");
        *rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_INVALID, BT_BAP_ASCS_REASON_CODEC_DATA);
        return -EINVAL;
    }

    int freq              = bt_audio_codec_cfg_freq_to_freq_hz(freq_ret);
    int frame_duration_us = bt_audio_codec_cfg_frame_dur_to_frame_dur_us(dur_ret);

    enum bt_audio_location chan_alloc = BT_AUDIO_LOCATION_MONO_AUDIO;
    bt_audio_codec_cfg_get_chan_allocation(stream->codec_cfg, &chan_alloc, true);

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
        is_right_stream[idx]     = has_right ? true : (has_left ? false : (idx == 1));
    }

    int blocks             = bt_audio_codec_cfg_get_frame_blocks_per_sdu(stream->codec_cfg, true);
    frames_per_sdu[idx]    = (blocks > 0) ? blocks : 1;
    octets_per_frame[idx]  = len_ret;
    samples_per_frame[idx] = (size_t)((frame_duration_us * freq) / USEC_PER_SEC);

    if (samples_per_frame[idx] > MAX_NUM_SAMPLES)
    {
        samples_per_frame[idx] = MAX_NUM_SAMPLES;
    }

    /* Setup decoder context(s) required by this stream */
    if (chan_cnt_per_stream[idx] == 2)
    {
        lc3_decoders[0] = lc3_setup_decoder(frame_duration_us, freq, freq, &lc3_decoder_mems[0]);
        lc3_decoders[1] = lc3_setup_decoder(frame_duration_us, freq, freq, &lc3_decoder_mems[1]);
    }
    else
    {
        int dec_idx           = is_right_stream[idx] ? 1 : 0;
        lc3_decoders[dec_idx] = lc3_setup_decoder(frame_duration_us, freq, freq, &lc3_decoder_mems[dec_idx]);
    }

    seq_initialized[idx] = false;
    stream_active[idx]   = true;
    left_staged          = false;
    right_staged         = false;

    LOG_INF("Enable Stream [%d]: %d Hz, %d us, %d Octets/Frame, %d Chans (alloc 0x%04X, %s)",
            idx,
            freq,
            frame_duration_us,
            octets_per_frame[idx],
            chan_cnt_per_stream[idx],
            (unsigned int)chan_alloc,
            (chan_cnt_per_stream[idx] == 2) ? "STEREO-1CIS" : (is_right_stream[idx] ? "RIGHT" : "LEFT"));

    return 0;
}

static int
lc3_start(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
    return 0;
}

static int
lc3_metadata(struct bt_bap_stream *stream, const uint8_t meta[], size_t len, struct bt_bap_ascs_rsp *rsp)
{
    return 0;
}

static int
lc3_disable(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
    return 0;
}

static void
cleanup_stream_state(struct bt_bap_stream *stream)
{
    int idx = get_stream_idx(stream);
    if (idx >= 0)
    {
        stream_active[idx]       = false;
        frames_per_sdu[idx]      = 0;
        octets_per_frame[idx]    = 0;
        chan_cnt_per_stream[idx] = 0;
        samples_per_frame[idx]   = 0;
        seq_initialized[idx]     = false;

        if (count_active_streams() == 0)
        {
            lc3_decoders[0] = NULL;
            lc3_decoders[1] = NULL;
            left_staged     = false;
            right_staged    = false;
            k_msgq_purge(&lc3_rx_msgq);
        }
        LOG_INF("Reset decoder context for stream index %d", idx);
    }
}

static int
lc3_stop(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
    LOG_INF("ASE Stop requested: stream %p", stream);
    cleanup_stream_state(stream);
    return 0;
}

static int
lc3_release(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
    LOG_INF("ASE Release requested: stream %p", stream);
    cleanup_stream_state(stream);
    return 0;
}

// --- Real-time Isochronous Data Paths (Fast Producer) ------------------------
static void
stream_recv(struct bt_bap_stream *stream, const struct bt_iso_recv_info *info, struct net_buf *buf)
{
    int idx = get_stream_idx(stream);
    if (idx < 0 || idx >= CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT || !stream_active[idx] || info == NULL)
    {
        return;
    }

    /* 1. Detect skipped sequence numbers per stream and enqueue PLC packets */
    if (seq_initialized[idx])
    {
        uint16_t gap = (uint16_t)(info->seq_num - expected_seq_num[idx]);
        if (gap > 0 && gap <= 4)
        {
            struct lc3_sdu_packet plc_pkt = {
                .stream_idx = (uint8_t)idx,
                .valid      = false,
                .len        = 0,
            };
            for (uint16_t g = 0; g < gap; g++)
            {
                k_msgq_put(&lc3_rx_msgq, &plc_pkt, K_NO_WAIT);
            }
        }
    }
    expected_seq_num[idx] = (uint16_t)(info->seq_num + 1U);
    seq_initialized[idx]  = true;

    /* 2. Package SDU payload and push to worker queue without blocking BT RX thread */
    size_t required_len = (size_t)(octets_per_frame[idx] * frames_per_sdu[idx] * chan_cnt_per_stream[idx]);
    bool   valid        = (buf != NULL) && ((info->flags & BT_ISO_FLAGS_VALID) != 0) && (buf->len >= required_len)
                          && (required_len > 0) && (required_len <= MAX_SDU_BUF_SIZE);

    struct lc3_sdu_packet pkt;
    pkt.stream_idx = (uint8_t)idx;
    pkt.valid      = valid;

    if (valid)
    {
        pkt.len = (uint16_t)required_len;
        memcpy(pkt.data, buf->data, required_len);
    }
    else
    {
        pkt.len = 0;
    }

    k_msgq_put(&lc3_rx_msgq, &pkt, K_NO_WAIT);
}

static void
stream_stopped(struct bt_bap_stream *stream, uint8_t reason)
{
    LOG_INF("Audio stream %p stopped (reason 0x%02X)", stream, reason);
    cleanup_stream_state(stream);
    if (count_active_streams() == 0)
    {
        audio_i2s_stop();
    }
}

static void
stream_started(struct bt_bap_stream *stream)
{
    int idx = get_stream_idx(stream);
    if (idx >= 0)
    {
        seq_initialized[idx] = false;
    }
    LOG_INF("Audio Stream %p (idx %d) streaming link active", stream, idx);
}

static void
stream_enabled_cb(struct bt_bap_stream *stream)
{
    if (stream_dir(stream) == BT_AUDIO_DIR_SINK)
    {
        const int err = bt_bap_stream_start(stream);
        if (err != 0)
        {
            LOG_WRN("Failed auto-initiating stream %p: %d", stream, err);
        }
    }
}

// --- Internal Helper Routines ------------------------------------------------
static int
get_stream_idx(struct bt_bap_stream *stream)
{
    for (int i = 0; i < ARRAY_SIZE(sink_streams); i++)
    {
        if (stream == &sink_streams[i])
        {
            return i;
        }
    }
    return -1;
}

static struct bt_bap_stream *
stream_alloc(enum bt_audio_dir dir)
{
    if (dir == BT_AUDIO_DIR_SINK)
    {
        for (size_t i = 0; i < ARRAY_SIZE(sink_streams); i++)
        {
            if (!sink_streams[i].conn)
            {
                return &sink_streams[i];
            }
        }
    }
    return NULL;
}

static enum bt_audio_dir
stream_dir(const struct bt_bap_stream *stream)
{
    for (size_t i = 0; i < ARRAY_SIZE(sink_streams); i++)
    {
        if (stream == &sink_streams[i])
        {
            return BT_AUDIO_DIR_SINK;
        }
    }
    __ASSERT(false, "Unknown context stream tracking pointer");
    return 0;
}

static int
set_location(void)
{
    return bt_pacs_set_location(BT_AUDIO_DIR_SINK, BT_AUDIO_LOCATION_FRONT_LEFT | BT_AUDIO_LOCATION_FRONT_RIGHT);
}

static int
set_supported_contexts(void)
{
    return bt_pacs_set_supported_contexts(BT_AUDIO_DIR_SINK, AVAILABLE_SINK_CONTEXT);
}

static int
set_available_contexts(void)
{
    return bt_pacs_set_available_contexts(BT_AUDIO_DIR_SINK, AVAILABLE_SINK_CONTEXT);
}

// --- Structure Bindings ------------------------------------------------------
static const struct bt_bap_unicast_server_cb unicast_server_cb = {
    .config   = lc3_config,
    .reconfig = lc3_reconfig,
    .qos      = lc3_qos,
    .enable   = lc3_enable,
    .start    = lc3_start,
    .metadata = lc3_metadata,
    .disable  = lc3_disable,
    .stop     = lc3_stop,
    .release  = lc3_release,
};

static struct bt_bap_stream_ops stream_ops = {
    .recv    = stream_recv,
    .stopped = stream_stopped,
    .started = stream_started,
    .enabled = stream_enabled_cb,
};

// --- Core Public Interface ---------------------------------------------------
struct bt_bap_stream *
ble_bap_unicast_server_fetch_streams(void)
{
    return sink_streams;
}

void
ble_bap_unicast_server_start(void)
{
    bt_bap_unicast_server_register(&param);
    bt_bap_unicast_server_register_cb(&unicast_server_cb);
    bt_pacs_cap_register(BT_AUDIO_DIR_SINK, &cap_sink);

    for (size_t i = 0; i < ARRAY_SIZE(sink_streams); i++)
    {
        bt_bap_stream_cb_register(&sink_streams[i], &stream_ops);
    }

    if (set_location() != 0 || set_supported_contexts() != 0 || set_available_contexts() != 0)
    {
        LOG_ERR("PACS registration failure");
    }
}