#include "ble_bap_unicast_server.h"
#include "ble_conn_control.h"
#include "../dac/audio_i2s.h"

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/bap_lc3_preset.h>
#include <zephyr/bluetooth/audio/pacs.h>
#include <zephyr/bluetooth/audio/vcp.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include "lc3.h"

LOG_MODULE_REGISTER(ble_unicast_server, LOG_LEVEL_INF);

#define MAX_SAMPLE_RATE       48000
#define MAX_FRAME_DURATION_US 10000
#define MAX_NUM_SAMPLES       ((MAX_FRAME_DURATION_US * MAX_SAMPLE_RATE) / USEC_PER_SEC)
#define MAX_AUDIO_CHANNELS    2
#define MAX_SDU_BUF_SIZE      248
#define MAX_SINK_STREAMS      (CONFIG_BT_MAX_CONN * CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT)

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
    uint16_t len;
    uint8_t  data[MAX_SDU_BUF_SIZE];
};

K_MSGQ_DEFINE(lc3_rx_msgq, sizeof(struct lc3_sdu_packet), 16, 4);

/* Strictly 48 kHz, 10 ms duration, 1 or 2 channels, 40..120 octets (fits in single 251B LL PDU) */
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

static struct bt_bap_stream sink_streams[MAX_SINK_STREAMS];

static struct bt_bap_unicast_server_register_param param
    = { .snk_cnt = CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT, .src_cnt = 0 };

static struct bt_pacs_cap cap_sink = {
    .codec_cap = &lc3_codec_cap,
};

static lc3_decoder_t         lc3_decoders[MAX_AUDIO_CHANNELS];
static lc3_decoder_mem_48k_t lc3_decoder_mems[MAX_AUDIO_CHANNELS];
static int16_t               pcm_left_buf[MAX_NUM_SAMPLES];
static int16_t               pcm_right_buf[MAX_NUM_SAMPLES];
static bool                  left_staged  = false;
static bool                  right_staged = false;

static bool     stream_configured[MAX_SINK_STREAMS];
static bool     stream_streaming[MAX_SINK_STREAMS];
static int      frames_per_sdu[MAX_SINK_STREAMS];
static int      octets_per_frame[MAX_SINK_STREAMS];
static int      chan_cnt_per_stream[MAX_SINK_STREAMS];
static bool     is_right_stream[MAX_SINK_STREAMS];
static size_t   samples_per_frame[MAX_SINK_STREAMS];
static uint16_t expected_seq_num[MAX_SINK_STREAMS];
static bool     seq_initialized[MAX_SINK_STREAMS];

static int                   get_stream_idx(struct bt_bap_stream *stream);
static struct bt_bap_stream *stream_alloc(enum bt_audio_dir dir);
static enum bt_audio_dir     stream_dir(const struct bt_bap_stream *stream);

static void
vcs_state_cb(struct bt_conn *conn, int err, uint8_t volume, uint8_t mute)
{
    if (err)
    {
        LOG_ERR("VCS state callback error: %d", err);
        return;
    }

    if (conn != NULL && !ble_conn_control_is_active_conn(conn))
    {
        return;
    }

    bool is_muted = (mute == BT_VCP_STATE_MUTED);
    audio_i2s_set_volume(volume, is_muted);
    LOG_INF("Volume updated: %u / 255 (%u%%), Mute: %s",
            volume,
            (unsigned int)((volume * 100U) / 255U),
            is_muted ? "YES" : "NO");
}

static void
vcs_flags_cb(struct bt_conn *conn, int err, uint8_t flags)
{
    ARG_UNUSED(conn);
    if (err)
    {
        LOG_ERR("VCS flags callback error: %d", err);
    }
}

static struct bt_vcp_vol_rend_cb vcp_cbs = {
    .state = vcs_state_cb,
    .flags = vcs_flags_cb,
};

static void
init_vcp_volume_renderer(void)
{
    struct bt_vcp_vol_rend_register_param vcp_param;
    memset(&vcp_param, 0, sizeof(vcp_param));

    vcp_param.step   = 1;
    vcp_param.mute   = BT_VCP_STATE_UNMUTED;
    vcp_param.volume = 200;
    vcp_param.cb     = &vcp_cbs;

    audio_i2s_set_volume(vcp_param.volume, false);

    int err = bt_vcp_vol_rend_register(&vcp_param);
    if (err)
    {
        LOG_ERR("Failed to register VCP Volume Renderer (err %d)", err);
    }
    else
    {
        LOG_INF("VCP Volume Renderer registered (initial volume: %u/255)", vcp_param.volume);
    }
}

static int
count_configured_streams(void)
{
    int cnt = 0;
    for (int i = 0; i < MAX_SINK_STREAMS; i++)
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
    for (int i = 0; i < MAX_SINK_STREAMS; i++)
    {
        if (stream_streaming[i])
        {
            cnt++;
        }
    }
    return cnt;
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
        if (idx >= MAX_SINK_STREAMS || !stream_streaming[idx])
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

        /* CASE 1: Single-CIS Multiplexed Stereo (2 channels packed in 1 SDU) */
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

        /* CASE 2: Dual-CIS Split Stereo (both Left and Right CIS links actively streaming) */
        if (count_streaming_streams() >= 2 && lc3_decoders[0] != NULL && lc3_decoders[1] != NULL)
        {
            bool right_ch = is_right_stream[idx];

            for (int b = 0; b < f_blocks; b++)
            {
                const uint8_t *frame_ptr = pkt.data + offset;
                offset += octets;

                if (!right_ch)
                {
                    if (left_staged)
                    {
                        decode_single_channel(
                            lc3_decoders[1], SDU_STATUS_LOST_PLC, NULL, octets, pcm_right_buf, n_samples);
                        audio_i2s_write_stereo_frame(pcm_left_buf, pcm_right_buf, n_samples);
                        left_staged  = false;
                        right_staged = false;
                    }

                    decode_single_channel(lc3_decoders[0], pkt.status, frame_ptr, octets, pcm_left_buf, n_samples);
                    left_staged = true;
                }
                else
                {
                    if (right_staged)
                    {
                        decode_single_channel(
                            lc3_decoders[0], SDU_STATUS_LOST_PLC, NULL, octets, pcm_left_buf, n_samples);
                        audio_i2s_write_stereo_frame(pcm_left_buf, pcm_right_buf, n_samples);
                        left_staged  = false;
                        right_staged = false;
                    }

                    decode_single_channel(lc3_decoders[1], pkt.status, frame_ptr, octets, pcm_right_buf, n_samples);
                    right_staged = true;
                }

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
        stream_configured[idx] = false;
        stream_streaming[idx]  = false;
        seq_initialized[idx]   = false;
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
        is_right_stream[idx]     = has_right ? true : (has_left ? false : ((idx % 2) == 1));
    }

    int blocks             = bt_audio_codec_cfg_get_frame_blocks_per_sdu(stream->codec_cfg, true);
    frames_per_sdu[idx]    = (blocks > 0) ? blocks : 1;
    octets_per_frame[idx]  = len_ret;
    samples_per_frame[idx] = (size_t)((frame_duration_us * freq) / USEC_PER_SEC);

    if (samples_per_frame[idx] > MAX_NUM_SAMPLES)
    {
        samples_per_frame[idx] = MAX_NUM_SAMPLES;
    }

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

    seq_initialized[idx]   = false;
    stream_configured[idx] = true;
    stream_streaming[idx]  = false;

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
        stream_configured[idx]   = false;
        stream_streaming[idx]    = false;
        frames_per_sdu[idx]      = 0;
        octets_per_frame[idx]    = 0;
        chan_cnt_per_stream[idx] = 0;
        samples_per_frame[idx]   = 0;
        seq_initialized[idx]     = false;

        left_staged  = false;
        right_staged = false;

        if (count_configured_streams() == 0)
        {
            lc3_decoders[0] = NULL;
            lc3_decoders[1] = NULL;
            k_msgq_purge(&lc3_rx_msgq);
        }
        LOG_INF("Reset decoder context for stream index %d", idx);
    }
}

void
ble_bap_unicast_server_evict_conn(struct bt_conn *conn)
{
    if (conn == NULL)
    {
        return;
    }

    for (size_t i = 0; i < ARRAY_SIZE(sink_streams); i++)
    {
        if (sink_streams[i].conn == conn)
        {
            cleanup_stream_state(&sink_streams[i]);
        }
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

static void
stream_recv(struct bt_bap_stream *stream, const struct bt_iso_recv_info *info, struct net_buf *buf)
{
    if (stream == NULL || !ble_conn_control_is_active_conn(stream->conn))
    {
        return;
    }

    int idx = get_stream_idx(stream);
    if (idx < 0 || idx >= MAX_SINK_STREAMS || !stream_streaming[idx] || info == NULL)
    {
        return;
    }

    if (seq_initialized[idx])
    {
        uint16_t gap = (uint16_t)(info->seq_num - expected_seq_num[idx]);
        if (gap > 0 && gap <= 4)
        {
            struct lc3_sdu_packet plc_pkt = {
                .stream_idx = (uint8_t)idx,
                .status     = SDU_STATUS_LOST_PLC,
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

    size_t required_len = (size_t)(octets_per_frame[idx] * frames_per_sdu[idx] * chan_cnt_per_stream[idx]);
    bool   flag_valid   = (info->flags & BT_ISO_FLAGS_VALID) != 0;

    struct lc3_sdu_packet pkt;
    pkt.stream_idx = (uint8_t)idx;

    if (flag_valid && buf != NULL && buf->len >= required_len && required_len > 0 && required_len <= MAX_SDU_BUF_SIZE)
    {
        pkt.status = SDU_STATUS_VALID_DATA;
        pkt.len    = (uint16_t)required_len;
        memcpy(pkt.data, buf->data, required_len);
    }
    else if (flag_valid && (buf == NULL || buf->len == 0))
    {
        /* Intentional 0-length keepalive SDU sent by phone during pause/silence */
        pkt.status = SDU_STATUS_SILENCE;
        pkt.len    = 0;
    }
    else
    {
        /* Corrupted or lost RF packet -> synthesize LC3 PLC */
        pkt.status = SDU_STATUS_LOST_PLC;
        pkt.len    = 0;
    }

    k_msgq_put(&lc3_rx_msgq, &pkt, K_NO_WAIT);
}

static void
stream_stopped(struct bt_bap_stream *stream, uint8_t reason)
{
    LOG_INF("Audio stream %p stopped (reason 0x%02X)", stream, reason);
    cleanup_stream_state(stream);
    if (count_streaming_streams() == 0)
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
        seq_initialized[idx]  = false;
        stream_streaming[idx] = true;
        left_staged           = false;
        right_staged          = false;
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

    init_vcp_volume_renderer();
}