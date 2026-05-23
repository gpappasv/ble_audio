// --- includes ----------------------------------------------------------------
#include "ble_bap_unicast_server.h"

#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/bap_lc3_preset.h>
#include <zephyr/bluetooth/audio/pacs.h>
#include <zephyr/logging/log.h>
#include "lc3.h"

// --- logging settings --------------------------------------------------------
LOG_MODULE_DECLARE(ble_m);

// --- defines -----------------------------------------------------------------
#define MAX_SAMPLE_RATE       48000
#define MAX_FRAME_DURATION_US 10000
#define MAX_NUM_SAMPLES       ((MAX_FRAME_DURATION_US * MAX_SAMPLE_RATE) / USEC_PER_SEC)

static const struct bt_audio_codec_cap lc3_codec_cap
    = BT_AUDIO_CODEC_CAP_LC3(BT_AUDIO_CODEC_CAP_FREQ_ANY,
                             BT_AUDIO_CODEC_CAP_DURATION_7_5 | BT_AUDIO_CODEC_CAP_DURATION_10,
                             BT_AUDIO_CODEC_CAP_CHAN_COUNT_SUPPORT(1),
                             40u,
                             120u,
                             1u,
                             (BT_AUDIO_CONTEXT_TYPE_CONVERSATIONAL | BT_AUDIO_CONTEXT_TYPE_MEDIA));

static const struct bt_audio_codec_qos_pref qos_pref
    = BT_AUDIO_CODEC_QOS_PREF(true, BT_GAP_LE_PHY_2M, 0x02, 10, 10000, 60000, 10000, 60000);

// --- static variables definitions --------------------------------------------
static struct bt_bap_stream sink_streams[CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT];

static struct bt_bap_unicast_server_register_param param
    = { .snk_cnt = CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT, .src_cnt = 0 };

static struct bt_pacs_cap cap_sink = {
    .codec_cap = &lc3_codec_cap,
};

// Isolated state tracking per active stream channel
static int16_t               audio_buf[CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT][MAX_NUM_SAMPLES];
static lc3_decoder_t         lc3_decoders[CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT];
static lc3_decoder_mem_48k_t lc3_decoder_mems[CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT];
static int                   frames_per_sdu[CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT];
static int                   octets_per_frame[CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT]; // Explicit cached size tracking

// --- forward declarations ----------------------------------------------------
static int                   get_stream_idx(struct bt_bap_stream *stream);
static struct bt_bap_stream *stream_alloc(enum bt_audio_dir dir);
static enum bt_audio_dir     stream_dir(const struct bt_bap_stream *stream);

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

    LOG_INF("ASE Configured: stream %p", *stream);
    *pref = qos_pref;

    int idx = get_stream_idx(*stream);
    if (idx >= 0)
    {
        lc3_decoders[idx] = NULL;
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

    // Cache the hardware limits for the real-time decoder loop
    frames_per_sdu[idx]   = bt_audio_codec_cfg_get_frame_blocks_per_sdu(stream->codec_cfg, true);
    octets_per_frame[idx] = len_ret;

    // Print out the exact parameters selected by the S24 Ultra
    LOG_INF("Enable Stream [%d]: %d Hz, %d us, %d Octets/Frame, %d Frames/SDU",
            idx,
            freq,
            frame_duration_us,
            octets_per_frame[idx],
            frames_per_sdu[idx]);

    lc3_decoders[idx] = lc3_setup_decoder(frame_duration_us, freq, freq, &lc3_decoder_mems[idx]);
    if (lc3_decoders[idx] == NULL)
    {
        LOG_ERR("Failed to instantiate context for decoder context instance %d", idx);
        *rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_INVALID, BT_BAP_ASCS_REASON_CODEC_DATA);
        return -1;
    }

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
static int
lc3_stop(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
    return 0;
}
static int
lc3_release(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
    return 0;
}

// --- Real-time Isochronous Data Paths ----------------------------------------
static void
stream_recv(struct bt_bap_stream *stream, const struct bt_iso_recv_info *info, struct net_buf *buf)
{
    int idx = get_stream_idx(stream);
    if (idx < 0 || lc3_decoders[idx] == NULL || info == NULL || buf == NULL)
    {
        return;
    }

    // Explicit validation: ensure the network buffer has enough bytes to decode
    if (buf->len < (octets_per_frame[idx] * frames_per_sdu[idx]))
    {
        return;
    }

    const uint8_t *in_buf = (info->flags & BT_ISO_FLAGS_VALID) ? buf->data : NULL;
    int            offset = 0;

    for (int i = 0; i < frames_per_sdu[idx]; i++)
    {
        // Use the explicitly cached configuration size rather than calculating from raw buffer len
        int err = lc3_decode(lc3_decoders[idx],
                             in_buf ? (in_buf + offset) : NULL,
                             octets_per_frame[idx],
                             LC3_PCM_FORMAT_S16,
                             audio_buf[idx],
                             1);

        if (in_buf)
        {
            offset += octets_per_frame[idx];
        }

        if (err < 0)
        {
            LOG_WRN("Decoder [%d] math failure on frame step %d (Passed Size: %d)", idx, i, octets_per_frame[idx]);
        }
    }
}

static void
stream_stopped(struct bt_bap_stream *stream, uint8_t reason)
{
    LOG_INF("Audio Stream %p stopped: reason 0x%02X", stream, reason);
}

static void
stream_started(struct bt_bap_stream *stream)
{
    LOG_INF("Audio Stream %p streaming link active", stream);
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