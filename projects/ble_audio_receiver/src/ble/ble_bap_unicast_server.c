// --- includes ----------------------------------------------------------------
#include "ble_bap_unicast_server.h"

#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/pacs.h>

#include <zephyr/logging/log.h>

// --- logging settings --------------------------------------------------------
LOG_MODULE_DECLARE(ble_m);

// --- defines -----------------------------------------------------------------
static const struct bt_audio_codec_cap lc3_codec_cap
    = BT_AUDIO_CODEC_CAP_LC3(BT_AUDIO_CODEC_CAP_FREQ_ANY,
                             BT_AUDIO_CODEC_CAP_DURATION_10,
                             BT_AUDIO_CODEC_CAP_CHAN_COUNT_SUPPORT(1),
                             40u,
                             120u,
                             1u,
                             (BT_AUDIO_CONTEXT_TYPE_CONVERSATIONAL | BT_AUDIO_CONTEXT_TYPE_MEDIA));
#define AUDIO_DATA_TIMEOUT_US 1000000UL /* Send data every 1 second */
#define SDU_INTERVAL_US       10000UL   /* 10 ms SDU interval */
static const struct bt_audio_codec_qos_pref qos_pref
    = BT_AUDIO_CODEC_QOS_PREF(true, BT_GAP_LE_PHY_2M, 0x02, 10, 40000, 40000, 40000, 40000);
NET_BUF_POOL_FIXED_DEFINE(tx_pool,
                          CONFIG_BT_ASCS_MAX_ASE_SRC_COUNT,
                          BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
                          CONFIG_BT_CONN_TX_USER_DATA_SIZE,
                          NULL);

// --- static functions declarations -------------------------------------------
static struct bt_bap_stream *stream_alloc(enum bt_audio_dir dir);

static int lc3_config(struct bt_conn                        *conn,
                      const struct bt_bap_ep                *ep,
                      enum bt_audio_dir                      dir,
                      const struct bt_audio_codec_cfg       *codec_cfg,
                      struct bt_bap_stream                 **stream,
                      struct bt_audio_codec_qos_pref * const pref,
                      struct bt_bap_ascs_rsp                *rsp);
static int lc3_reconfig(struct bt_bap_stream                  *stream,
                        enum bt_audio_dir                      dir,
                        const struct bt_audio_codec_cfg       *codec_cfg,
                        struct bt_audio_codec_qos_pref * const pref,
                        struct bt_bap_ascs_rsp                *rsp);
static int lc3_qos(struct bt_bap_stream *stream, const struct bt_audio_codec_qos *qos, struct bt_bap_ascs_rsp *rsp);
static int lc3_enable(struct bt_bap_stream *stream, const uint8_t meta[], size_t meta_len, struct bt_bap_ascs_rsp *rsp);
static int lc3_start(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp);
static int lc3_metadata(struct bt_bap_stream   *stream,
                        const uint8_t           meta[],
                        size_t                  meta_len,
                        struct bt_bap_ascs_rsp *rsp);
static int lc3_disable(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp);
static int lc3_stop(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp);
static int lc3_release(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp);

static enum bt_audio_dir stream_dir(const struct bt_bap_stream *stream);

#if defined(CONFIG_LIBLC3)
static void stream_recv_lc3_codec(struct bt_bap_stream          *stream,
                                  const struct bt_iso_recv_info *info,
                                  struct net_buf                *buf);
#else
static void stream_recv(struct bt_bap_stream *stream, const struct bt_iso_recv_info *info, struct net_buf *buf);
#endif
static void stream_stopped(struct bt_bap_stream *stream, uint8_t reason);
static void stream_started(struct bt_bap_stream *stream);
static void stream_enabled_cb(struct bt_bap_stream *stream);

static int set_location(void);
static int set_supported_contexts(void);
static int set_available_contexts(void);

// --- static variables definitions --------------------------------------------
static size_t                  configured_source_stream_count;
static struct k_work_delayable audio_send_work;
static struct bt_bap_stream    sink_streams[CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT];
static struct audio_source
{
    struct bt_bap_stream stream;
    uint16_t             seq_num;
    uint16_t             max_sdu;
    size_t               len_to_send;
} source_streams[CONFIG_BT_ASCS_MAX_ASE_SRC_COUNT];

static struct bt_bap_unicast_server_register_param param
    = { CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT, CONFIG_BT_ASCS_MAX_ASE_SRC_COUNT };

static struct bt_pacs_cap cap_sink = {
    .codec_cap = &lc3_codec_cap,
};

static struct bt_pacs_cap cap_source = {
    .codec_cap = &lc3_codec_cap,
};
#if defined(CONFIG_LIBLC3)

#include "lc3.h"

#define MAX_SAMPLE_RATE       48000
#define MAX_FRAME_DURATION_US 10000
#define MAX_NUM_SAMPLES       ((MAX_FRAME_DURATION_US * MAX_SAMPLE_RATE) / USEC_PER_SEC)
static int16_t               audio_buf[MAX_NUM_SAMPLES];
static lc3_decoder_t         lc3_decoder;
static lc3_decoder_mem_48k_t lc3_decoder_mem;
static int                   frames_per_sdu;

#endif
static struct bt_bap_stream_ops stream_ops = {
#if defined(CONFIG_LIBLC3)
    .recv = stream_recv_lc3_codec,
#else
    .recv = stream_recv,
#endif
    .stopped = stream_stopped,
    .started = stream_started,
    .enabled = stream_enabled_cb,
};

// --- static functions definitions --------------------------------------------
static struct bt_bap_stream *
stream_alloc(enum bt_audio_dir dir)
{
    if (dir == BT_AUDIO_DIR_SOURCE)
    {
        for (size_t i = 0; i < ARRAY_SIZE(source_streams); i++)
        {
            struct bt_bap_stream *stream = &source_streams[i].stream;

            if (!stream->conn)
            {
                return stream;
            }
        }
    }
    else
    {
        for (size_t i = 0; i < ARRAY_SIZE(sink_streams); i++)
        {
            struct bt_bap_stream *stream = &sink_streams[i];

            if (!stream->conn)
            {
                return stream;
            }
        }
    }

    return NULL;
}

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
        LOG_ERR("No streams available\n");
        *rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_NO_MEM, BT_BAP_ASCS_REASON_NONE);

        return -ENOMEM;
    }

    LOG_INF("ASE Codec Config stream %p\n", *stream);

    if (dir == BT_AUDIO_DIR_SOURCE)
    {
        configured_source_stream_count++;
    }

    *pref = qos_pref;

#if defined(CONFIG_LIBLC3)
    /* Nothing to free as static memory is used */
    lc3_decoder = NULL;
#endif

    return 0;
}

static int
lc3_reconfig(struct bt_bap_stream                  *stream,
             enum bt_audio_dir                      dir,
             const struct bt_audio_codec_cfg       *codec_cfg,
             struct bt_audio_codec_qos_pref * const pref,
             struct bt_bap_ascs_rsp                *rsp)
{
    /* We only support one QoS at the moment, reject changes */
    LOG_WRN("QoS cannot be changed");
    return -ENOEXEC;
}

static int
lc3_qos(struct bt_bap_stream *stream, const struct bt_audio_codec_qos *qos, struct bt_bap_ascs_rsp *rsp)
{
    LOG_INF("QoS: stream %p qos %p\n", stream, qos);
    LOG_INF(
        "QoS: interval %u framing 0x%02x phy 0x%02x sdu %u "
        "rtn %u latency %u pd %u\n",
        qos->interval,
        qos->framing,
        qos->phy,
        qos->sdu,
        qos->rtn,
        qos->latency,
        qos->pd);
    for (size_t i = 0U; i < configured_source_stream_count; i++)
    {
        if (stream == &source_streams[i].stream)
        {
            source_streams[i].max_sdu = qos->sdu;
            break;
        }
    }
    return 0;
}

static int
lc3_enable(struct bt_bap_stream *stream, const uint8_t meta[], size_t meta_len, struct bt_bap_ascs_rsp *rsp)
{
    LOG_INF("Enable: stream %p meta_len %zu\n", stream, meta_len);

#if defined(CONFIG_LIBLC3)
    {
        int frame_duration_us;
        int freq;
        int ret;

        ret = bt_audio_codec_cfg_get_freq(stream->codec_cfg);
        if (ret > 0)
        {
            freq = bt_audio_codec_cfg_freq_to_freq_hz(ret);
        }
        else
        {
            LOG_ERR("Error: Codec frequency not set, cannot start codec.");
            *rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_INVALID, BT_BAP_ASCS_REASON_CODEC_DATA);
            return ret;
        }

        ret = bt_audio_codec_cfg_get_frame_dur(stream->codec_cfg);
        if (ret > 0)
        {
            frame_duration_us = bt_audio_codec_cfg_frame_dur_to_frame_dur_us(ret);
        }
        else
        {
            LOG_ERR("Error: Frame duration not set, cannot start codec.");
            *rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_INVALID, BT_BAP_ASCS_REASON_CODEC_DATA);
            return ret;
        }

        frames_per_sdu = bt_audio_codec_cfg_get_frame_blocks_per_sdu(stream->codec_cfg, true);

        lc3_decoder = lc3_setup_decoder(frame_duration_us,
                                        freq,
                                        0, /* No resampling */
                                        &lc3_decoder_mem);

        if (lc3_decoder == NULL)
        {
            LOG_ERR("ERROR: Failed to setup LC3 encoder - wrong parameters?\n");
            *rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_INVALID, BT_BAP_ASCS_REASON_CODEC_DATA);
            return -1;
        }
    }
#endif

    return 0;
}

static int
lc3_start(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
    printk("Start: stream %p\n", stream);

    for (size_t i = 0U; i < configured_source_stream_count; i++)
    {
        if (stream == &source_streams[i].stream)
        {
            source_streams[i].seq_num     = 0U;
            source_streams[i].len_to_send = 0U;
            break;
        }
    }

    if (configured_source_stream_count > 0 && !k_work_delayable_is_pending(&audio_send_work))
    {

        /* Start send timer */
        k_work_schedule(&audio_send_work, K_MSEC(0));
    }

    return 0;
}

static int
lc3_metadata(struct bt_bap_stream *stream, const uint8_t meta[], size_t meta_len, struct bt_bap_ascs_rsp *rsp)
{
    LOG_INF("lc3_metadata was just invoked");
    return 0;
}

static int
lc3_disable(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
    LOG_INF("Disable: stream %p\n", stream);
    return 0;
}

static int
lc3_stop(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
    LOG_INF("Stop: stream %p\n", stream);
    return 0;
}

static int
lc3_release(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
    LOG_INF("Release: stream %p\n", stream);
    return 0;
}

static enum bt_audio_dir
stream_dir(const struct bt_bap_stream *stream)
{
    for (size_t i = 0U; i < ARRAY_SIZE(source_streams); i++)
    {
        if (stream == &source_streams[i].stream)
        {
            return BT_AUDIO_DIR_SOURCE;
        }
    }

    for (size_t i = 0U; i < ARRAY_SIZE(sink_streams); i++)
    {
        if (stream == &sink_streams[i])
        {
            return BT_AUDIO_DIR_SINK;
        }
    }

    __ASSERT(false, "Invalid stream %p", stream);
    return 0;
}

#if defined(CONFIG_LIBLC3)
static void
stream_recv_lc3_codec(struct bt_bap_stream *stream, const struct bt_iso_recv_info *info, struct net_buf *buf)
{
    const uint8_t *in_buf;
    uint8_t        err              = -1;
    const int      octets_per_frame = buf->len / frames_per_sdu;

    if (lc3_decoder == NULL)
    {
        LOG_WRN("LC3 decoder not setup, cannot decode data.\n");
        return;
    }

    if ((info->flags & BT_ISO_FLAGS_VALID) == 0)
    {
        LOG_WRN("Bad packet: 0x%02X\n", info->flags);

        in_buf = NULL;
    }
    else
    {
        in_buf = buf->data;
    }

    /* This code is to demonstrate the use of the LC3 codec. On an actual implementation
     * it might be required to offload the processing to another task to avoid blocking the
     * BT stack.
     */
    for (int i = 0; i < frames_per_sdu; i++)
    {

        int offset = 0;

        err = lc3_decode(lc3_decoder, in_buf + offset, octets_per_frame, LC3_PCM_FORMAT_S16, audio_buf, 1);

        if (in_buf != NULL)
        {
            offset += octets_per_frame;
        }
    }

    LOG_INF("RX stream %p len %u\n", stream, buf->len);

    if (err == 1)
    {
        LOG_INF("  decoder performed PLC\n");
        return;
    }
    else if (err < 0)
    {
        LOG_WRN("  decoder failed - wrong parameters?\n");
        return;
    }
}
#else
static void
stream_recv(struct bt_bap_stream *stream, const struct bt_iso_recv_info *info, struct net_buf *buf)
{
    if (info->flags & BT_ISO_FLAGS_VALID)
    {
        LOG_INF("Incoming audio on stream %p len %u\n", stream, buf->len);
    }
}
#endif

static void
stream_stopped(struct bt_bap_stream *stream, uint8_t reason)
{
    LOG_INF("Audio Stream %p stopped with reason 0x%02X\n", stream, reason);

    /* Stop send timer */
    k_work_cancel_delayable(&audio_send_work);
}

static void
stream_started(struct bt_bap_stream *stream)
{
    LOG_INF("Audio Stream %p started\n", stream);
}

static void
stream_enabled_cb(struct bt_bap_stream *stream)
{
    /* The unicast server is responsible for starting sink ASEs after the
     * client has enabled them.
     */
    if (stream_dir(stream) == BT_AUDIO_DIR_SINK)
    {
        const int err = bt_bap_stream_start(stream);

        if (err != 0)
        {
            LOG_WRN("Failed to start stream %p: %d", stream, err);
        }
    }
}

static int
set_location(void)
{
    int err;

    if (IS_ENABLED(CONFIG_BT_PAC_SNK_LOC))
    {
        err = bt_pacs_set_location(BT_AUDIO_DIR_SINK, BT_AUDIO_LOCATION_FRONT_CENTER);
        if (err != 0)
        {
            printk("Failed to set sink location (err %d)\n", err);
            return err;
        }
    }

    if (IS_ENABLED(CONFIG_BT_PAC_SRC_LOC))
    {
        err = bt_pacs_set_location(BT_AUDIO_DIR_SOURCE, (BT_AUDIO_LOCATION_FRONT_LEFT | BT_AUDIO_LOCATION_FRONT_RIGHT));
        if (err != 0)
        {
            printk("Failed to set source location (err %d)\n", err);
            return err;
        }
    }

    printk("Location successfully set\n");

    return 0;
}

static int
set_supported_contexts(void)
{
    int err;

    if (IS_ENABLED(CONFIG_BT_PAC_SNK))
    {
        err = bt_pacs_set_supported_contexts(BT_AUDIO_DIR_SINK, AVAILABLE_SINK_CONTEXT);
        if (err != 0)
        {
            printk("Failed to set sink supported contexts (err %d)\n", err);

            return err;
        }
    }

    if (IS_ENABLED(CONFIG_BT_PAC_SRC))
    {
        err = bt_pacs_set_supported_contexts(BT_AUDIO_DIR_SOURCE, AVAILABLE_SOURCE_CONTEXT);
        if (err != 0)
        {
            printk("Failed to set source supported contexts (err %d)\n", err);

            return err;
        }
    }

    printk("Supported contexts successfully set\n");

    return 0;
}

static int
set_available_contexts(void)
{
    int err;

    if (IS_ENABLED(CONFIG_BT_PAC_SNK))
    {
        err = bt_pacs_set_available_contexts(BT_AUDIO_DIR_SINK, AVAILABLE_SINK_CONTEXT);
        if (err != 0)
        {
            printk("Failed to set sink available contexts (err %d)\n", err);
            return err;
        }
    }

    if (IS_ENABLED(CONFIG_BT_PAC_SRC))
    {
        err = bt_pacs_set_available_contexts(BT_AUDIO_DIR_SOURCE, AVAILABLE_SOURCE_CONTEXT);
        if (err != 0)
        {
            printk("Failed to set source available contexts (err %d)\n", err);
            return err;
        }
    }

    printk("Available contexts successfully set\n");
    return 0;
}

// --- Connection Callbacks ----------------------------------------------------
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

// --- Functions Definitions ---------------------------------------------------
void
ble_bap_unicast_server_start(void)
{
    int err;
    bt_bap_unicast_server_register(&param);
    bt_bap_unicast_server_register_cb(&unicast_server_cb);

    bt_pacs_cap_register(BT_AUDIO_DIR_SINK, &cap_sink);
    bt_pacs_cap_register(BT_AUDIO_DIR_SOURCE, &cap_source);

    for (size_t i = 0; i < ARRAY_SIZE(sink_streams); i++)
    {
        bt_bap_stream_cb_register(&sink_streams[i], &stream_ops);
    }

    for (size_t i = 0; i < ARRAY_SIZE(source_streams); i++)
    {
        bt_bap_stream_cb_register(&source_streams[i].stream, &stream_ops);
    }

    err = set_location();
    if (err != 0)
    {
        return;
    }

    err = set_supported_contexts();
    if (err != 0)
    {
        return;
    }

    err = set_available_contexts();
    if (err != 0)
    {
        return;
    }
}