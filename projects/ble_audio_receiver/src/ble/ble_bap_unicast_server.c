#include "ble_bap_unicast_server.h"
#include "ble_audio_stream.h"
#include "ble_bap_broadcast_sink.h"
#include "ble_conn_control.h"
#include "../dac/audio_i2s.h"

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/pacs.h>
#include <zephyr/bluetooth/audio/vcp.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(ble_unicast_server, LOG_LEVEL_INF);

/* Supports 24 kHz (Auracast Standard Quality) and 48 kHz (Unicast & Auracast High Quality),
 * 7.5 ms & 10 ms durations, 1 or 2 channels, 30..120 octets. */
static const struct bt_audio_codec_cap lc3_codec_cap
    = BT_AUDIO_CODEC_CAP_LC3((BT_AUDIO_CODEC_CAP_FREQ_24KHZ | BT_AUDIO_CODEC_CAP_FREQ_48KHZ),
                             (BT_AUDIO_CODEC_CAP_DURATION_7_5 | BT_AUDIO_CODEC_CAP_DURATION_10),
                             BT_AUDIO_CODEC_CAP_CHAN_COUNT_SUPPORT(1, 2),
                             30u,
                             120u,
                             2u,
                             (BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED | BT_AUDIO_CONTEXT_TYPE_MEDIA
                              | BT_AUDIO_CONTEXT_TYPE_CONVERSATIONAL | BT_AUDIO_CONTEXT_TYPE_GAME));

static const struct bt_audio_codec_qos_pref qos_pref
    = BT_AUDIO_CODEC_QOS_PREF(true, BT_GAP_LE_PHY_2M, 0x02, 10, 10000, 60000, 10000, 60000);

static struct bt_bap_unicast_server_register_param param
    = { .snk_cnt = CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT, .src_cnt = 0 };

static struct bt_pacs_cap cap_sink = {
    .codec_cap = &lc3_codec_cap,
};

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
lc3_config(struct bt_conn                        *conn,
           const struct bt_bap_ep                *ep,
           enum bt_audio_dir                      dir,
           const struct bt_audio_codec_cfg       *codec_cfg,
           struct bt_bap_stream                 **stream,
           struct bt_audio_codec_qos_pref * const pref,
           struct bt_bap_ascs_rsp                *rsp)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(ep);
    ARG_UNUSED(codec_cfg);

    if (dir != BT_AUDIO_DIR_SINK)
    {
        return -EINVAL;
    }

    ble_bap_broadcast_sink_suspend();

    *stream = ble_audio_stream_alloc_unicast();
    if (*stream == NULL)
    {
        LOG_ERR("No structural unicast streams available");
        *rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_NO_MEM, BT_BAP_ASCS_REASON_NONE);
        return -ENOMEM;
    }

    LOG_INF("ASE Configured: stream %p", *stream);
    *pref = qos_pref;
    return 0;
}

static int
lc3_reconfig(struct bt_bap_stream                  *stream,
             enum bt_audio_dir                      dir,
             const struct bt_audio_codec_cfg       *codec_cfg,
             struct bt_audio_codec_qos_pref * const pref,
             struct bt_bap_ascs_rsp                *rsp)
{
    ARG_UNUSED(stream);
    ARG_UNUSED(dir);
    ARG_UNUSED(codec_cfg);
    ARG_UNUSED(pref);
    ARG_UNUSED(rsp);

    LOG_WRN("Dynamic Reconfig rejected");
    return -ENOEXEC;
}

static int
lc3_qos(struct bt_bap_stream *stream, const struct bt_audio_codec_qos *qos, struct bt_bap_ascs_rsp *rsp)
{
    ARG_UNUSED(rsp);
    LOG_INF("QoS Confirmed: stream %p, interval %u us, SDU %u", stream, qos->interval, qos->sdu);
    return 0;
}

static int
lc3_enable(struct bt_bap_stream *stream, const uint8_t meta[], size_t meta_len, struct bt_bap_ascs_rsp *rsp)
{
    ARG_UNUSED(meta);
    ARG_UNUSED(meta_len);

    ble_bap_broadcast_sink_suspend();

    int err = ble_audio_stream_configure(stream, stream->codec_cfg);
    if (err != 0)
    {
        *rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_INVALID, BT_BAP_ASCS_REASON_CODEC_DATA);
        return err;
    }

    return 0;
}

static int
lc3_start(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
    ARG_UNUSED(stream);
    ARG_UNUSED(rsp);
    return 0;
}

static int
lc3_metadata(struct bt_bap_stream *stream, const uint8_t meta[], size_t len, struct bt_bap_ascs_rsp *rsp)
{
    ARG_UNUSED(stream);
    ARG_UNUSED(meta);
    ARG_UNUSED(len);
    ARG_UNUSED(rsp);
    return 0;
}

static int
lc3_disable(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
    ARG_UNUSED(stream);
    ARG_UNUSED(rsp);
    return 0;
}

static int
lc3_stop(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
    ARG_UNUSED(rsp);
    LOG_INF("ASE Stop requested: stream %p", stream);
    ble_audio_stream_cleanup(stream);

    if (!ble_audio_stream_has_active_unicast())
    {
        ble_bap_broadcast_sink_resume();
    }
    return 0;
}

static int
lc3_release(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
    ARG_UNUSED(rsp);
    LOG_INF("ASE Release requested: stream %p", stream);
    ble_audio_stream_cleanup(stream);

    if (!ble_audio_stream_has_active_unicast())
    {
        ble_bap_broadcast_sink_resume();
    }
    return 0;
}

void
ble_bap_unicast_server_evict_conn(struct bt_conn *conn)
{
    ble_audio_stream_evict_conn(conn);
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

void
ble_bap_unicast_server_start(void)
{
    bt_bap_unicast_server_register(&param);
    bt_bap_unicast_server_register_cb(&unicast_server_cb);
    bt_pacs_cap_register(BT_AUDIO_DIR_SINK, &cap_sink);

    if (set_location() != 0 || set_supported_contexts() != 0 || set_available_contexts() != 0)
    {
        LOG_ERR("PACS registration failure");
    }

    init_vcp_volume_renderer();
}