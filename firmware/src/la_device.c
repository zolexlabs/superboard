/**
  **************************************************************************
  * @file     la_device.c
  * @brief    Logic Analyzer Device Implementation
  * @version  1.0
  * @date     2024
  **************************************************************************
  */

#include "la_device.h"
#include "fw_version.h"   /* FW_VER_* — the real product version reported in PONG */
#include <string.h>
#include <stdio.h>   /* snprintf (debug strings in handle_stop) */

/* DWT cycle counter — Cortex-M4 debug register at fixed address */
#define DWT_CYCCNT (*(volatile uint32_t *)0xE0001004)

/* UART debug printing (defined in main.c, always available) */
extern void uart_printf(const char *fmt, ...);

/* USB send timing stats (defined in main.c, updated by la_usb_send) */
extern volatile uint32_t usb_send_cyc_min;
extern volatile uint32_t usb_send_cyc_max;
extern volatile uint64_t usb_send_cyc_total;
extern volatile uint32_t usb_send_count;
extern volatile uint32_t usb_send_fail_count;
extern volatile uint32_t usb_send_delta_max;
extern volatile uint32_t usb_send_delta_max_at;

/* Hardware capabilities */
#define LA_MAX_SAMPLE_RATE      24000000    /* 24 MHz base clock */
#define LA_BUFFER_SIZE          LA_SAMPLE_BUF_SIZE

/* 4 KB blast buffer — sits outside LaDevice to avoid bloating the struct.
 * Pre-filled once in handle_arm(), then blasted repeatedly. */
#define BLAST_4K_SIZE  4096
static uint8_t blast_4k_buf[BLAST_4K_SIZE] __attribute__((aligned(4)));

/* ══════════════════════════════════════════════════════════════════════
 * Helper Functions
 * ══════════════════════════════════════════════════════════════════════ */

/**
  * @brief  Write packet header with checksum
  */
void la_write_header(uint8_t *buf, uint8_t type, uint8_t flags, 
                     uint16_t payload_len, uint8_t seq)
{
    buf[0] = LA_SYNC_0;
    buf[1] = LA_SYNC_1;
    buf[2] = type;
    buf[3] = flags;
    buf[4] = (uint8_t)(payload_len & 0xFF);         /* little-endian */
    buf[5] = (uint8_t)((payload_len >> 8) & 0xFF);
    buf[6] = seq;
    
    /* XOR checksum of bytes [0..6] */
    buf[7] = buf[0] ^ buf[1] ^ buf[2] ^ buf[3] ^ buf[4] ^ buf[5] ^ buf[6];
}

/**
  * @brief  Send response with payload
  */
int la_send_response(LaDevice *dev, uint8_t type, const void *payload, uint16_t len)
{
    if (len > LA_MAX_PAYLOAD) {
        return -1;
    }
    
    /* Write header */
    la_write_header(dev->tx_buf, type, 0, len, dev->seq++);
    
    /* Copy payload if present */
    if (len > 0 && payload != NULL) {
        memcpy(dev->tx_buf + LA_PKT_HEADER_SIZE, payload, len);
    }
    
    /* Send via USB */
    return dev->send_fn(dev->tx_buf, LA_PKT_HEADER_SIZE + len, dev->send_ctx);
}

/**
  * @brief  Send ACK response
  */
int la_send_ack(LaDevice *dev, uint8_t cmd)
{
    LaRspAck ack;
    ack.acked_cmd = cmd;
    return la_send_response(dev, LA_RSP_ACK, &ack, sizeof(ack));
}

/**
  * @brief  Send NAK response
  */
int la_send_nak(LaDevice *dev, uint8_t cmd, uint8_t error)
{
    LaRspNak nak;
    nak.naked_cmd = cmd;
    nak.error_code = error;
    return la_send_response(dev, LA_RSP_NAK, &nak, sizeof(nak));
}

/* ══════════════════════════════════════════════════════════════════════
 * Device Management
 * ══════════════════════════════════════════════════════════════════════ */

/**
  * @brief  Initialize LA device
  */
void la_device_init(LaDevice *dev, 
                    int (*send_fn)(const uint8_t *buf, uint16_t len, void *ctx),
                    void *ctx)
{
    memset(dev, 0, sizeof(LaDevice));
    
    dev->send_fn = send_fn;
    dev->send_ctx = ctx;
    dev->state = LA_STATE_IDLE;
    dev->watchdog_timeout = 0;  /* watchdog disabled */
    
    /* Reset configuration */
    dev->config.rate_divider = 1;
    dev->config.sample_depth = 0;  /* infinite */
    dev->config.channel_mask = 0xFF;
    dev->config.trigger_channel = 0;
    dev->config.trigger_edge = LA_EDGE_NONE;
    dev->config.trigger_enabled = 0;
    dev->config.pretrigger_pct = 50;
    dev->config.capture_mode = LA_ARM_SINGLE;
    dev->config.flags = 0;
    dev->config.sample_width = 8;     /* back-compat default */
    dev->config.port = LA_PORT_GPIOC; /* back-compat default */
    dev->config.configured_fields = 0;
}

/**
  * @brief  Reset device to IDLE state
  */
void la_device_reset(LaDevice *dev)
{
    int (*send_fn)(const uint8_t *buf, uint16_t len, void *ctx) = dev->send_fn;
    void *ctx = dev->send_ctx;
    uint32_t timeout = dev->watchdog_timeout;
    
    /* Stop hardware if running */
    if (dev->state == LA_STATE_ARMED || dev->state == LA_STATE_STREAMING) {
        la_hw_stop_capture(dev);
    }
    
    /* Reinitialize */
    la_device_init(dev, send_fn, ctx);
    dev->watchdog_timeout = timeout;
}

/* ══════════════════════════════════════════════════════════════════════
 * Command Handlers
 * ══════════════════════════════════════════════════════════════════════ */

static void handle_ping(LaDevice *dev, const LaCmd *cmd)
{
    LaRspPong pong;
    pong.version_major = FW_VER_MAJOR;
    pong.version_minor = FW_VER_MINOR;
    pong.version_patch = FW_VER_PATCH;
    pong.flags = 0;
    
    la_send_response(dev, LA_RSP_PONG, &pong, sizeof(pong));
}

static void handle_get_caps(LaDevice *dev, const LaCmd *cmd)
{
    LaRspCaps caps;
    caps.max_sample_rate = LA_MAX_SAMPLE_RATE;
    caps.max_channels = LA_MAX_CHANNELS;
    caps.reserved0 = 0;
    caps.reserved1 = 0;
    caps.buffer_size = LA_BUFFER_SIZE;
    
    la_send_response(dev, LA_RSP_CAPS, &caps, sizeof(caps));
}

static void handle_set_rate(LaDevice *dev, const LaCmd *cmd)
{
    uint32_t divider;
    /* Can't configure while armed/streaming */
    if (dev->state == LA_STATE_ARMED || dev->state == LA_STATE_STREAMING) {
        la_send_nak(dev, LA_CMD_SET_RATE, LA_ERR_BUSY);
        return;
    }
    
    /* Extract rate_divider (24-bit little-endian) */
    divider = cmd->param[0] | (cmd->param[1] << 8) | (cmd->param[2] << 16);
    
    if (divider == 0) {
        la_send_nak(dev, LA_CMD_SET_RATE, LA_ERR_BAD_PARAM);
        return;
    }
    
    dev->config.rate_divider = divider;
    dev->config.configured_fields |= LA_DCFG_RATE;
    
    la_send_ack(dev, LA_CMD_SET_RATE);
    
    /* Transition to CONFIGURED if we have minimum config */
    if ((dev->config.configured_fields & (LA_DCFG_RATE | LA_DCFG_CHANNELS)) == 
        (LA_DCFG_RATE | LA_DCFG_CHANNELS)) {
        dev->state = LA_STATE_CONFIGURED;
    }
}

static void handle_set_port(LaDevice *dev, const LaCmd *cmd)
{
    if (dev->state == LA_STATE_ARMED || dev->state == LA_STATE_STREAMING) {
        la_send_nak(dev, LA_CMD_SET_PORT, LA_ERR_BUSY);
        return;
    }
    uint8_t p = cmd->param[0];
    if (p != LA_PORT_GPIOC && p != LA_PORT_GPIOB) {
        la_send_nak(dev, LA_CMD_SET_PORT, LA_ERR_BAD_PARAM);
        return;
    }
    dev->config.port = p;
    dev->config.configured_fields |= LA_DCFG_PORT;
    la_send_ack(dev, LA_CMD_SET_PORT);
}

static void handle_set_sample_width(LaDevice *dev, const LaCmd *cmd)
{
    if (dev->state == LA_STATE_ARMED || dev->state == LA_STATE_STREAMING) {
        la_send_nak(dev, LA_CMD_SET_SAMPLE_WIDTH, LA_ERR_BUSY);
        return;
    }
    uint8_t w = cmd->param[0];
    if (w != 8 && w != 16) {
        la_send_nak(dev, LA_CMD_SET_SAMPLE_WIDTH, LA_ERR_BAD_PARAM);
        return;
    }
    dev->config.sample_width = w;
    dev->config.configured_fields |= LA_DCFG_SAMPLE_WIDTH;
    la_send_ack(dev, LA_CMD_SET_SAMPLE_WIDTH);
}

static void handle_set_channels(LaDevice *dev, const LaCmd *cmd)
{
    if (dev->state == LA_STATE_ARMED || dev->state == LA_STATE_STREAMING) {
        la_send_nak(dev, LA_CMD_SET_CHANNELS, LA_ERR_BUSY);
        return;
    }
    
    dev->config.channel_mask = cmd->param[0];
    dev->config.configured_fields |= LA_DCFG_CHANNELS;
    
    la_send_ack(dev, LA_CMD_SET_CHANNELS);
    
    /* Transition to CONFIGURED if we have minimum config */
    if ((dev->config.configured_fields & (LA_DCFG_RATE | LA_DCFG_CHANNELS)) == 
        (LA_DCFG_RATE | LA_DCFG_CHANNELS)) {
        dev->state = LA_STATE_CONFIGURED;
    }
}

static void handle_set_trigger(LaDevice *dev, const LaCmd *cmd)
{
    uint8_t channel, edge, enabled;
    if (dev->state == LA_STATE_ARMED || dev->state == LA_STATE_STREAMING) {
        la_send_nak(dev, LA_CMD_SET_TRIGGER, LA_ERR_BUSY);
        return;
    }
    
    channel = cmd->param[0];
    edge = cmd->param[1];
    enabled = cmd->param[2];
    
    if (channel >= LA_MAX_CHANNELS || edge > LA_EDGE_BOTH) {
        la_send_nak(dev, LA_CMD_SET_TRIGGER, LA_ERR_BAD_PARAM);
        return;
    }
    
    dev->config.trigger_channel = channel;
    dev->config.trigger_edge = edge;
    dev->config.trigger_enabled = enabled ? 1 : 0;
    dev->config.configured_fields |= LA_DCFG_TRIGGER;
    
    la_send_ack(dev, LA_CMD_SET_TRIGGER);
}

static void handle_set_depth(LaDevice *dev, const LaCmd *cmd)
{
    uint32_t depth;
    if (dev->state == LA_STATE_ARMED || dev->state == LA_STATE_STREAMING) {
        la_send_nak(dev, LA_CMD_SET_DEPTH, LA_ERR_BUSY);
        return;
    }
    
    /* Extract depth (24-bit little-endian) */
    depth = cmd->param[0] | (cmd->param[1] << 8) | (cmd->param[2] << 16);
    
    dev->config.sample_depth = depth;
    dev->config.configured_fields |= LA_DCFG_DEPTH;
    
    la_send_ack(dev, LA_CMD_SET_DEPTH);
}

static void handle_set_pretrig(LaDevice *dev, const LaCmd *cmd)
{
    uint8_t pct;
    if (dev->state == LA_STATE_ARMED || dev->state == LA_STATE_STREAMING) {
        la_send_nak(dev, LA_CMD_SET_PRETRIG, LA_ERR_BUSY);
        return;
    }
    
    pct = cmd->param[0];
    if (pct > 100) {
        pct = 100;  /* clamp to 100% */
    }
    
    dev->config.pretrigger_pct = pct;
    dev->config.configured_fields |= LA_DCFG_PRETRIG;
    
    la_send_ack(dev, LA_CMD_SET_PRETRIG);
}

static void handle_set_flags(LaDevice *dev, const LaCmd *cmd)
{
    if (dev->state == LA_STATE_ARMED || dev->state == LA_STATE_STREAMING) {
        la_send_nak(dev, LA_CMD_SET_FLAGS, LA_ERR_BUSY);
        return;
    }
    
    dev->config.flags = cmd->param[0];
    dev->config.configured_fields |= LA_DCFG_FLAGS;
    
    la_send_ack(dev, LA_CMD_SET_FLAGS);
}

static void handle_arm(LaDevice *dev, const LaCmd *cmd)
{
    uint8_t mode;
    LaRspTriggered trig;
    
    /* Must be CONFIGURED to ARM */
    if (dev->state != LA_STATE_CONFIGURED) {
        la_send_nak(dev, LA_CMD_ARM, LA_ERR_BAD_PARAM);
        return;
    }
    
    /* Check minimum configuration (rate + channels) */
    if ((dev->config.configured_fields & (LA_DCFG_RATE | LA_DCFG_CHANNELS)) != 
        (LA_DCFG_RATE | LA_DCFG_CHANNELS)) {
        la_send_nak(dev, LA_CMD_ARM, LA_ERR_BAD_PARAM);
        return;
    }
    
    mode = cmd->param[0];
    if (mode != LA_ARM_SINGLE && mode != LA_ARM_STREAM &&
        mode != LA_ARM_BLAST_504 && mode != LA_ARM_BLAST_100 &&
        mode != LA_ARM_BLAST_4K) {
        la_send_nak(dev, LA_CMD_ARM, LA_ERR_BAD_PARAM);
        return;
    }
    
    dev->config.capture_mode = mode;
    
    /* Reset capture state */
    dev->capture_sample_count = 0;
    dev->capture_trigger_idx = 0xFFFFFFFF;
    dev->triggered = 0;
    dev->sample_wr_idx = 0;
    dev->sample_rd_idx = 0;
    dev->synth_counter = 0;
    dev->tx_pending_len = 0;
    dev->tx_pending_samples = 0;
    dev->pkt_min = 0xFFFF;     /* will be driven down */
    dev->pkt_max = 0;
    dev->pkt_count = 0;
    dev->usb_fail_count = 0;
    
    /* Reset USB send timing stats */
    usb_send_cyc_min = 0xFFFFFFFF;
    usb_send_cyc_max = 0;
    usb_send_cyc_total = 0;
    usb_send_count = 0;
    usb_send_fail_count = 0;
    usb_send_delta_max = 0;
    usb_send_delta_max_at = 0;
    {
        extern volatile uint32_t usb_send_start_tick;
        extern volatile uint32_t tick_ms;
        usb_send_start_tick = tick_ms;
    }
    
    /* Reset double-buffer diagnostics */
    {
        extern volatile uint32_t dbl_isr_pickup;
        extern volatile uint32_t dbl_isr_idle;
        dbl_isr_pickup = 0;
        dbl_isr_idle = 0;
    }
    
    /* Start hardware capture (skip for blast benchmark — no DMA needed) */
    if (mode != LA_ARM_BLAST_504 && mode != LA_ARM_BLAST_100 &&
        mode != LA_ARM_BLAST_4K) {
        la_hw_start_capture(dev);
    } else {
        /* Pre-fill blast payload with counter pattern */
        uint16_t i;
        uint16_t sz = (mode == LA_ARM_BLAST_504) ? LA_MAX_PAYLOAD :
                      (mode == LA_ARM_BLAST_100) ? 100 : 0;
        if (sz > 0) {
            for (i = 0; i < sz; i++) {
                dev->tx_buf[LA_PKT_HEADER_SIZE + i] = (uint8_t)(i & 0xFF);
            }
        }
        /* Pre-fill the 4K blast buffer with 8 framed LA packets
         * (8-byte header + 504-byte payload = 512 bytes each).
         * Seq numbers are static — parser will count drops, but
         * byte/throughput metrics remain accurate. */
        if (mode == LA_ARM_BLAST_4K) {
            uint16_t off;
            for (off = 0; off < BLAST_4K_SIZE; off += 512) {
                la_write_header(blast_4k_buf + off, LA_DATA_SAMPLES, 0,
                                LA_MAX_PAYLOAD, (uint8_t)(off / 512));
                for (i = 0; i < LA_MAX_PAYLOAD; i++) {
                    blast_4k_buf[off + LA_PKT_HEADER_SIZE + i] = (uint8_t)(i & 0xFF);
                }
            }
        }
        uart_printf("BLAST: mode=%s, payload=%u bytes\n",
                    (mode == LA_ARM_BLAST_504) ? "504" :
                    (mode == LA_ARM_BLAST_100) ? "100" : "4K",
                    (mode == LA_ARM_BLAST_4K) ? 4096u : (unsigned)sz);
    }
    
    /* Send ACK */
    la_send_ack(dev, LA_CMD_ARM);
    
    /* Transition to ARMED */
    dev->state = LA_STATE_ARMED;
    
    /* Immediately trigger and start streaming real GPIO data */
    trig.trigger_sample = 0;
    la_send_response(dev, LA_RSP_TRIGGERED, &trig, sizeof(trig));
    dev->triggered = 1;
    dev->state = LA_STATE_STREAMING;
}

static void handle_stop(LaDevice *dev, const LaCmd *cmd)
{
    if (dev->state != LA_STATE_ARMED && dev->state != LA_STATE_STREAMING) {
        la_send_nak(dev, LA_CMD_STOP, LA_ERR_NOT_ARMED);
        return;
    }
    
    /* Snapshot DLT stats NOW, before any debug sends contaminate them */
    uint32_t snap_delta_max    = usb_send_delta_max;
    uint32_t snap_delta_max_at = usb_send_delta_max_at;
    uint32_t snap_send_count   = usb_send_count;
    
    {
    LaRspDataEnd end;
    /* Stop hardware */
    la_hw_stop_capture(dev);
    
    /* Send ACK */
    la_send_ack(dev, LA_CMD_STOP);
    
    /* Send debug text with device-side stats */
    {
        char dbg[LA_MAX_PAYLOAD];
        int pos = 0;
        extern volatile uint32_t dbl_isr_pickup;
        extern volatile uint32_t dbl_isr_idle;
        if (snap_send_count > 0) {
            uint32_t avg_cyc = (uint32_t)(usb_send_cyc_total / snap_send_count);
            pos += snprintf(dbg + pos, sizeof(dbg) - pos,
                "USB: %lu sends, avg=%lu us, max=%lu us, fail=%lu\n",
                (unsigned long)snap_send_count,
                (unsigned long)(avg_cyc / 216),
                (unsigned long)(usb_send_cyc_max / 216),
                (unsigned long)usb_send_fail_count);
        }
        pos += snprintf(dbg + pos, sizeof(dbg) - pos,
            "DLT: max %lu us (send #%lu/%lu, window 0.5-14.5s)\n",
            (unsigned long)(snap_delta_max / 216),
            (unsigned long)snap_delta_max_at,
            (unsigned long)snap_send_count);
        pos += snprintf(dbg + pos, sizeof(dbg) - pos,
            "DBL: pickup=%lu idle=%lu\n",
            (unsigned long)dbl_isr_pickup,
            (unsigned long)dbl_isr_idle);
        la_send_response(dev, LA_RSP_DEBUG_TEXT, dbg, (uint16_t)pos);
    }
    
    /* Send DATA_END */
    end.total_samples = dev->capture_sample_count;
    la_send_response(dev, LA_DATA_END, &end, sizeof(end));
    
    /* PA9 uart_printf capture-stats dump removed to reclaim flash (MDK-Lite
       32 KB cap) for the LED-mirror feature. The same CAP/USB/DLT/DBL stats
       still reach the workbench via the LA_RSP_DEBUG_TEXT response built above. */

    /* Transition to STOPPED */
    dev->state = LA_STATE_STOPPED;
    }
}

static void handle_flush(LaDevice *dev, const LaCmd *cmd)
{
    /* Clear sample buffer */
    dev->sample_rd_idx = dev->sample_wr_idx;
    la_send_ack(dev, LA_CMD_FLUSH);
}

static void handle_reset(LaDevice *dev, const LaCmd *cmd)
{
    LaRspPong pong;
    la_device_reset(dev);
    
    /* Send PONG after reset */
    pong.version_major = FW_VER_MAJOR;
    pong.version_minor = FW_VER_MINOR;
    pong.version_patch = FW_VER_PATCH;
    pong.flags = 0;
    la_send_response(dev, LA_RSP_PONG, &pong, sizeof(pong));
}

/* ══════════════════════════════════════════════════════════════════════
 * Command Dispatch
 * ══════════════════════════════════════════════════════════════════════ */

/**
  * @brief  Process received command data from USB Bulk OUT
  */
int la_device_on_rx(LaDevice *dev, const uint8_t *buf, uint32_t len)
{
    int cmd_count = 0;
    const LaCmd *cmd;
    
    /* Reset watchdog */
    dev->watchdog_ticks = 0;
    
    /* Process complete 8-byte commands */
    while (len >= LA_CMD_SIZE) {
        cmd = (const LaCmd *)buf;
        
        /* Dispatch command */
        switch (cmd->cmd) {
            case LA_CMD_PING:
                handle_ping(dev, cmd);
                break;
            case LA_CMD_GET_CAPS:
                handle_get_caps(dev, cmd);
                break;
            case LA_CMD_SET_RATE:
                handle_set_rate(dev, cmd);
                break;
            case LA_CMD_SET_CHANNELS:
                handle_set_channels(dev, cmd);
                break;
            case LA_CMD_SET_TRIGGER:
                handle_set_trigger(dev, cmd);
                break;
            case LA_CMD_SET_DEPTH:
                handle_set_depth(dev, cmd);
                break;
            case LA_CMD_SET_PRETRIG:
                handle_set_pretrig(dev, cmd);
                break;
            case LA_CMD_SET_FLAGS:
                handle_set_flags(dev, cmd);
                break;
            case LA_CMD_SET_SAMPLE_WIDTH:
                handle_set_sample_width(dev, cmd);
                break;
            case LA_CMD_SET_PORT:
                handle_set_port(dev, cmd);
                break;
            case LA_CMD_ARM:
                handle_arm(dev, cmd);
                break;
            case LA_CMD_STOP:
                handle_stop(dev, cmd);
                break;
            case LA_CMD_FLUSH:
                handle_flush(dev, cmd);
                break;
            case LA_CMD_RESET:
                handle_reset(dev, cmd);
                break;
            default:
                la_send_nak(dev, cmd->cmd, LA_ERR_UNKNOWN_CMD);
                break;
        }
        
        buf += LA_CMD_SIZE;
        len -= LA_CMD_SIZE;
        cmd_count++;
    }
    
    return cmd_count;
}

/* ══════════════════════════════════════════════════════════════════════
 * Polling & Streaming
 * ══════════════════════════════════════════════════════════════════════ */

/* Defined in la_hardware.c */
extern uint32_t la_hw_ring_available(void);
extern uint32_t la_hw_ring_read(uint8_t *buf, uint32_t count);

/**
  * @brief  Polling function - drains timer-sampled ring buffer into USB packets.
  *         When DMA ISR signals data-ready (HDT/FDT), drains all available
  *         data in a tight loop of max-sized USB packets.
  *
  *         Non-blocking: if USB endpoint is busy, we stop draining and
  *         retry the same packet next time (tx_pending keeps the unsent
  *         packet in tx_buf).
  */
int la_device_poll(LaDevice *dev)
{
    LaRspDataEnd end;
    uint32_t avail;
    uint32_t samples_to_send;
    int result;
    int packets_sent = 0;
    
    if (dev->state != LA_STATE_STREAMING) {
        return 0;
    }
    
    /* ── Blast benchmark: bypass DMA, smash USB as fast as possible ── */
    if (dev->config.capture_mode == LA_ARM_BLAST_504 ||
        dev->config.capture_mode == LA_ARM_BLAST_100) {
        
        uint16_t payload_sz = (dev->config.capture_mode == LA_ARM_BLAST_504)
                              ? LA_MAX_PAYLOAD : 100;
        
        /* Bounded blast loop — send up to N packets per poll() call,
         * then return to main loop so LED, USB RX, tick all get serviced.
         * 64 × 512B = 32 KB per call; main loop runs thousands of times/sec. */
        int burst_limit = 64;
        while (burst_limit-- > 0) {
            la_write_header(dev->tx_buf, LA_DATA_SAMPLES, 0,
                            payload_sz, dev->seq);
            
            /* Stamp DWT timestamp at payload[0..3] */
            {
                uint32_t ts = DWT_CYCCNT;
                uint8_t *p = dev->tx_buf + LA_PKT_HEADER_SIZE;
                p[0] = (uint8_t)(ts);
                p[1] = (uint8_t)(ts >> 8);
                p[2] = (uint8_t)(ts >> 16);
                p[3] = (uint8_t)(ts >> 24);
            }
            
            result = dev->send_fn(dev->tx_buf,
                                  LA_PKT_HEADER_SIZE + payload_sz,
                                  dev->send_ctx);
            if (result >= 0) {
                dev->seq++;  /* advance only after successful send */
                dev->capture_sample_count += payload_sz;
                dev->pkt_count++;
                packets_sent++;
            } else {
                dev->usb_fail_count++;
                break;  /* USB busy — yield to main loop for RX check */
            }
        }
        
        /* Track watermarks (constant in blast mode but keep consistent) */
        if (payload_sz < dev->pkt_min) dev->pkt_min = payload_sz;
        if (payload_sz > dev->pkt_max) dev->pkt_max = payload_sz;
        
        return packets_sent;
    }
    
    /* ── 4K Blast: single 4096-byte USB transfer per call ── */
    if (dev->config.capture_mode == LA_ARM_BLAST_4K) {
        /* Send the pre-filled 4K buffer as one USB transfer.
         * The USB stack splits it into 8 × 512-byte hardware packets
         * internally, reducing per-call overhead vs 8 separate sends.
         * Bounded to 8 sends per call (8 × 4 KB = 32 KB) so the main
         * loop can service LED, USB RX, tick between bursts.
         *
         * Stamp correct sequence numbers on all 8 sub-packet headers
         * before each send — only seq (byte 6) and checksum (byte 7)
         * change; the rest of the header and payload are pre-filled. */
        int burst_limit = 8;
        while (burst_limit-- > 0) {
            /* Stamp seq + checksum in each of the 8 sub-packet headers.
             * Also stamp DWT timestamp at payload[0..3] of each sub-packet.
             * Save seq so we can roll back if the send fails. */
            uint8_t saved_seq = dev->seq;
            {
                uint16_t off;
                for (off = 0; off < BLAST_4K_SIZE; off += 512) {
                    uint8_t *hdr = blast_4k_buf + off;
                    hdr[6] = dev->seq++;
                    hdr[7] = hdr[0] ^ hdr[1] ^ hdr[2] ^ hdr[3]
                           ^ hdr[4] ^ hdr[5] ^ hdr[6];
                    /* Timestamp at payload[0..3] */
                    uint32_t ts = DWT_CYCCNT;
                    uint8_t *p = hdr + LA_PKT_HEADER_SIZE;
                    p[0] = (uint8_t)(ts);
                    p[1] = (uint8_t)(ts >> 8);
                    p[2] = (uint8_t)(ts >> 16);
                    p[3] = (uint8_t)(ts >> 24);
                }
            }
            result = dev->send_fn(blast_4k_buf, BLAST_4K_SIZE, dev->send_ctx);
            if (result >= 0) {
                dev->capture_sample_count += BLAST_4K_SIZE;
                dev->pkt_count++;
                packets_sent++;
            } else {
                dev->seq = saved_seq;  /* roll back — packet never went out */
                dev->usb_fail_count++;
                break;
            }
        }
        
        if (BLAST_4K_SIZE < dev->pkt_min) dev->pkt_min = BLAST_4K_SIZE;
        if (BLAST_4K_SIZE > dev->pkt_max) dev->pkt_max = BLAST_4K_SIZE;
        
        return packets_sent;
    }
    
    /* Check if we've reached sample depth (single-shot mode) */
    if (dev->config.capture_mode == LA_ARM_SINGLE && 
        dev->config.sample_depth > 0 &&
        dev->capture_sample_count >= dev->config.sample_depth) {
        
        /* Send debug text + DATA_END and stop */
        /* Snapshot DLT before debug sends contaminate the counters */
        uint32_t snap_dm  = usb_send_delta_max;
        uint32_t snap_dma = usb_send_delta_max_at;
        uint32_t snap_sc  = usb_send_count;
        {
            char dbg[LA_MAX_PAYLOAD];
            int pos = 0;
            extern volatile uint32_t dbl_isr_pickup;
            extern volatile uint32_t dbl_isr_idle;
            if (snap_sc > 0) {
                uint32_t avg_cyc = (uint32_t)(usb_send_cyc_total / snap_sc);
                pos += snprintf(dbg + pos, sizeof(dbg) - pos,
                    "USB: %lu sends, avg=%lu us, max=%lu us, fail=%lu\n",
                    (unsigned long)snap_sc,
                    (unsigned long)(avg_cyc / 216),
                    (unsigned long)(usb_send_cyc_max / 216),
                    (unsigned long)usb_send_fail_count);
            }
            pos += snprintf(dbg + pos, sizeof(dbg) - pos,
                "DLT: max %lu us (send #%lu/%lu, window 0.5-14.5s)\n",
                (unsigned long)(snap_dm / 216),
                (unsigned long)snap_dma,
                (unsigned long)snap_sc);
            pos += snprintf(dbg + pos, sizeof(dbg) - pos,
                "DBL: pickup=%lu idle=%lu\n",
                (unsigned long)dbl_isr_pickup,
                (unsigned long)dbl_isr_idle);
            la_send_response(dev, LA_RSP_DEBUG_TEXT, dbg, (uint16_t)pos);
        }
        end.total_samples = dev->capture_sample_count;
        la_send_response(dev, LA_DATA_END, &end, sizeof(end));
        
        /* PA9 uart_printf capture-stats dump removed to reclaim flash (MDK-Lite
           32 KB cap) for the LED-mirror feature. The same CAP/USB/DLT/DBL stats
           still reach the workbench via the LA_RSP_DEBUG_TEXT response above. */

        dev->state = LA_STATE_STOPPED;
        la_hw_stop_capture(dev);
        return -1;
    }
    
    /* If we have a pending unsent packet from last time, retry it first */
    if (dev->tx_pending_len > 0) {
        result = dev->send_fn(dev->tx_buf, dev->tx_pending_len, dev->send_ctx);
        if (result < 0) {
            return 0;  /* still busy — try again next time */
        }
        /* Track watermarks */
        if (dev->tx_pending_samples < dev->pkt_min) dev->pkt_min = dev->tx_pending_samples;
        if (dev->tx_pending_samples > dev->pkt_max) dev->pkt_max = dev->tx_pending_samples;
        dev->pkt_count++;
        dev->capture_sample_count += dev->tx_pending_samples;
        dev->tx_pending_len = 0;
        dev->tx_pending_samples = 0;
        packets_sent++;
    }
    
    /* Wait for data-ready flag from DMA ISR (HDT/FDT fired,
     * meaning ~16 KB chunk is available), OR enough data polled
     * via DTCNT for a full packet.  This prevents sending tiny
     * packets while still giving sample-level granularity. */
    {
        extern volatile uint32_t g_data_ready;
        avail = la_hw_ring_available();
        if (avail < LA_MAX_PAYLOAD && !g_data_ready) {
            return packets_sent;
        }
        if (g_data_ready) {
            g_data_ready = 0;
        }
    }
    
    /* Drain loop: send back-to-back FULL USB packets until fewer than
     * LA_MAX_PAYLOAD bytes remain in the ring, or USB is busy.
     * Typical HDT gives 16 KB ≈ 32 packets.
     *
     * CRITICAL: only send full 504-byte packets.  Sending runts wastes
     * USB bandwidth on headers — at 1 MHz the average payload was 29 B
     * instead of 504 B, consuming 17× more packets than necessary.
     * Remainder stays in the ring until the next poll() call. */
    int drain_limit = 32;  /* yield to main loop (CDC) after 32 packets */
    while (drain_limit-- > 0) {
        avail = la_hw_ring_available();
        if (avail < LA_MAX_PAYLOAD) {
            break;  /* not enough for a full packet — leave in ring */
        }
        
        samples_to_send = LA_MAX_PAYLOAD;
        
        /* Limit to remaining samples in single-shot mode (may produce runt) */
        if (dev->config.capture_mode == LA_ARM_SINGLE && dev->config.sample_depth > 0) {
            uint32_t remaining = dev->config.sample_depth - dev->capture_sample_count;
            if (remaining == 0) break;
            if (samples_to_send > remaining) {
                samples_to_send = remaining;
            }
        }
        
        /* Copy from ring buffer into tx_buf payload area */
        samples_to_send = la_hw_ring_read(
            dev->tx_buf + LA_PKT_HEADER_SIZE, samples_to_send);
        
        if (samples_to_send == 0) {
            break;
        }
        
        /* Write header */
        la_write_header(dev->tx_buf, LA_DATA_SAMPLES, 0,
                        (uint16_t)samples_to_send, dev->seq++);
        
        /* Try to send — non-blocking */
        result = dev->send_fn(dev->tx_buf,
                              LA_PKT_HEADER_SIZE + samples_to_send, dev->send_ctx);
        
        if (result >= 0) {
            /* Track watermarks */
            if (samples_to_send < dev->pkt_min) dev->pkt_min = (uint16_t)samples_to_send;
            if (samples_to_send > dev->pkt_max) dev->pkt_max = (uint16_t)samples_to_send;
            dev->pkt_count++;
            dev->capture_sample_count += samples_to_send;
            dev->watchdog_ticks = 0;
            packets_sent++;
        } else {
            /* USB busy — save pending state, retry next poll() call */
            dev->usb_fail_count++;
            dev->tx_pending_len = LA_PKT_HEADER_SIZE + samples_to_send;
            dev->tx_pending_samples = samples_to_send;
            break;
        }
    }
    
    return packets_sent;
}

/**
  * @brief  Tick function - manages watchdog timer (time-based)
  *         Must be called from main loop.  Uses an external millisecond
  *         counter so the watchdog fires after real elapsed time, not
  *         after N busy-loop iterations.
  */

/* Provided by main.c (SysTick / TMR3 based) */
extern volatile uint32_t tick_ms;

void la_device_tick(LaDevice *dev)
{
    uint32_t now;
    uint32_t elapsed;

    if (dev->watchdog_timeout == 0) {
        return;  /* watchdog disabled */
    }
    
    /* Only run watchdog when actively streaming or armed */
    if (dev->state != LA_STATE_STREAMING && dev->state != LA_STATE_ARMED) {
        return;
    }
    
    now = tick_ms;
    elapsed = now - dev->watchdog_ticks;   /* watchdog_ticks = last activity timestamp */
    
    if (elapsed >= dev->watchdog_timeout) {
        /* Timeout - reset to IDLE */
        la_device_reset(dev);
    }
}
