/**
  **************************************************************************
  * @file     la_device.h
  * @brief    Logic Analyzer Device State Machine and API
  * @version  1.0
  * @date     2024
  **************************************************************************
  */

#ifndef __LA_DEVICE_H__
#define __LA_DEVICE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "la_protocol.h"
#include <stdint.h>

/* ── Device States ── */
typedef enum {
    LA_STATE_IDLE = 0,          /* Power-on, after RESET */
    LA_STATE_CONFIGURED,        /* Rate + channels configured, can ARM */
    LA_STATE_ARMED,             /* Waiting for trigger */
    LA_STATE_STREAMING,         /* Actively sending sample data */
    LA_STATE_STOPPED            /* Capture complete, can read out */
} LaState;

/* ── Device Configuration ── */
typedef struct {
    uint32_t rate_divider;        /* base_clock / desired_rate */
    uint32_t sample_depth;        /* max samples (0 = infinite) */
    uint8_t  channel_mask;        /* bit per channel (0xFF = all) */
    uint8_t  trigger_channel;     /* 0-7 */
    uint8_t  trigger_edge;        /* LA_EDGE_* */
    uint8_t  trigger_enabled;     /* 0 or 1 */
    uint8_t  pretrigger_pct;      /* 0-100 */
    uint8_t  capture_mode;        /* LA_ARM_SINGLE or LA_ARM_STREAM */
    uint8_t  flags;               /* bit 0 = RLE */
    uint8_t  sample_width;        /* 8 or 16 (bits per sample); default 8 */
    uint8_t  port;                /* LA_PORT_GPIOC (default) or LA_PORT_GPIOB */
    uint8_t  configured_fields;   /* bitmask tracking which SET_* received */
} LaDeviceConfig;

/* ── USB Packet TX Buffer ── */
#define LA_TX_BUF_SIZE          (LA_PKT_HEADER_SIZE + LA_MAX_PAYLOAD)

/* ── Sample Buffer ── */
#define LA_SAMPLE_BUF_SIZE      (32 * 1024)  /* 32KB ring buffer */

/* ── Device Context ── */
typedef struct LaDevice {
    /* State */
    LaState         state;
    LaDeviceConfig  config;
    uint8_t         seq;                /* packet sequence counter */
    
    /* USB send callback */
    int (*send_fn)(const uint8_t *buf, uint16_t len, void *ctx);
    void *send_ctx;
    
    /* Capture state */
    uint32_t        capture_sample_count;   /* total samples captured in current session */
    uint32_t        capture_trigger_idx;    /* sample index where trigger fired (0xFFFFFFFF = none) */
    uint8_t         triggered;              /* 0 = waiting, 1 = triggered */
    
    /* TX state */
    uint8_t         tx_buf[LA_TX_BUF_SIZE];
    uint16_t        tx_pending_len;         /* bytes in tx_buf awaiting USB retry (0 = none) */
    uint16_t        tx_pending_samples;     /* sample count in pending packet */
    
    /* Sample buffer (ring buffer for DMA) */
    uint8_t         sample_buf[LA_SAMPLE_BUF_SIZE];
    uint32_t        sample_wr_idx;          /* DMA write index */
    uint32_t        sample_rd_idx;          /* firmware read index */
    
    /* Packet size watermarks (payload bytes, tracked at USB handoff) */
    uint16_t        pkt_min;                /* smallest payload sent */
    uint16_t        pkt_max;                /* largest payload sent */
    uint32_t        pkt_count;              /* total packets sent */
    uint32_t        usb_fail_count;         /* send_fn returned -1 */
    
    /* Watchdog */
    uint32_t        watchdog_ticks;         /* ticks since last activity */
    uint32_t        watchdog_timeout;       /* auto-reset timeout (0 = disabled) */
    
    /* Synthetic data generator (for testing) */
    uint32_t        synth_counter;
    
} LaDevice;

/* ══════════════════════════════════════════════════════════════════════
 * API Functions
 * ══════════════════════════════════════════════════════════════════════ */

/**
  * @brief  Initialize LA device
  * @param  dev: pointer to LaDevice structure
  * @param  send_fn: USB TX callback function
  * @param  ctx: opaque context passed to send_fn
  * @retval None
  */
void la_device_init(LaDevice *dev, 
                    int (*send_fn)(const uint8_t *buf, uint16_t len, void *ctx),
                    void *ctx);

/**
  * @brief  Reset device to IDLE state
  * @param  dev: pointer to LaDevice structure
  * @retval None
  */
void la_device_reset(LaDevice *dev);

/**
  * @brief  Process received command data from USB Bulk OUT
  * @param  dev: pointer to LaDevice structure
  * @param  buf: received data buffer
  * @param  len: length of received data
  * @retval Number of commands processed
  */
int la_device_on_rx(LaDevice *dev, const uint8_t *buf, uint32_t len);

/**
  * @brief  Polling function - generates and sends sample data if STREAMING
  * @param  dev: pointer to LaDevice structure
  * @retval 1 = packet sent, 0 = nothing to do, -1 = capture complete
  */
int la_device_poll(LaDevice *dev);

/**
  * @brief  Tick function - manages watchdog timer
  * @param  dev: pointer to LaDevice structure
  * @retval None
  */
void la_device_tick(LaDevice *dev);

/**
  * @brief  Write packet header with checksum
  * @param  buf: destination buffer (must be at least LA_PKT_HEADER_SIZE bytes)
  * @param  type: response type (LA_RSP_* or LA_DATA_*)
  * @param  flags: flags byte
  * @param  payload_len: length of payload (0-504)
  * @param  seq: sequence number
  * @retval None
  */
void la_write_header(uint8_t *buf, uint8_t type, uint8_t flags, 
                     uint16_t payload_len, uint8_t seq);

/**
  * @brief  Send response with payload
  * @param  dev: pointer to LaDevice structure
  * @param  type: response type
  * @param  payload: pointer to payload data (can be NULL if len=0)
  * @param  len: payload length
  * @retval Send result from send_fn
  */
int la_send_response(LaDevice *dev, uint8_t type, const void *payload, uint16_t len);

/**
  * @brief  Send ACK response
  * @param  dev: pointer to LaDevice structure
  * @param  cmd: command ID being ACKed
  * @retval Send result
  */
int la_send_ack(LaDevice *dev, uint8_t cmd);

/**
  * @brief  Send NAK response
  * @param  dev: pointer to LaDevice structure
  * @param  cmd: command ID being NAKed
  * @param  error: error code (LA_ERR_*)
  * @retval Send result
  */
int la_send_nak(LaDevice *dev, uint8_t cmd, uint8_t error);

/* ══════════════════════════════════════════════════════════════════════
 * Hardware Capture Engine (MCU-specific)
 * ══════════════════════════════════════════════════════════════════════ */

/**
  * @brief  Initialize hardware capture engine (GPIO/Timer/DMA)
  * @param  dev: pointer to LaDevice structure
  * @retval None
  */
void la_hw_init(LaDevice *dev);

/**
  * @brief  Start hardware capture
  * @param  dev: pointer to LaDevice structure
  * @retval None
  */
void la_hw_start_capture(LaDevice *dev);

/**
  * @brief  Stop hardware capture
  * @param  dev: pointer to LaDevice structure
  * @retval None
  */
void la_hw_stop_capture(LaDevice *dev);

/**
  * @brief  Read GPIOC[0:7] into buffer (software polling)
  * @param  buf: destination buffer
  * @param  count: number of samples to read
  * @param  mask: channel mask
  * @retval number of samples read
  */
uint32_t la_hw_read_gpio(uint8_t *buf, uint32_t count, uint8_t mask);

/**
  * @brief  Get number of available samples in ring buffer
  * @param  dev: pointer to LaDevice structure
  * @retval Number of samples available
  */
uint32_t la_hw_available_samples(LaDevice *dev);

/**
  * @brief  Read samples from ring buffer
  * @param  dev: pointer to LaDevice structure
  * @param  buf: destination buffer
  * @param  count: number of samples to read
  * @retval Number of samples actually read
  */
uint32_t la_hw_read_samples(LaDevice *dev, uint8_t *buf, uint32_t count);

#ifdef __cplusplus
}
#endif

#endif /* __LA_DEVICE_H__ */
