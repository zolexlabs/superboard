/*
 * la_protocol.h — Wire protocol for the Unified Debug Logic Analyser
 *
 * SINGLE SOURCE OF TRUTH — compiled verbatim by both the device firmware and
 * the host tools. Edit only this file; every other copy is synced from it.
 *
 * Portable: compiles on PC (w64devkit / MSVC / GCC) and embedded
 * (ARM Cortex-M4 GCC, RISC-V GCC).
 *
 * Design:
 *   Commands:  PC → Device,  8 bytes fixed, Bulk OUT
 *   Responses: Device → PC,  variable,      Bulk IN
 *
 * Zero-copy on embedded:
 *   DMA fills samples starting at buf + LA_PKT_HEADER_SIZE.
 *   When ready to send, call la_pkt_write_header() to stamp the
 *   header into the reserved space.  Submit buf[0..header+data] to
 *   USB TX as a single contiguous transfer — no memcpy of payload.
 *
 *   ┌───────────┬──────────────────────────────────┐
 *   │ HEADER 8B │ SAMPLE DATA (N bytes)            │
 *   │ (reserved)│ ← DMA writes here                │
 *   └───────────┴──────────────────────────────────┘
 *   ^                                               ^
 *   buf                                       buf + 8 + N
 *   └── submit this whole region to USB TX ────────┘
 *
 * Header layout (8 bytes, naturally aligned):
 *   [0] sync0      0xAA
 *   [1] sync1      0x55
 *   [2] type       LA_RSP_* or LA_DATA_*
 *   [3] flags      bit 0 = RLE compressed, bits 1-7 reserved
 *   [4] len_lo     payload_len low byte   (uint16_t at 4-byte offset)
 *   [5] len_hi     payload_len high byte
 *   [6] seq        wrapping sequence counter (0-255, detects drops)
 *   [7] hdr_csum   XOR of bytes [0..6] — catches corrupted headers
 */

#ifndef LA_PROTOCOL_H
#define LA_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ──────────────────────────── Constants ──────────────────────────── */

#define LA_SYNC_0               0xAA
#define LA_SYNC_1               0x55
#define LA_PKT_HEADER_SIZE      8       /* sync(2) + type(1) + flags(1) + len(2) + seq(1) + csum(1) */
#define LA_CMD_SIZE             8       /* all commands are fixed 8B    */
#define LA_MAX_CHANNELS         16      /* 16 channels (PC0..PC15) when sample_width=16, else 8 */
#define LA_MAX_PAYLOAD          (512 - LA_PKT_HEADER_SIZE)  /* fits one HS USB packet */

/* Header flags (byte 3) */
#define LA_FLAG_RLE             0x01    /* payload is RLE-compressed */
#define LA_FLAG_PACKED          0x02    /* payload is bit-packed (see below) */

/* ──────────────── Bit-packed payloads (LA_FLAG_PACKED) ─────────────
 *
 * Most captures do not use all eight channels: a card drives four pins and the
 * rest sit idle. Idle channels cost nothing in RLE terms -- a constant bit does
 * not break a run -- but they cost a full bit per sample in a raw payload, and
 * raw is exactly what gets sent when the signal changes too fast for RLE. So
 * packing wins precisely where RLE cannot, and the two compose.
 *
 * Layout:
 *
 *   [0] width   bits per packed sample: 1, 2, 4 or 8
 *   [1] mask    which channels are present, bit n = channel n
 *   [2] const   values of the channels NOT in the mask
 *   [3..]       packed samples, first sample in the low bits of each byte
 *
 * Lossless: a channel is left out only when it did not change within this
 * packet, and its value is carried in `const`, so the host reconstructs full
 * 8-bit samples and nothing downstream needs to know packing happened.
 *
 * Width is a power of two so samples never straddle a byte boundary. Arbitrary
 * widths would save a little more at 3, 5, 6 and 7 channels but need a bit
 * accumulator carrying across bytes, and on a device where the encoder is
 * already the throughput limit that is the wrong trade. The channel mask is
 * unrestricted -- non-contiguous is fine -- because compaction is a 256-byte
 * lookup rather than a per-sample bit gather.
 */

/* ──────────────── Command IDs (PC → Device, Bulk OUT) ────────────── */

#define LA_CMD_PING             0x01
#define LA_CMD_GET_CAPS         0x02
#define LA_CMD_SET_RATE         0x10
#define LA_CMD_SET_CHANNELS     0x11
#define LA_CMD_SET_TRIGGER      0x12
#define LA_CMD_SET_DEPTH        0x13
#define LA_CMD_SET_PRETRIG      0x14
#define LA_CMD_SET_FLAGS        0x15    /* param[0] = flag bits (LA_FLAG_RLE, etc.) */
#define LA_CMD_SET_SAMPLE_WIDTH 0x16    /* param[0] = 8 (default) or 16 */
#define LA_CMD_SET_PORT         0x17    /* param[0] = 0 (GPIOC, default) or 1 (GPIOB) */
#define LA_CMD_ARM              0x20
#define LA_CMD_STOP             0x21
#define LA_CMD_FLUSH            0x22
#define LA_CMD_RESET            0xF0

/* ARM mode parameter */
#define LA_ARM_SINGLE           0x00
#define LA_ARM_STREAM           0x01
#define LA_ARM_BLAST_504        0x10    /* USB throughput benchmark: 504-byte payloads */
#define LA_ARM_BLAST_100        0x11    /* USB throughput benchmark: 100-byte payloads */
#define LA_ARM_BLAST_4K         0x12    /* USB throughput benchmark: 4096-byte transfers */

/* Source port parameter (LA_CMD_SET_PORT) */
#define LA_PORT_GPIOC           0x00
#define LA_PORT_GPIOB           0x01

/* Trigger edge parameter */
#define LA_EDGE_NONE            0x00
#define LA_EDGE_RISING          0x01
#define LA_EDGE_FALLING         0x02
#define LA_EDGE_BOTH            0x03

/* ──────────────── Response types (Device → PC, Bulk IN) ──────────── */

#define LA_RSP_PONG             0x01
#define LA_RSP_CAPS             0x02
#define LA_RSP_ACK              0x10
#define LA_RSP_NAK              0x11
#define LA_RSP_TRIGGERED        0x20
#define LA_DATA_SAMPLES         0x80
#define LA_DATA_END             0x81
#define LA_RSP_DEBUG_TEXT       0x82    /* free-form debug text (payload = UTF-8 string) */
#define LA_RSP_OVERFLOW         0xFF

/* NAK error codes */
#define LA_ERR_UNKNOWN_CMD      0x01
#define LA_ERR_BAD_PARAM        0x02
#define LA_ERR_BUSY             0x03
#define LA_ERR_NOT_ARMED        0x04

/* ── USB identifiers (device side) ── */
#define LA_USB_VID              0x1209
#define LA_USB_PID              0x0002
#define LA_USB_EP_OUT           0x01
#define LA_USB_EP_IN            0x81
/* Device Interface GUID: {6895393B-2900-47F4-98C0-76031A56885A} */

/* ── Configuration bitmask (tracks which SET_* commands received) ── */
#define LA_DCFG_RATE            0x01
#define LA_DCFG_CHANNELS        0x02
#define LA_DCFG_TRIGGER         0x04
#define LA_DCFG_DEPTH           0x08
#define LA_DCFG_PRETRIG         0x10
#define LA_DCFG_FLAGS           0x20
#define LA_DCFG_SAMPLE_WIDTH    0x40
#define LA_DCFG_PORT            0x80

/* ──────────────────── Firmware compatibility (the "line") ───────────
 * The device reports its REAL product firmware version (FW_VER_* from
 * fw_version.h) in the PONG. The host tools warn (warn-only — never block)
 * if the connected device is OLDER than the minimum below.
 *
 * LA_MIN_FW_* is the line: raise it ONLY after a breaking change, to the
 * firmware version that first ships the fix. Cosmetic firmware bumps above
 * the line never warn. Older field boards warn until re-flashed.
 *
 * Currently 1.1.0 — the firmware version this codebase ships.
 */
#define LA_MIN_FW_MAJOR         1
#define LA_MIN_FW_MINOR         1
#define LA_MIN_FW_PATCH         0

/* ────────────────────── Wire Structs (packed) ────────────────────── */

/*
 * Use a pragma/attribute that works on GCC (arm, risc-v, x64) and MSVC.
 * We test both; the struct sizes are compile-time asserted below.
 */
#if defined(__GNUC__) || defined(__clang__)
  #define LA_PACKED __attribute__((packed))
#elif defined(_MSC_VER)
  #pragma pack(push, 1)
  #define LA_PACKED
#else
  #define LA_PACKED
#endif

/* ── Command packet (PC → Device): always 8 bytes ── */

typedef struct LA_PACKED {
    uint8_t  cmd;           /* LA_CMD_* */
    uint8_t  param[3];      /* command-specific */
    uint8_t  reserved[4];   /* pad to 8 bytes, zero */
} LaCmd;

/* ── Response/data packet header (Device → PC): always 8 bytes ── */

typedef struct LA_PACKED {
    uint8_t  sync0;         /* LA_SYNC_0 = 0xAA */
    uint8_t  sync1;         /* LA_SYNC_1 = 0x55 */
    uint8_t  type;          /* LA_RSP_* or LA_DATA_* */
    uint8_t  flags;         /* LA_FLAG_* (bit 0 = RLE, rest reserved) */
    uint16_t payload_len;   /* little-endian, bytes of payload following */
    uint8_t  seq;           /* wrapping sequence counter (0-255) */
    uint8_t  hdr_csum;      /* XOR of bytes [0..6] — header integrity */
} LaPktHeader;

/* ── Specific response payloads ── */

typedef struct LA_PACKED {
    uint8_t  version_major;
    uint8_t  version_minor;
    uint8_t  version_patch;
    uint8_t  flags;         /* reserved */
} LaRspPong;                /* 4 bytes, follows LaPktHeader */

typedef struct LA_PACKED {
    uint32_t max_sample_rate;  /* Hz */
    uint8_t  max_channels;
    uint8_t  reserved0;
    uint16_t reserved1;
    uint32_t buffer_size;      /* bytes of on-device sample buffer */
} LaRspCaps;                   /* 12 bytes */

typedef struct LA_PACKED {
    uint8_t  acked_cmd;     /* which command ID this ACKs */
} LaRspAck;                 /* 1 byte */

typedef struct LA_PACKED {
    uint8_t  naked_cmd;     /* which command ID this NAKs */
    uint8_t  error_code;    /* LA_ERR_* */
} LaRspNak;                 /* 2 bytes */

typedef struct LA_PACKED {
    uint32_t trigger_sample; /* sample index where trigger fired */
} LaRspTriggered;            /* 4 bytes */

typedef struct LA_PACKED {
    uint32_t total_samples;  /* total samples sent in this capture */
} LaRspDataEnd;              /* 4 bytes */

typedef struct LA_PACKED {
    uint32_t dropped_count;  /* number of samples lost */
} LaRspOverflow;             /* 4 bytes */

#if defined(_MSC_VER)
  #pragma pack(pop)
#endif

/* ──────────────────── Compile-time size checks ──────────────────── */

#define LA_STATIC_ASSERT(cond, msg) typedef char la_assert_##msg[(cond) ? 1 : -1]

LA_STATIC_ASSERT(sizeof(LaCmd)         == 8,  cmd_size);
LA_STATIC_ASSERT(sizeof(LaPktHeader)   == 8,  header_size);
LA_STATIC_ASSERT(sizeof(LaRspPong)     == 4,  pong_size);
LA_STATIC_ASSERT(sizeof(LaRspCaps)     == 12, caps_size);
LA_STATIC_ASSERT(sizeof(LaRspAck)      == 1,  ack_size);
LA_STATIC_ASSERT(sizeof(LaRspNak)      == 2,  nak_size);
LA_STATIC_ASSERT(sizeof(LaRspTriggered)== 4,  trig_size);
LA_STATIC_ASSERT(sizeof(LaRspDataEnd)  == 4,  end_size);
LA_STATIC_ASSERT(sizeof(LaRspOverflow) == 4,  overflow_size);

/* ─────────────────────────── Inline helpers ──────────────────────────
 * Convenience builders/parsers for the host tools. Gated to C99+/C++
 * because the device firmware compiles under C90 (Keil), where `inline` is
 * not a keyword — and the firmware doesn't use these (it builds packets
 * directly). The wire structs/macros above are the shared contract; these
 * helpers are host sugar.
 */
#if defined(__cplusplus) || (defined(__STDC_VERSION__) && __STDC_VERSION__ >= 199901L)

/* ─────────────── Inline helpers: packet building (TX side) ───────── */

/*
 * la_pkt_write_header()
 *
 * Stamps an 8-byte header into `buf`.  The caller has already placed
 * payload data starting at buf + LA_PKT_HEADER_SIZE.
 *
 * buf:          start of the reserved region (8 bytes before payload)
 * type:         LA_RSP_* or LA_DATA_*
 * flags:        LA_FLAG_* bits (0 for normal, LA_FLAG_RLE for compressed)
 * payload_len:  byte count of the payload that follows the header
 * seq:          sequence counter (caller increments per packet)
 *
 * Returns total packet size (header + payload).
 *
 * Zero-copy usage on embedded:
 *   uint8_t tx_buf[LA_PKT_HEADER_SIZE + 504];  // header + max payload
 *   uint8_t *samples = tx_buf + LA_PKT_HEADER_SIZE;
 *   // DMA fills samples[0..n-1]
 *   uint16_t total = la_pkt_write_header(tx_buf, LA_DATA_SAMPLES, 0, n, seq++);
 *   usb_transmit(tx_buf, total);  // single contiguous transfer
 */
static inline uint16_t la_pkt_write_header(uint8_t *buf, uint8_t type,
                                           uint8_t flags,
                                           uint16_t payload_len,
                                           uint8_t seq)
{
    buf[0] = LA_SYNC_0;
    buf[1] = LA_SYNC_1;
    buf[2] = type;
    buf[3] = flags;
    buf[4] = (uint8_t)(payload_len & 0xFF);         /* len low byte  */
    buf[5] = (uint8_t)((payload_len >> 8) & 0xFF);   /* len high byte */
    buf[6] = seq;
    buf[7] = buf[0] ^ buf[1] ^ buf[2] ^ buf[3] ^ buf[4] ^ buf[5] ^ buf[6];
    return (uint16_t)(LA_PKT_HEADER_SIZE + payload_len);
}

/*
 * la_pkt_write_simple()
 *
 * Build a complete small response packet (header + payload) in one call.
 * For responses like PONG, ACK, NAK, TRIGGERED, DATA_END, OVERFLOW.
 *
 * buf:          output buffer (must be >= LA_PKT_HEADER_SIZE + payload_len)
 * type:         LA_RSP_*
 * payload:      pointer to payload struct (LaRspPong, etc.)
 * payload_len:  sizeof(payload struct)
 *
 * Returns total packet size.
 */
static inline uint16_t la_pkt_write_simple(uint8_t *buf, uint8_t type,
                                           const void *payload,
                                           uint16_t payload_len,
                                           uint8_t seq)
{
    /* Write header */
    la_pkt_write_header(buf, type, 0, payload_len, seq);

    /* Copy small payload after header.
     * On Cortex-M4, for 1-12 byte payloads this compiles to a few
     * LDR/STR instructions — no function call overhead. */
    const uint8_t *src = (const uint8_t *)payload;
    uint8_t *dst = buf + LA_PKT_HEADER_SIZE;
    for (uint16_t i = 0; i < payload_len; i++)
        dst[i] = src[i];

    return (uint16_t)(LA_PKT_HEADER_SIZE + payload_len);
}

/* ─────────────── Inline helpers: command building (PC side) ──────── */

/*
 * la_cmd_build()
 *
 * Fill an 8-byte command buffer.  Zeroes reserved bytes.
 *
 * out:  8-byte output buffer
 * cmd:  LA_CMD_*
 * p0-p2: param[0..2], command-specific
 */
static inline void la_cmd_build(uint8_t *out, uint8_t cmd,
                                uint8_t p0, uint8_t p1, uint8_t p2)
{
    out[0] = cmd;
    out[1] = p0;
    out[2] = p1;
    out[3] = p2;
    out[4] = 0;
    out[5] = 0;
    out[6] = 0;
    out[7] = 0;
}

/* Convenience: build a command directly into a LaCmd struct */
static inline LaCmd la_cmd_make(uint8_t cmd,
                                uint8_t p0, uint8_t p1, uint8_t p2)
{
    LaCmd c;
    c.cmd = cmd;
    c.param[0] = p0;
    c.param[1] = p1;
    c.param[2] = p2;
    c.reserved[0] = 0;
    c.reserved[1] = 0;
    c.reserved[2] = 0;
    c.reserved[3] = 0;
    return c;
}

/* ─── Convenience command builders ─── */

static inline LaCmd la_cmd_ping(void)
{
    return la_cmd_make(LA_CMD_PING, 0, 0, 0);
}

static inline LaCmd la_cmd_get_caps(void)
{
    return la_cmd_make(LA_CMD_GET_CAPS, 0, 0, 0);
}

/*
 * rate_divider: base_clock / desired_rate.
 * 24-bit value packed into param[0..2] little-endian.
 * E.g. for 120 MHz base clock, divider=12 gives 10 MHz sample rate.
 */
static inline LaCmd la_cmd_set_rate(uint32_t rate_divider)
{
    return la_cmd_make(LA_CMD_SET_RATE,
        (uint8_t)(rate_divider & 0xFF),
        (uint8_t)((rate_divider >> 8) & 0xFF),
        (uint8_t)((rate_divider >> 16) & 0xFF));
}

static inline LaCmd la_cmd_set_channels(uint8_t channel_mask)
{
    return la_cmd_make(LA_CMD_SET_CHANNELS, channel_mask, 0, 0);
}

static inline LaCmd la_cmd_set_trigger(uint8_t channel, uint8_t edge,
                                       uint8_t enable)
{
    return la_cmd_make(LA_CMD_SET_TRIGGER, channel, edge, enable);
}

/* depth: 24-bit sample count, little-endian */
static inline LaCmd la_cmd_set_depth(uint32_t sample_count)
{
    return la_cmd_make(LA_CMD_SET_DEPTH,
        (uint8_t)(sample_count & 0xFF),
        (uint8_t)((sample_count >> 8) & 0xFF),
        (uint8_t)((sample_count >> 16) & 0xFF));
}

static inline LaCmd la_cmd_set_pretrig(uint8_t percent)
{
    return la_cmd_make(LA_CMD_SET_PRETRIG,
        (percent > 100) ? 100 : percent, 0, 0);
}

/* flags: bitfield — bit 0 = LA_FLAG_RLE (enable RLE compression), rest reserved */
static inline LaCmd la_cmd_set_flags(uint8_t flags)
{
    return la_cmd_make(LA_CMD_SET_FLAGS, flags, 0, 0);
}

/* sample width in bits: 8 (default, low byte of GPIOC) or 16 (full GPIOC). */
static inline LaCmd la_cmd_set_sample_width(uint8_t width_bits)
{
    return la_cmd_make(LA_CMD_SET_SAMPLE_WIDTH, width_bits, 0, 0);
}

/* source port: LA_PORT_GPIOC (default) or LA_PORT_GPIOB. */
static inline LaCmd la_cmd_set_port(uint8_t port)
{
    return la_cmd_make(LA_CMD_SET_PORT, port, 0, 0);
}

/* mode: LA_ARM_SINGLE or LA_ARM_STREAM */
static inline LaCmd la_cmd_arm(uint8_t mode)
{
    return la_cmd_make(LA_CMD_ARM, mode, 0, 0);
}

static inline LaCmd la_cmd_stop(void)
{
    return la_cmd_make(LA_CMD_STOP, 0, 0, 0);
}

static inline LaCmd la_cmd_flush(void)
{
    return la_cmd_make(LA_CMD_FLUSH, 0, 0, 0);
}

static inline LaCmd la_cmd_reset(void)
{
    return la_cmd_make(LA_CMD_RESET, 0, 0, 0);
}

/* ─────────────── Inline helpers: command parsing (Device side) ───── */

/* Extract 24-bit little-endian value from param[0..2] */
static inline uint32_t la_cmd_param24(const LaCmd *c)
{
    return (uint32_t)c->param[0]
         | ((uint32_t)c->param[1] << 8)
         | ((uint32_t)c->param[2] << 16);
}

/* ─────────────── Inline helpers: packet parsing (PC side) ────────── */

/*
 * la_pkt_find_sync()
 *
 * Scan a byte buffer for the next sync pattern (0xAA 0x55).
 * Returns offset of sync[0], or -1 if not found.
 *
 * Use this to re-synchronise after corruption or partial reads.
 */
static inline int la_pkt_find_sync(const uint8_t *buf, int len)
{
    for (int i = 0; i < len - 1; i++) {
        if (buf[i] == LA_SYNC_0 && buf[i + 1] == LA_SYNC_1)
            return i;
    }
    return -1;
}

/*
 * la_pkt_parse_header()
 *
 * Parse an 8-byte header at buf[0..7].
 * Returns 1 if valid (sync matches AND checksum OK), 0 if invalid.
 * On success, fills *type, *flags, *payload_len, *seq.
 * Pass NULL for any output you don't need.
 */
static inline int la_pkt_parse_header(const uint8_t *buf,
                                      uint8_t *type,
                                      uint8_t *flags,
                                      uint16_t *payload_len,
                                      uint8_t *seq)
{
    if (buf[0] != LA_SYNC_0 || buf[1] != LA_SYNC_1)
        return 0;

    /* Verify header checksum */
    uint8_t csum = buf[0] ^ buf[1] ^ buf[2] ^ buf[3]
                 ^ buf[4] ^ buf[5] ^ buf[6];
    if (csum != buf[7])
        return 0;

    if (type)        *type        = buf[2];
    if (flags)       *flags       = buf[3];
    if (payload_len) *payload_len = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
    if (seq)         *seq         = buf[6];
    return 1;
}

#endif /* C99+ / C++ : inline helpers (excluded from the C90 firmware build) */

#ifdef __cplusplus
}
#endif

#endif /* LA_PROTOCOL_H */
