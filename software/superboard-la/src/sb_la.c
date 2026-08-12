/*
 * sb_la.c — host for the Superboard logic analyser.
 *
 * Speaks the wire protocol in la_protocol.h over WinUSB, and writes samples to
 * a file or stdout.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Zolex Labs Ltd
 */

#include <windows.h>
#include <setupapi.h>
#include <cfgmgr32.h>
#include <winusb.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <io.h>
#include <fcntl.h>

#include "la_protocol.h"

/* Device interface GUID advertised by the board's MS OS descriptors.
 * Also in la_protocol.h as a comment; spelled out here for SetupAPI. */
static const GUID SB_LA_GUID =
    { 0x6895393B, 0x2900, 0x47F4,
      { 0x98, 0xC0, 0x76, 0x03, 0x1A, 0x56, 0x88, 0x5A } };

#define SB_PIPE_TIMEOUT_MS   1000
#define SB_RX_CHUNK          16384   /* bytes per ReadPipe */
#define SB_RX_BUF            65536   /* reassembly buffer  */
#define SB_EXPAND_SLICE      4096    /* RLE/packed expansion slice */

/* ───────────────────────────── Device ───────────────────────────── */

typedef struct {
    HANDLE                  file;
    WINUSB_INTERFACE_HANDLE usb;
    char                    path[512];
    char                    serial[64];
} SbDev;

/* Truncating copy that always NUL-terminates. */
static void copy_str(char *dst, size_t n, const char *src)
{
    size_t i = 0;
    if (!n) return;
    for (; i + 1 < n && src[i]; i++)
        dst[i] = src[i];
    dst[i] = '\0';
}

/*
 * The USB serial number lives on the composite parent, not on the interface
 * devnode we enumerated, so walk up one level: the parent instance ID looks
 * like "USB\VID_1209&PID_0002\9482DF6C0516" and the serial is the tail.
 */
static void devnode_serial(DEVINST inst, char *out, size_t outlen)
{
    DEVINST parent = 0;
    char    id[256];

    out[0] = '\0';
    if (CM_Get_Parent(&parent, inst, 0) != CR_SUCCESS)
        return;
    if (CM_Get_Device_IDA(parent, id, (ULONG)sizeof(id), 0) != CR_SUCCESS)
        return;

    const char *bs = strrchr(id, '\\');
    copy_str(out, outlen, bs ? bs + 1 : id);
}

static void str_lower(char *s)
{
    for (; *s; s++)
        if (*s >= 'A' && *s <= 'Z') *s += 32;
}

/*
 * Walk every present interface carrying our GUID.
 *
 * dev == NULL  → list mode: print each match, return the count.
 * dev != NULL  → open the first match, return 1, or 0 if none.
 *
 * want_serial (may be NULL/empty) narrows to one board when several are
 * plugged in.
 */
static int enumerate(uint16_t vid, uint16_t pid, const char *want_serial,
                     SbDev *dev)
{
    HDEVINFO set = SetupDiGetClassDevsA(&SB_LA_GUID, NULL, NULL,
                                        DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    if (set == INVALID_HANDLE_VALUE)
        return 0;

    char vid_s[16], pid_s[16];
    snprintf(vid_s, sizeof(vid_s), "vid_%04x", vid);
    snprintf(pid_s, sizeof(pid_s), "pid_%04x", pid);

    SP_DEVICE_INTERFACE_DATA ifd;
    ifd.cbSize = sizeof(ifd);

    int found = 0;

    for (DWORD i = 0; SetupDiEnumDeviceInterfaces(set, NULL, &SB_LA_GUID, i,
                                                  &ifd); i++)
    {
        DWORD need = 0;
        SetupDiGetDeviceInterfaceDetailA(set, &ifd, NULL, 0, &need, NULL);
        if (!need) continue;

        SP_DEVICE_INTERFACE_DETAIL_DATA_A *det =
            (SP_DEVICE_INTERFACE_DETAIL_DATA_A *)malloc(need);
        if (!det) continue;
        det->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_A);

        SP_DEVINFO_DATA dd;
        dd.cbSize = sizeof(dd);

        if (SetupDiGetDeviceInterfaceDetailA(set, &ifd, det, need, NULL, &dd)) {
            char low[512];
            copy_str(low, sizeof(low), det->DevicePath);
            str_lower(low);

            /* Composite device: MI_00 is the analyser, MI_01 the CMSIS-DAP
             * probe. Only MI_00 is ours, so a debugger can hold the DAP
             * interface at the same time. */
            int ours = strstr(low, vid_s) && strstr(low, pid_s)
                    && !strstr(low, "mi_01") && !strstr(low, "mi_02")
                    && !strstr(low, "mi_03");

            if (ours) {
                char ser[64];
                devnode_serial(dd.DevInst, ser, sizeof(ser));

                int wanted = !(want_serial && want_serial[0])
                          || _stricmp(ser, want_serial) == 0;

                if (wanted && !dev) {
                    printf("  %-16s  %s\n",
                           ser[0] ? ser : "(no serial)", det->DevicePath);
                    found++;
                }
                else if (wanted && dev) {
                    HANDLE h = CreateFileA(det->DevicePath,
                                           GENERIC_READ | GENERIC_WRITE,
                                           FILE_SHARE_READ | FILE_SHARE_WRITE,
                                           NULL, OPEN_EXISTING,
                                           FILE_ATTRIBUTE_NORMAL |
                                           FILE_FLAG_OVERLAPPED, NULL);
                    if (h != INVALID_HANDLE_VALUE) {
                        dev->file = h;
                        copy_str(dev->path,   sizeof(dev->path),   det->DevicePath);
                        copy_str(dev->serial, sizeof(dev->serial), ser);
                        free(det);
                        found = 1;
                        break;
                    }
                }
            }
        }
        free(det);
    }

    SetupDiDestroyDeviceInfoList(set);
    return found;
}

static int dev_open(SbDev *dev, uint16_t vid, uint16_t pid, const char *serial)
{
    memset(dev, 0, sizeof(*dev));
    dev->file = INVALID_HANDLE_VALUE;

    if (!enumerate(vid, pid, serial, dev))
        return -1;

    if (!WinUsb_Initialize(dev->file, &dev->usb)) {
        CloseHandle(dev->file);
        dev->file = INVALID_HANDLE_VALUE;
        return -2;
    }

    /* Bounded reads: without this a ReadPipe on an idle device blocks for
     * ever, and "no data yet" is a normal state between packets. */
    ULONG timeout = SB_PIPE_TIMEOUT_MS;
    WinUsb_SetPipePolicy(dev->usb, LA_USB_EP_IN, PIPE_TRANSFER_TIMEOUT,
                         sizeof(timeout), &timeout);
    return 0;
}

static void dev_close(SbDev *dev)
{
    if (dev->usb)  WinUsb_Free(dev->usb);
    if (dev->file != INVALID_HANDLE_VALUE) CloseHandle(dev->file);
    dev->usb  = NULL;
    dev->file = INVALID_HANDLE_VALUE;
}

/* Commands are eight bytes, fixed, on the bulk OUT endpoint. */
static int dev_cmd(SbDev *dev, LaCmd c)
{
    ULONG sent = 0;
    if (!WinUsb_WritePipe(dev->usb, LA_USB_EP_OUT, (PUCHAR)&c,
                          LA_CMD_SIZE, &sent, NULL))
        return -1;
    return sent == LA_CMD_SIZE ? 0 : -1;
}

/* ───────────────────────────── Options ──────────────────────────── */

static uint16_t  opt_vid    = LA_USB_VID;
static uint16_t  opt_pid    = LA_USB_PID;
static const char *opt_serial = NULL;
static uint32_t  opt_rate   = 1000000;
static uint64_t  opt_count  = 0;         /* 0 = until Ctrl-C / --time */
static double    opt_time   = 0.0;
static const char *opt_out  = NULL;
static int       opt_csv    = 0;
static int       opt_rle    = 0;
static int       opt_width  = 8;
static uint8_t   opt_port   = LA_PORT_GPIOC;
static int       opt_info   = 0;
static int       opt_list   = 0;

/* ───────────────────────────── Sample sink ──────────────────────── */

static FILE     *sink_fp;
static uint64_t  sink_count;      /* samples written — not bytes */
static volatile int stop_flag;

/* One sample, however wide. CSV gets a row of per-channel bits; raw gets the
 * sample little-endian, which for 8-bit is the byte straight through. */
static void emit(uint32_t v)
{
    if (opt_csv) {
        fprintf(sink_fp, "%llu", (unsigned long long)sink_count);
        for (int c = 0; c < opt_width; c++)
            fprintf(sink_fp, ",%d", (int)((v >> c) & 1));
        fputc('\n', sink_fp);
    } else {
        fputc((int)(v & 0xFF), sink_fp);
        if (opt_width == 16) fputc((int)((v >> 8) & 0xFF), sink_fp);
    }

    sink_count++;
    if (opt_count && sink_count >= opt_count)
        stop_flag = 1;
}

/*
 * Sample bytes arrive here from all three payload forms. At 16 channels the
 * device sends two bytes per sample, little-endian, and a packet boundary can
 * fall between them — so the odd byte is carried across calls rather than
 * assumed to be the start of a pair.
 */
static void sink_samples(const uint8_t *s, uint32_t n)
{
    static uint8_t pending;
    static int     have_pending;

    if (opt_width == 16) {
        for (uint32_t i = 0; i < n; i++) {
            if (!have_pending) { pending = s[i]; have_pending = 1; continue; }
            emit((uint32_t)pending | ((uint32_t)s[i] << 8));
            have_pending = 0;
        }
    } else {
        for (uint32_t i = 0; i < n; i++)
            emit(s[i]);
    }
}

/* ── RLE expansion ───────────────────────────────────────────────────
 *
 * [run_length 2B LE][value 1B] per run, so three bytes carry up to 65535
 * identical samples. Expanded in slices rather than into one buffer: a
 * single 504-byte payload can legitimately describe 168 * 65535 samples.
 */
static void expand_rle(const uint8_t *p, uint16_t len)
{
    static uint8_t slice[SB_EXPAND_SLICE];
    uint32_t used = 0;

    /* A trailing partial triple means a torn packet — stop rather than
     * invent samples from it. */
    for (uint16_t i = 0; i + 3 <= len; i += 3) {
        uint32_t run = (uint32_t)p[i] | ((uint32_t)p[i + 1] << 8);
        uint8_t  val = p[i + 2];

        while (run) {
            uint32_t space = SB_EXPAND_SLICE - used;
            uint32_t n     = run < space ? run : space;
            memset(slice + used, val, n);
            used += n;
            run  -= n;
            if (used == SB_EXPAND_SLICE) { sink_samples(slice, used); used = 0; }
        }
    }
    if (used) sink_samples(slice, used);
}

/* ── Bit-packed expansion ────────────────────────────────────────────
 *
 * Layout is in la_protocol.h. Channels that did not change within the packet
 * are left out of the payload and carried as constant bits; full 8-bit
 * samples are rebuilt here.
 */
static void expand_packed(const uint8_t *p, uint16_t len)
{
    if (len < 3) return;

    uint8_t width = p[0], mask = p[1], konst = p[2];
    if (width != 1 && width != 2 && width != 4 && width != 8) return;

    /* Scatter table: packed value → the mask's bits back in their places.
     * Rebuilt when the mask changes. */
    static uint8_t scatter[256];
    static int     valid;
    static uint8_t built_for;

    if (!valid || built_for != mask) {
        for (int v = 0; v < 256; v++) {
            uint8_t out = 0;
            int     src = 0;
            for (int b = 0; b < 8; b++)
                if (mask & (1u << b)) {
                    if (v & (1u << src)) out |= (uint8_t)(1u << b);
                    src++;
                }
            scatter[v] = out;
        }
        built_for = mask;
        valid     = 1;
    }

    static uint8_t slice[SB_EXPAND_SLICE];
    uint32_t used     = 0;
    uint8_t  per_byte = (uint8_t)(8 / width);
    uint8_t  vmask    = (uint8_t)((1u << width) - 1u);

    for (uint16_t i = 3; i < len; i++) {
        for (uint8_t k = 0; k < per_byte; k++) {
            uint8_t v = (uint8_t)((p[i] >> (k * width)) & vmask);
            slice[used++] = (uint8_t)(scatter[v] | konst);
            if (used == SB_EXPAND_SLICE) { sink_samples(slice, used); used = 0; }
        }
    }
    if (used) sink_samples(slice, used);
}

/* ───────────────────────── Packet reassembly ────────────────────── */

static uint8_t  rx[SB_RX_BUF];
static size_t   rx_len;

/* Device state learned from responses */
static int      got_pong, got_caps, got_end;
static LaRspPong pong;
static LaRspCaps caps;
static uint32_t  overflow_total;
static uint8_t   last_seq;
static int       seq_started;
static uint64_t  seq_gaps;

static void handle_packet(uint8_t type, uint8_t flags, uint8_t seq,
                          const uint8_t *payload, uint16_t len)
{
    /* The sequence counter is the only drop detector on the wire — the
     * device stamps it per packet and it wraps at 256. */
    if (type == LA_DATA_SAMPLES) {
        if (seq_started && (uint8_t)(last_seq + 1) != seq)
            seq_gaps++;
        last_seq    = seq;
        seq_started = 1;
    }

    switch (type) {
    case LA_RSP_PONG:
        if (len >= sizeof(pong)) { memcpy(&pong, payload, sizeof(pong));
                                   got_pong = 1; }
        break;

    case LA_RSP_CAPS:
        if (len >= sizeof(caps)) { memcpy(&caps, payload, sizeof(caps));
                                   got_caps = 1; }
        break;

    case LA_DATA_SAMPLES:
        if (flags & LA_FLAG_PACKED)   expand_packed(payload, len);
        else if (flags & LA_FLAG_RLE) expand_rle(payload, len);
        else                          sink_samples(payload, len);
        break;

    case LA_DATA_END:
        got_end = 1;
        break;

    case LA_RSP_OVERFLOW:
        if (len >= 4) {
            uint32_t d;
            memcpy(&d, payload, 4);
            overflow_total += d;
        }
        break;

    case LA_RSP_DEBUG_TEXT:
        fprintf(stderr, "[dev] %.*s\n", (int)len, (const char *)payload);
        break;

    case LA_RSP_NAK:
        if (len >= 2)
            fprintf(stderr, "NAK: cmd 0x%02X error 0x%02X\n",
                    payload[0], payload[1]);
        break;

    default:
        break;   /* ACK, TRIGGERED and anything newer: nothing to do here */
    }
}

/* Consume every complete packet sitting in rx[], resynchronising on the
 * 0xAA 0x55 pattern if the stream is ever torn. */
static void drain_packets(void)
{
    size_t pos = 0;

    for (;;) {
        if (rx_len - pos < LA_PKT_HEADER_SIZE) break;

        uint8_t  type, flags, seq;
        uint16_t plen;

        if (!la_pkt_parse_header(rx + pos, &type, &flags, &plen, &seq)) {
            /* Bad header: hunt for the next sync pair and try again. */
            int off = la_pkt_find_sync(rx + pos + 1,
                                       (int)(rx_len - pos - 1));
            if (off < 0) { pos = rx_len; break; }
            pos += 1 + (size_t)off;
            continue;
        }

        if (rx_len - pos < (size_t)LA_PKT_HEADER_SIZE + plen)
            break;                      /* tail of a packet still in flight */

        handle_packet(type, flags, seq, rx + pos + LA_PKT_HEADER_SIZE, plen);
        pos += LA_PKT_HEADER_SIZE + plen;
    }

    if (pos) {
        memmove(rx, rx + pos, rx_len - pos);
        rx_len -= pos;
    }
}

/* One blocking read, bounded by the pipe timeout. Returns bytes read,
 * 0 on timeout (normal), -1 on a real failure. */
static int pump(SbDev *dev)
{
    if (rx_len + SB_RX_CHUNK > SB_RX_BUF)
        rx_len = 0;                     /* defensive: packets are far smaller */

    ULONG got = 0;
    BOOL  ok  = WinUsb_ReadPipe(dev->usb, LA_USB_EP_IN,
                                rx + rx_len, SB_RX_CHUNK, &got, NULL);
    if (!ok) {
        DWORD e = GetLastError();
        if (e == ERROR_SEM_TIMEOUT) return 0;
        return -1;
    }

    rx_len += got;
    drain_packets();
    return (int)got;
}

/* ───────────────────────────── Ctrl-C ───────────────────────────── */

static BOOL WINAPI on_break(DWORD type)
{
    (void)type;
    stop_flag = 1;
    return TRUE;
}

/* ───────────────────────────── Arguments ────────────────────────── */

static void usage(void)
{
    fprintf(stderr,
        "sb_la — minimal reference host for the Superboard logic analyser\n"
        "\n"
        "Usage: sb_la [options]\n"
        "\n"
        "  --list          List connected boards and exit\n"
        "  --info          Print device capabilities and exit\n"
        "  --serial S      Select the board with USB serial S\n"
        "  --vid V         USB vendor ID  (default 0x%04X)\n"
        "  --pid P         USB product ID (default 0x%04X)\n"
        "  -r RATE         Sample rate in Hz (default 1000000)\n"
        "  -n COUNT        Stop after COUNT samples\n"
        "  -t SECONDS      Stop after SECONDS\n"
        "  -o FILE         Write to FILE (default stdout)\n"
        "  -f raw|csv      Output format (default raw)\n"
        "  --rle           Permit compressed packets on the wire (8-bit only)\n"
        "  --width 8|16    Channels sampled (default 8)\n"
        "  --port c|b      Source port GPIOC (default) or GPIOB\n"
        "  -h, --help      This help\n"
        "\n"
        "Examples:\n"
        "  sb_la --list\n"
        "  sb_la --info\n"
        "  sb_la -r 2000000 -n 1000000 -o cap.bin\n"
        "  sb_la -t 2 -f csv -o cap.csv\n"
        , LA_USB_VID, LA_USB_PID);
}

/* Accepts 0x1209 and 4617 alike; rejects trailing junk. */
static int parse_u16(const char *s, uint16_t *out)
{
    char *end = NULL;
    unsigned long v = strtoul(s, &end, 0);
    if (end == s || *end || v > 0xFFFFUL) return -1;
    *out = (uint16_t)v;
    return 0;
}

/* Accepts a K/M/G suffix. */
static int parse_count(const char *s, uint64_t *out)
{
    char *end = NULL;
    unsigned long long v = strtoull(s, &end, 0);
    if (end == s) return -1;
    if      (*end == 'k' || *end == 'K') { v *= 1000ULL;       end++; }
    else if (*end == 'm' || *end == 'M') { v *= 1000000ULL;    end++; }
    else if (*end == 'g' || *end == 'G') { v *= 1000000000ULL; end++; }
    if (*end) return -1;
    *out = v;
    return 0;
}

static int parse_args(int argc, char **argv)
{
    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        int has_next = i + 1 < argc;

        if      (!strcmp(a, "-h") || !strcmp(a, "--help")) { usage(); exit(0); }
        else if (!strcmp(a, "--list")) opt_list = 1;
        else if (!strcmp(a, "--info")) opt_info = 1;
        else if (!strcmp(a, "--rle"))  opt_rle  = 1;
        else if (!strcmp(a, "--serial") && has_next) opt_serial = argv[++i];
        else if (!strcmp(a, "-o")      && has_next) opt_out    = argv[++i];
        else if (!strcmp(a, "--vid") && has_next) {
            if (parse_u16(argv[++i], &opt_vid)) {
                fprintf(stderr, "--vid must be a 16-bit USB ID\n"); return -1; }
        }
        else if (!strcmp(a, "--pid") && has_next) {
            if (parse_u16(argv[++i], &opt_pid)) {
                fprintf(stderr, "--pid must be a 16-bit USB ID\n"); return -1; }
        }
        else if (!strcmp(a, "-r") && has_next) {
            uint64_t v;
            if (parse_count(argv[++i], &v) || v == 0 || v > 0xFFFFFFFFULL) {
                fprintf(stderr, "-r must be a sample rate in Hz\n"); return -1; }
            opt_rate = (uint32_t)v;
        }
        else if (!strcmp(a, "-n") && has_next) {
            if (parse_count(argv[++i], &opt_count)) {
                fprintf(stderr, "-n must be a sample count\n"); return -1; }
        }
        else if (!strcmp(a, "-t") && has_next) {
            opt_time = atof(argv[++i]);
            if (opt_time <= 0) {
                fprintf(stderr, "-t must be seconds > 0\n"); return -1; }
        }
        else if (!strcmp(a, "-f") && has_next) {
            const char *f = argv[++i];
            if      (!strcmp(f, "csv")) opt_csv = 1;
            else if (!strcmp(f, "raw")) opt_csv = 0;
            else { fprintf(stderr, "-f must be raw or csv\n"); return -1; }
        }
        else if (!strcmp(a, "--width") && has_next) {
            opt_width = atoi(argv[++i]);
            if (opt_width != 8 && opt_width != 16) {
                fprintf(stderr, "--width must be 8 or 16\n"); return -1; }
        }
        else if (!strcmp(a, "--port") && has_next) {
            const char *p = argv[++i];
            if      (*p == 'c' || *p == 'C') opt_port = LA_PORT_GPIOC;
            else if (*p == 'b' || *p == 'B') opt_port = LA_PORT_GPIOB;
            else { fprintf(stderr, "--port must be c or b\n"); return -1; }
        }
        else {
            fprintf(stderr, "Unknown option: %s\n", a);
            usage();
            return -1;
        }
    }

    /* The packed format carries an 8-bit channel mask and RLE runs are
     * byte-valued, so both are 8-bit constructs. Rather than half-support
     * them at 16 channels, say so. */
    if (opt_rle && opt_width != 8) {
        fprintf(stderr, "--rle applies to 8-bit captures only\n");
        return -1;
    }
    return 0;
}

/* ───────────────────────────── Main ─────────────────────────────── */

/* Read for up to ms milliseconds or until `done` goes true. */
static void pump_until(SbDev *dev, const int *done, int ms)
{
    DWORD deadline = GetTickCount() + (DWORD)ms;
    while ((int)(deadline - GetTickCount()) > 0) {
        if (done && *done) return;
        if (pump(dev) < 0) return;
    }
}

int main(int argc, char **argv)
{
    if (parse_args(argc, argv) < 0)
        return 1;

    if (opt_list) {
        printf("Connected boards:\n");
        int n = enumerate(opt_vid, opt_pid, opt_serial, NULL);
        if (!n) printf("  (none — is a board plugged in?)\n");
        return 0;
    }

    SbDev dev;
    int rc = dev_open(&dev, opt_vid, opt_pid, opt_serial);
    if (rc < 0) {
        fprintf(stderr,
                rc == -1 ? "No board found at %04X:%04X%s%s. (try --list)\n"
                         : "Found the board at %04X:%04X%s%s "
                           "but WinUSB would not open it.\n",
                opt_vid, opt_pid,
                opt_serial ? " serial " : "", opt_serial ? opt_serial : "");
        return 1;
    }
    fprintf(stderr, "Opened %04X:%04X serial %s\n",
            opt_vid, opt_pid, dev.serial[0] ? dev.serial : "(none)");

    /* Handshake: PING for the firmware version, GET_CAPS for the limits.
     * The sample rate is set as a divider of the device's base clock, which
     * is what max_sample_rate reports. */
    dev_cmd(&dev, la_cmd_ping());
    pump_until(&dev, &got_pong, 500);
    dev_cmd(&dev, la_cmd_get_caps());
    pump_until(&dev, &got_caps, 500);

    if (!got_pong || !got_caps) {
        fprintf(stderr, "Device did not answer PING/GET_CAPS.\n");
        dev_close(&dev);
        return 1;
    }

    if (opt_info) {
        printf("VID:PID            %04X:%04X\n", opt_vid, opt_pid);
        printf("Serial             %s\n", dev.serial[0] ? dev.serial : "(none)");
        printf("Firmware           %u.%u.%u  (host expects >= %u.%u.%u)\n",
               pong.version_major, pong.version_minor, pong.version_patch,
               LA_MIN_FW_MAJOR, LA_MIN_FW_MINOR, LA_MIN_FW_PATCH);
        printf("Base sample clock  %u Hz\n", caps.max_sample_rate);
        printf("Max channels       %u\n", caps.max_channels);
        printf("Device buffer      %u bytes\n", caps.buffer_size);
        printf("Device path        %s\n", dev.path);
        dev_close(&dev);
        return 0;
    }

    if (pong.version_major < LA_MIN_FW_MAJOR ||
        (pong.version_major == LA_MIN_FW_MAJOR &&
         pong.version_minor < LA_MIN_FW_MINOR))
        fprintf(stderr, "Warning: firmware %u.%u.%u is older than %u.%u.%u — "
                        "some features may be missing.\n",
                pong.version_major, pong.version_minor, pong.version_patch,
                LA_MIN_FW_MAJOR, LA_MIN_FW_MINOR, LA_MIN_FW_PATCH);

    uint32_t divider = caps.max_sample_rate ? caps.max_sample_rate / opt_rate
                                            : 24;
    if (!divider) divider = 1;
    uint32_t actual = caps.max_sample_rate ? caps.max_sample_rate / divider
                                           : opt_rate;
    if (actual != opt_rate)
        fprintf(stderr, "Rate %u Hz is not an exact divider of %u Hz — "
                        "using %u Hz.\n",
                opt_rate, caps.max_sample_rate, actual);

    /* Output */
    if (opt_out) {
        sink_fp = fopen(opt_out, opt_csv ? "w" : "wb");
        if (!sink_fp) {
            fprintf(stderr, "Cannot open %s for writing.\n", opt_out);
            dev_close(&dev);
            return 1;
        }
    } else {
        sink_fp = stdout;
        if (!opt_csv) _setmode(_fileno(stdout), _O_BINARY);
    }

    if (opt_csv) {
        fprintf(sink_fp, "sample");
        for (int c = 0; c < opt_width; c++)
            fprintf(sink_fp, ",ch%d", c);
        fputc('\n', sink_fp);
    }

    /* Configure, then arm. Each SET_* is ACKed; we do not gate on the ACKs
     * here beyond draining them, which is why the short pump follows. */
    dev_cmd(&dev, la_cmd_set_rate(divider));
    dev_cmd(&dev, la_cmd_set_channels(0xFF));
    dev_cmd(&dev, la_cmd_set_sample_width((uint8_t)opt_width));
    dev_cmd(&dev, la_cmd_set_port(opt_port));
    dev_cmd(&dev, la_cmd_set_trigger(0, LA_EDGE_NONE, 0));
    dev_cmd(&dev, la_cmd_set_flags(opt_rle ? LA_FLAG_RLE : 0));
    pump_until(&dev, NULL, 150);

    SetConsoleCtrlHandler(on_break, TRUE);

    if (dev_cmd(&dev, la_cmd_arm(LA_ARM_STREAM)) < 0) {
        fprintf(stderr, "ARM failed.\n");
        dev_close(&dev);
        return 1;
    }
    fprintf(stderr, "Capturing at %u Hz — Ctrl-C to stop.\n", actual);

    DWORD  t0       = GetTickCount();
    DWORD  deadline = opt_time > 0 ? t0 + (DWORD)(opt_time * 1000.0) : 0;
    DWORD  last_report = t0;

    while (!stop_flag && !got_end) {
        if (pump(&dev) < 0) {
            fprintf(stderr, "USB read failed (device unplugged?).\n");
            break;
        }
        DWORD now = GetTickCount();
        if (deadline && (int)(deadline - now) <= 0) break;

        if (now - last_report >= 1000) {
            double secs = (now - t0) / 1000.0;
            fprintf(stderr, "\r%llu samples  %.2f MS/s  ",
                    (unsigned long long)sink_count,
                    secs > 0 ? sink_count / secs / 1e6 : 0.0);
            fflush(stderr);
            last_report = now;
        }
    }

    dev_cmd(&dev, la_cmd_stop());
    pump_until(&dev, NULL, 200);        /* collect the tail + DATA_END */

    double secs = (GetTickCount() - t0) / 1000.0;
    fprintf(stderr, "\r%llu samples in %.2f s (%.2f MS/s)\n",
            (unsigned long long)sink_count, secs,
            secs > 0 ? sink_count / secs / 1e6 : 0.0);
    if (seq_gaps)
        fprintf(stderr, "Warning: %llu packet sequence gaps — samples were "
                        "lost in transit.\n", (unsigned long long)seq_gaps);
    if (overflow_total)
        fprintf(stderr, "Warning: device reported %u dropped samples "
                        "(host too slow).\n", overflow_total);

    if (sink_fp && sink_fp != stdout) fclose(sink_fp);
    dev_close(&dev);
    return 0;
}
