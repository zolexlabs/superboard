/**
  **************************************************************************
  * @file     la_hardware.c
  * @brief    Logic Analyzer Hardware Capture Engine (AT32F405-specific)
  * @version  4.0
  * @date     2026-03
  **************************************************************************
  * Proto 1 wiring:  RP2040 GPIO 0-7  ->  AT32 PC0-PC7
  *
  * DMA-based acquisition with DTCNT polling (no per-sample ISR):
  *   TMR2 overflow -> DMAMUX -> DMA1_Channel1 reads GPIOC->IDT byte
  *   into a 32 KB circular ring buffer.
  *
  *   The main loop polls the DMA DTCNT register directly to determine
  *   how far DMA has written.  This gives sample-level granularity
  *   with zero interrupt overhead for normal operation.
  *
  *   A single FDT (full-transfer) interrupt fires once per complete
  *   ring cycle as a lap counter for overflow detection.
  *
  * Ring buffer layout:
  *   DMA hardware auto-increments its write pointer 0..N-1 and wraps.
  *   DTCNT counts DOWN from N to 1, then reloads to N.
  *   dma_write_pos = N - DTCNT  (ring index 0..N-1)
  *   g_rd_pos chases behind, draining data to USB.
  *
  * Overflow: if DMA completes a full lap before the reader drains it,
  *   samples are silently overwritten.  The lap counter detects this.
  **************************************************************************
  */

#include <stddef.h>
#include <string.h>
#include "la_device.h"
#include "at32f402_405.h"

/* ======================================================================
 * Hardware Configuration
 * ====================================================================== */

#define LA_GPIO_PORT            GPIOC
#define LA_GPIO_CLOCK           CRM_GPIOC_PERIPH_CLOCK
#define LA_GPIO_PINS            (GPIO_PINS_0 | GPIO_PINS_1 | GPIO_PINS_2 | GPIO_PINS_3 | \
                                 GPIO_PINS_4 | GPIO_PINS_5 | GPIO_PINS_6 | GPIO_PINS_7)

#define LA_TIMER                TMR2
#define LA_TIMER_CLOCK          CRM_TMR2_PERIPH_CLOCK
#define LA_TIMER_INPUT_FREQ     216000000   /* APB1*2 = 216 MHz */

/* DMA configuration: DMA1 Channel 1, routed via DMAMUX to TMR2 overflow */
#define LA_DMA                  DMA1
#define LA_DMA_CLOCK            CRM_DMA1_PERIPH_CLOCK
#define LA_DMA_CHANNEL          DMA1_CHANNEL1
#define LA_DMA_MUX_CHANNEL      DMA1MUX_CHANNEL1
#define LA_DMA_MUX_REQ          DMAMUX_DMAREQ_ID_TMR2_OVERFLOW
#define LA_DMA_IRQn             DMA1_Channel1_IRQn
#define LA_DMA_HDT_FLAG         DMA1_HDT1_FLAG
#define LA_DMA_FDT_FLAG         DMA1_FDT1_FLAG

/*
 * Ring buffer — power of 2 for fast masking.
 * 32 KB gives 1.37 ms of headroom at 24 MHz max rate.
 */
#define LA_HW_RING_SIZE         (32 * 1024)
#define LA_HW_RING_MASK         (LA_HW_RING_SIZE - 1)

static uint8_t g_ring[LA_HW_RING_SIZE] __attribute__((aligned(4)));

/* Reader state */
static uint32_t g_rd_pos;              /* ring index 0..N-1, next byte to read */
static uint32_t g_rd_laps;            /* full laps consumed by reader */

/* Bytes per sample (1 = 8-bit GPIOC->IDT low byte, 2 = 16-bit full word).
 * Set by la_hw_start_capture from dev->config.sample_width. The DMA's
 * buffer_size is in transfers, so transfers per lap = RING_SIZE / g_sample_bytes. */
static uint8_t  g_sample_bytes = 1;
static uint32_t g_dma_xfers    = LA_HW_RING_SIZE;

/* Writer state (FDT ISR only touches g_wr_laps) */
static volatile uint32_t g_wr_laps;   /* full laps completed by DMA */

/* Data-ready flag: set by HDT/FDT ISR, cleared by reader.
 * Signals the main loop that a big chunk (~16 KB) is available. */
volatile uint32_t g_data_ready;

/* Diagnostics (exported to main.c) */
volatile uint32_t g_dma_overflow_count;  /* overflow events detected */
volatile uint32_t g_dma_fdt_count;       /* FDT interrupts (= laps) */
volatile uint32_t g_dma_hdt_count;       /* HDT interrupts */

/* ======================================================================
 * Internal: get consistent DMA write position
 *
 * Returns the total number of bytes DMA has written since capture start,
 * as (laps * RING_SIZE + write_pos).  Uses a retry loop to handle the
 * race where FDT fires between reading laps and DTCNT.
 * ====================================================================== */

static void la_hw_get_write_total(uint32_t *out_laps, uint32_t *out_pos)
{
    uint32_t laps, dtcnt, pos;
    do {
        laps  = g_wr_laps;
        dtcnt = (uint32_t)dma_data_number_get(LA_DMA_CHANNEL);
        /* DTCNT counts xfers N..1; at reload it goes back to N.
         * Each xfer is g_sample_bytes bytes, so byte position = (N-DTCNT)*bytes.
         * If DTCNT == N that means pos=0 (just wrapped).
         * If DTCNT == 0 the DMA is idle (not running). */
        pos = (g_dma_xfers - dtcnt) * g_sample_bytes;
        if (pos >= LA_HW_RING_SIZE) {
            pos = 0;  /* handle DTCNT==N after reload */
        }
    } while (laps != g_wr_laps);  /* retry if FDT fired during read */

    *out_laps = laps;
    *out_pos  = pos;
}

/* ======================================================================
 * GPIO Init (always runs at boot)
 * ====================================================================== */

void la_hw_init(LaDevice *dev)
{
    gpio_init_type gpio_init_struct;
    (void)dev;

    g_rd_pos  = 0;
    g_rd_laps = 0;
    g_wr_laps = 0;

    /* Enable GPIOC clock */
    crm_periph_clock_enable(LA_GPIO_CLOCK, TRUE);

    /* Configure PC0-PC15 as floating inputs (high-Z, target drives).
     * All 16 pins are init'd unconditionally; the active sample width (8 vs 16)
     * just decides whether DMA reads the low byte or the full halfword. */
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_pins = GPIO_PINS_ALL;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(LA_GPIO_PORT, &gpio_init_struct);

    /* Enable TMR2 clock — timer is configured on ARM */
    crm_periph_clock_enable(LA_TIMER_CLOCK, TRUE);

    /* Enable DMA1 clock */
    crm_periph_clock_enable(LA_DMA_CLOCK, TRUE);
}

/* ======================================================================
 * Ring Buffer — polled from DTCNT
 * ====================================================================== */

/**
  * @brief  Return number of samples available to read.
  *         Polls DMA DTCNT register directly — no ISR dependency.
  *         Detects overflow (DMA lapped the reader).
  */
uint32_t la_hw_ring_available(void)
{
    uint32_t wr_laps, wr_pos;
    uint32_t wr_total, rd_total, avail;

    la_hw_get_write_total(&wr_laps, &wr_pos);

    wr_total = wr_laps * LA_HW_RING_SIZE + wr_pos;
    rd_total = g_rd_laps * LA_HW_RING_SIZE + g_rd_pos;
    avail    = wr_total - rd_total;

    if (avail > LA_HW_RING_SIZE) {
        /* Overflow: DMA has lapped the reader.
         * Snap reader to current write position (discard stale data). */
        g_dma_overflow_count++;
        g_rd_laps = wr_laps;
        g_rd_pos  = wr_pos;
        return 0;
    }

    return avail;
}

/**
  * @brief  Copy up to 'count' samples from ring buffer into buf.
  *         Handles wrap-around transparently.
  *         Returns actual number copied.
  */
uint32_t la_hw_ring_read(uint8_t *buf, uint32_t count)
{
    uint32_t avail = la_hw_ring_available();
    uint32_t first, second;

    if (count > avail) {
        count = avail;
    }
    if (count == 0) {
        return 0;
    }

    /* How many bytes from g_rd_pos to end of ring? */
    first = LA_HW_RING_SIZE - g_rd_pos;
    if (first > count) {
        first = count;
    }

    /* Copy first segment (rd_pos .. end or rd_pos .. rd_pos+count) */
    memcpy(buf, &g_ring[g_rd_pos], first);

    /* Copy wrapped segment if needed */
    second = count - first;
    if (second > 0) {
        memcpy(buf + first, &g_ring[0], second);
    }

    /* Advance reader */
    g_rd_pos += count;
    if (g_rd_pos >= LA_HW_RING_SIZE) {
        g_rd_pos -= LA_HW_RING_SIZE;
        g_rd_laps++;
    }

    return count;
}

/* ======================================================================
 * Software GPIO Read (fallback / testing)
 * ====================================================================== */

uint32_t la_hw_read_gpio(uint8_t *buf, uint32_t count, uint8_t mask)
{
    uint32_t i;
    for (i = 0; i < count; i++) {
        buf[i] = (uint8_t)(LA_GPIO_PORT->idt & 0xFF) & mask;
    }
    return count;
}

/* ======================================================================
 * DMA + Timer Paced Capture
 *
 * Flow:  TMR2 overflow -> DMAMUX -> DMA1_CH1 reads GPIOC->IDT (byte)
 *        -> writes to g_ring[] in circular mode.
 *        Main loop polls DTCNT for write position (sample granularity).
 *        FDT ISR counts laps for overflow detection only.
 * ====================================================================== */

void la_hw_start_capture(LaDevice *dev)
{
    uint32_t divider = dev->config.rate_divider;
    uint32_t period;
    dma_init_type dma_init_struct;
    gpio_type *src_port = LA_GPIO_PORT;   /* default GPIOC */

    /* Latch sample width: 1 byte = 8-channel, 2 bytes = 16-channel.
     * Anything other than 16 falls back to 8 for forward-compat. */
    g_sample_bytes = (dev->config.sample_width == 16) ? 2 : 1;
    g_dma_xfers    = LA_HW_RING_SIZE / g_sample_bytes;

    /* Pick the source port. GPIOB needs its clock + pin direction set up
     * once. PB0 is the board's own heartbeat LED output and must NOT be
     * forced to input — DMA reads IDT regardless of direction so the LED
     * still appears in the sample stream as channel 0. */
    if (dev->config.port == LA_PORT_GPIOB) {
        src_port = GPIOB;
        crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
        gpio_init_type gi;
        gpio_default_para_init(&gi);
        gi.gpio_pins = (uint16_t)(GPIO_PINS_ALL & ~GPIO_PINS_0);
        gi.gpio_mode = GPIO_MODE_INPUT;
        gi.gpio_pull = GPIO_PULL_NONE;
        gpio_init(GPIOB, &gi);
    }

    /* Reset state */
    g_rd_pos  = 0;
    g_rd_laps = 0;
    g_wr_laps = 0;
    g_data_ready = 0;
    g_dma_overflow_count = 0;
    g_dma_fdt_count = 0;
    g_dma_hdt_count = 0;

    /* ── Timer setup ── */
    if (divider == 0) {
        divider = 1;
    }
    /* Host sends rate_divider = 24 MHz_base / desired_rate.
     * Our timer runs at 216 MHz, so real divider = divider * 9. */
    divider = divider * (LA_TIMER_INPUT_FREQ / 24000000u);
    period = divider - 1;

    tmr_counter_enable(LA_TIMER, FALSE);
    tmr_counter_value_set(LA_TIMER, 0);
    tmr_base_init(LA_TIMER, period, 0);
    tmr_cnt_dir_set(LA_TIMER, TMR_COUNT_UP);
    tmr_clock_source_div_set(LA_TIMER, TMR_CLOCK_DIV1);
    tmr_dma_request_enable(LA_TIMER, TMR_OVERFLOW_DMA_REQUEST, TRUE);

    /* ── DMA setup ── */
    dma_channel_enable(LA_DMA_CHANNEL, FALSE);
    dma_reset(LA_DMA_CHANNEL);

    dma_default_para_init(&dma_init_struct);
    dma_init_struct.peripheral_base_addr  = (uint32_t)&(src_port->idt);
    dma_init_struct.memory_base_addr      = (uint32_t)g_ring;
    dma_init_struct.direction             = DMA_DIR_PERIPHERAL_TO_MEMORY;
    dma_init_struct.buffer_size           = g_dma_xfers;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.memory_inc_enable     = TRUE;
    dma_init_struct.peripheral_data_width = (g_sample_bytes == 2)
        ? DMA_PERIPHERAL_DATA_WIDTH_HALFWORD : DMA_PERIPHERAL_DATA_WIDTH_BYTE;
    dma_init_struct.memory_data_width     = (g_sample_bytes == 2)
        ? DMA_MEMORY_DATA_WIDTH_HALFWORD : DMA_MEMORY_DATA_WIDTH_BYTE;
    dma_init_struct.loop_mode_enable      = TRUE;   /* circular */
    dma_init_struct.priority              = DMA_PRIORITY_HIGH;
    dma_init(LA_DMA_CHANNEL, &dma_init_struct);

    /* Route TMR2 overflow to DMA1_Channel1 via DMAMUX */
    dmamux_enable(LA_DMA, TRUE);
    dmamux_init(LA_DMA_MUX_CHANNEL, LA_DMA_MUX_REQ);

    /* Enable HDT + FDT interrupts.
     * HDT fires at 16 KB (half ring) — sets data-ready flag.
     * FDT fires at 32 KB (full ring) — sets data-ready flag + counts laps. */
    dma_interrupt_enable(LA_DMA_CHANNEL, DMA_HDT_INT | DMA_FDT_INT, TRUE);
    nvic_irq_enable(LA_DMA_IRQn, 1, 0);

    /* Enable DMA channel */
    dma_channel_enable(LA_DMA_CHANNEL, TRUE);

    /* Start timer */
    tmr_counter_enable(LA_TIMER, TRUE);
}

void la_hw_stop_capture(LaDevice *dev)
{
    (void)dev;

    tmr_counter_enable(LA_TIMER, FALSE);
    tmr_dma_request_enable(LA_TIMER, TMR_OVERFLOW_DMA_REQUEST, FALSE);

    dma_channel_enable(LA_DMA_CHANNEL, FALSE);
    dma_interrupt_enable(LA_DMA_CHANNEL, DMA_HDT_INT | DMA_FDT_INT, FALSE);
    nvic_irq_disable(LA_DMA_IRQn);
}

uint32_t la_hw_available_samples(LaDevice *dev)
{
    (void)dev;
    return la_hw_ring_available();
}

uint32_t la_hw_read_samples(LaDevice *dev, uint8_t *buf, uint32_t count)
{
    (void)dev;
    return la_hw_ring_read(buf, count);
}

/* ======================================================================
 * DMA1 Channel 1 ISR — HDT + FDT
 *
 * HDT fires at the half-ring mark (16 KB), FDT at the full mark (32 KB).
 * At 1 MHz: every 16.4 ms.  At 24 MHz: every 682 µs.
 * Purpose:
 *   - Set g_data_ready flag so main loop knows a chunk is available.
 *   - FDT also increments lap counter for overflow detection.
 * ====================================================================== */
void DMA1_Channel1_IRQHandler(void)
{
    if (dma_flag_get(LA_DMA_HDT_FLAG)) {
        dma_flag_clear(LA_DMA_HDT_FLAG);
        g_data_ready = 1;
        g_dma_hdt_count++;
    }
    if (dma_flag_get(LA_DMA_FDT_FLAG)) {
        dma_flag_clear(LA_DMA_FDT_FLAG);
        g_data_ready = 1;
        g_wr_laps++;
        g_dma_fdt_count++;
    }
}

/* Return DMA data number counter (counts DOWN from buffer_size) */
uint16_t la_hw_dma_remaining(void)
{
    return (uint16_t)dma_data_number_get(LA_DMA_CHANNEL);
}
