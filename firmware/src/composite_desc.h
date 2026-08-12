/**
  **************************************************************************
  * @file     composite_desc.h
  * @brief    USB composite WinUSB + CDC descriptor header
  **************************************************************************
  */

#ifndef __COMPOSITE_DESC_H
#define __COMPOSITE_DESC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "composite_class.h"
#include "usbd_core.h"
#include "fw_version.h"

/* ── CDC virtual COM port ──
   Define CDC_DEBUG_ENABLED to include CDC ACM interfaces in the
   composite descriptor.  Comment out for WinUSB-only mode. */
/* #define CDC_DEBUG_ENABLED */  /* now set via project defines (-D flag) */

/* ── VID / PID ── */
#define USBD_COMP_VENDOR_ID             0x1209
#define USBD_COMP_PRODUCT_ID            0x0002

/* ── Descriptor sizes ──
   Three build tiers. LA_ONLY_BUILD is the open-source logic-analyser cut: a
   single vendor interface, no CMSIS-DAP and no CDC, so the device is NOT a
   composite at all (bDeviceClass 0xFF) and WinUSB binds interface 0 directly.
   Fewer moving parts than the full product build, and nothing on the wire but
   the LA protocol. */
#ifdef LA_ONLY_BUILD
#define USBD_COMP_CONFIG_DESC_SIZE      32    /* LA (23) + config (9) */
#define USBD_COMP_NUM_INTERFACES        1
#elif defined(CDC_DEBUG_ENABLED)
#define USBD_COMP_CONFIG_DESC_SIZE      138   /* LA (23) + DAP HID (32) + IADs (16) + CDC (67) + config (9) */
#define USBD_COMP_NUM_INTERFACES        4
#else
#define USBD_COMP_CONFIG_DESC_SIZE      64    /* LA (23) + DAP HID (32) + config (9) */
#define USBD_COMP_NUM_INTERFACES        2
#endif
#define USBD_COMP_SIZ_STRING_LANGID     4
#define USBD_COMP_SIZ_STRING_SERIAL     0x1A  /* 2 + 12 chars*2 (48-bit UID hash hex) */

/* ── Strings ── */
#define USBD_COMP_DESC_MANUFACTURER_STRING    "Zolex"
#define USBD_COMP_DESC_PRODUCT_STRING         "Zolex Superboard v" FW_VERSION_STR
#define USBD_COMP_DESC_CONFIGURATION_STRING   "LA + DAP"
#define USBD_COMP_DESC_INTERFACE_STRING       "CMSIS-DAP"

/* ── MCU unique ID addresses ── */
#define MCU_ID1                         (0x1FFFF7E8)
#define MCU_ID2                         (0x1FFFF7EC)
#define MCU_ID3                         (0x1FFFF7F0)

/* ── MS OS 1.0 vendor code ── */
#define WINUSB_BMS_VENDOR_CODE          0xA0

/* CMSIS-DAP HID report descriptor (29 bytes, vendor usage page 0xFF00) */
#define COMP_DAP_HID_REPORT_SIZE  29
extern uint8_t g_dap_hid_report[COMP_DAP_HID_REPORT_SIZE];

extern usbd_desc_handler composite_desc_handler;

#ifdef __cplusplus
}
#endif

#endif /* __COMPOSITE_DESC_H */
