/**
  **************************************************************************
  * @file     composite_class.h
  * @brief    USB composite WinUSB + CDC class header
  **************************************************************************
  */

#ifndef __COMPOSITE_CLASS_H
#define __COMPOSITE_CLASS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usb_std.h"
#include "usbd_core.h"

/* ══════════════════════════════════════════════════════════════════════
 * Interface & Endpoint Assignment
 *
 *  Interface 0    : WinUSB LA  (vendor-specific)  EP1 IN + EP1 OUT  bulk
 *  Interface 1    : CMSIS-DAP (HID when DAP_FW_V1, else WinUSB bulk on EP2)
 *  Interface 2/3  : CDC       EP3 interrupt + EP4 bulk
 * ══════════════════════════════════════════════════════════════════════ */

/* ── WinUSB LA endpoints (Interface 0) ── */
#define COMP_WINUSB_INTERFACE           0x00
#define COMP_WINUSB_BULK_IN_EPT         0x81
#define COMP_WINUSB_BULK_OUT_EPT        0x01
#define COMP_WINUSB_IN_MAXPACKET        0x0040   /* FS */
#define COMP_WINUSB_OUT_MAXPACKET       0x0040
#define COMP_WINUSB_HS_IN_MAXPACKET     0x0200   /* HS 512 */
#define COMP_WINUSB_HS_OUT_MAXPACKET    0x0200

/* ── CMSIS-DAP v2 WinUSB endpoints (Interface 1) ── */
#define COMP_DAP_INTERFACE              0x01
#define COMP_DAP_BULK_IN_EPT            0x82
#define COMP_DAP_BULK_OUT_EPT           0x02
#define COMP_DAP_IN_MAXPACKET           0x0040   /* FS */
#define COMP_DAP_OUT_MAXPACKET          0x0040
#define COMP_DAP_HS_IN_MAXPACKET        0x0200   /* HS 512 */
#define COMP_DAP_HS_OUT_MAXPACKET       0x0200

/* ── CDC endpoints (disabled) ── */
#ifdef CDC_DEBUG_ENABLED
#define COMP_CDC_COM_INTERFACE          0x02
#define COMP_CDC_DATA_INTERFACE         0x03
#define COMP_CDC_INT_EPT                0x83     /* interrupt notify IN */
#define COMP_CDC_BULK_IN_EPT            0x84     /* data IN  */
#define COMP_CDC_BULK_OUT_EPT           0x04     /* data OUT */
#define COMP_CDC_CMD_MAXPACKET          0x08
#define COMP_CDC_IN_MAXPACKET           0x0040   /* FS */
#define COMP_CDC_OUT_MAXPACKET          0x0040
#define COMP_CDC_HS_IN_MAXPACKET        0x0200   /* HS 512 */
#define COMP_CDC_HS_OUT_MAXPACKET       0x0200
#endif /* CDC_DEBUG_ENABLED */

/* ── CDC line coding ── */
#ifdef CDC_DEBUG_ENABLED
typedef struct {
  uint32_t bitrate;
  uint8_t  format;
  uint8_t  parity;
  uint8_t  data;
} cdc_linecoding_type;
#endif

/* ── Composite device state ── */
typedef struct {
  /* WinUSB — Logic Analyzer */
  uint32_t             winusb_alt;
  uint8_t             *winusb_rx_buf;
  uint16_t             winusb_rxlen;
  volatile uint8_t     winusb_tx_done;
  volatile uint8_t     winusb_rx_done;
  uint32_t             winusb_maxpkt;

  /* CMSIS-DAP v2 — debug probe */
  uint32_t             dap_alt;
  uint8_t             *dap_rx_buf;
  uint16_t             dap_rxlen;
  volatile uint8_t     dap_tx_done;
  volatile uint8_t     dap_rx_done;
  uint32_t             dap_maxpkt;

  /* CDC */
#ifdef CDC_DEBUG_ENABLED
  uint32_t             cdc_alt;
  uint8_t             *cdc_rx_buf;
  uint8_t             *cdc_cmd_buf;
  uint8_t              cdc_req;
  uint16_t             cdc_len;
  uint16_t             cdc_rxlen;
  volatile uint8_t     cdc_tx_done;
  volatile uint8_t     cdc_rx_done;
  cdc_linecoding_type  linecoding;
  uint32_t             cdc_maxpkt;
#endif
} composite_struct_type;

/* ── API ── */

extern usbd_class_handler composite_class_handler;

/* WinUSB bulk I/O — same API as the standalone driver */
error_status usb_winusb_send_data(void *udev, uint8_t *buf, uint16_t len);
uint16_t     usb_winusb_get_rxdata(void *udev, uint8_t *recv_data);

/* DAP bulk I/O */
error_status usb_dap_send_data(void *udev, uint8_t *buf, uint16_t len);
uint16_t     usb_dap_get_rxdata(void *udev, uint8_t *recv_data);

/* CDC bulk I/O */
#ifdef CDC_DEBUG_ENABLED
error_status usb_cdc_send_data(void *udev, uint8_t *buf, uint16_t len);
uint16_t     usb_cdc_get_rxdata(void *udev, uint8_t *recv_data);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __COMPOSITE_CLASS_H */
