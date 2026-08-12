/**
  **************************************************************************
  * @file     composite_desc.c
  * @brief    USB composite WinUSB + CDC ACM descriptors
  **************************************************************************
  */

#include "usb_std.h"
#include "usbd_sdr.h"
#include "usbd_core.h"
#include "composite_desc.h"
#include "fw_version.h"

/* ──────────────────────────────────────────────────────────────────────
 * Forward declarations
 * ────────────────────────────────────────────────────────────────────── */

static usbd_desc_t *get_device_descriptor(void);
static usbd_desc_t *get_device_qualifier(void);
static usbd_desc_t *get_device_configuration(void);
static usbd_desc_t *get_device_other_speed(void);
static usbd_desc_t *get_device_lang_id(void);
static usbd_desc_t *get_device_manufacturer_string(void);
static usbd_desc_t *get_device_product_string(void);
static usbd_desc_t *get_device_serial_string(void);
static usbd_desc_t *get_device_interface_string(void);
static usbd_desc_t *get_device_config_string(void);
static usbd_desc_t *get_hs_device_configuration(void);

#if USBD_SUPPORT_WINUSB == 1
static usbd_desc_t *get_device_winusb_os_string(void);
static usbd_desc_t *get_device_winusb_os_feature(void);
static usbd_desc_t *get_device_winusb_os_property(void);
#endif

static uint16_t usbd_unicode_convert(uint8_t *string, uint8_t *unicode_buf);
static void usbd_int_to_unicode(uint32_t value, uint8_t *pbuf, uint8_t len);
static void get_serial_num(void);

#if defined ( __ICCARM__ )
  #pragma data_alignment=4
#endif
ALIGNED_HEAD static uint8_t g_usbd_desc_buffer[256] ALIGNED_TAIL;

/* ──────────────────────────────────────────────────────────────────────
 * Descriptor handler — passed to usbd_init()
 * ────────────────────────────────────────────────────────────────────── */

usbd_desc_handler composite_desc_handler =
{
  get_device_descriptor,
  get_device_qualifier,
  get_device_configuration,
  get_device_other_speed,
  get_device_lang_id,
  get_device_manufacturer_string,
  get_device_product_string,
  get_device_serial_string,
  get_device_interface_string,
  get_device_config_string,
  get_hs_device_configuration,
#if USBD_SUPPORT_WINUSB == 1
  get_device_winusb_os_string,
  get_device_winusb_os_feature,
  get_device_winusb_os_property,
#endif
};

/* ──────────────────────────────────────────────────────────────────────
 * CMSIS-DAP HID report descriptor  (25 bytes, vendor usage page 0xFF00)
 * ────────────────────────────────────────────────────────────────────── */
ALIGNED_HEAD uint8_t g_dap_hid_report[COMP_DAP_HID_REPORT_SIZE] ALIGNED_TAIL = {
    0x06, 0x00, 0xFF,   /* Usage Page = 0xFF00 (Vendor Defined) */
    0x09, 0x01,          /* Usage (Vendor Usage 1) */
    0xA1, 0x01,          /* Collection (Application) */
    0x19, 0x01,          /* Usage Minimum */
    0x29, 0x40,          /* Usage Maximum (64) */
    0x15, 0x00,          /* Logical Minimum (0) */
    0x26, 0xFF, 0x00,    /* Logical Maximum (255) */
    0x75, 0x08,          /* Report Size: 8 bits */
    0x95, 0x40,          /* Report Count: 64 */
    0x81, 0x00,          /* Input  (Data, Array, Abs) */
    0x19, 0x01,          /* Usage Minimum */
    0x29, 0x40,          /* Usage Maximum */
    0x91, 0x00,          /* Output (Data, Array, Abs) */
    0xC0                 /* End Collection */
};

/* ══════════════════════════════════════════════════════════════════════
 * Device descriptor — class 0xEF / 0x02 / 0x01  (IAD composite)
 * ══════════════════════════════════════════════════════════════════════ */

#if defined ( __ICCARM__ )
  #pragma data_alignment=4
#endif
ALIGNED_HEAD static uint8_t g_usbd_descriptor[USB_DEVICE_DESC_LEN] ALIGNED_TAIL =
{
  USB_DEVICE_DESC_LEN,                   /* bLength */
  USB_DESCIPTOR_TYPE_DEVICE,             /* bDescriptorType */
  0x00,                                  /* bcdUSB lo */
  0x02,                                  /* bcdUSB hi  — USB 2.0 */
#if defined(LA_ONLY_BUILD)
  0xFF,                                  /* bDeviceClass: vendor — single
                                            interface, not a composite, so
                                            WinUSB binds the device directly */
  0x00,                                  /* bDeviceSubClass */
  0x00,                                  /* bDeviceProtocol */
#elif defined(CDC_DEBUG_ENABLED)
  0xEF,                                  /* bDeviceClass: Misc (IAD) */
  0x02,                                  /* bDeviceSubClass: Common */
  0x01,                                  /* bDeviceProtocol: IAD */
#else
  0x00,                                  /* bDeviceClass: composite */
  0x00,                                  /* bDeviceSubClass */
  0x00,                                  /* bDeviceProtocol */
#endif
  USB_MAX_EP0_SIZE,                      /* bMaxPacketSize */
  LBYTE(USBD_COMP_VENDOR_ID),
  HBYTE(USBD_COMP_VENDOR_ID),
  LBYTE(USBD_COMP_PRODUCT_ID),
  HBYTE(USBD_COMP_PRODUCT_ID),
  FW_BCD_DEVICE_LO,                      /* bcdDevice lo — from fw_version.h */
  FW_BCD_DEVICE_HI,                      /* bcdDevice hi — e.g. 1.0.3 -> 0x0103 */
  USB_MFC_STRING,                        /* iManufacturer */
  USB_PRODUCT_STRING,                    /* iProduct */
  USB_SERIAL_STRING,                     /* iSerialNumber */
  1                                      /* bNumConfigurations */
};

/* ══════════════════════════════════════════════════════════════════════
 * Configuration descriptor (Full-Speed) — 106 bytes
 *
 *   IAD 0  ─ WinUSB   interface 0       EP1 IN + EP1 OUT  bulk 64
 *   IAD 1  ─ CDC ACM  interfaces 1+2    EP3 IN int 8,  EP2 IN/OUT bulk 64
 * ══════════════════════════════════════════════════════════════════════ */

#if defined ( __ICCARM__ )
  #pragma data_alignment=4
#endif
ALIGNED_HEAD static uint8_t g_usbd_configuration[USBD_COMP_CONFIG_DESC_SIZE] ALIGNED_TAIL =
{
  /* ── Configuration descriptor ── */
  USB_DEVICE_CFG_DESC_LEN,               /* 0  bLength */
  USB_DESCIPTOR_TYPE_CONFIGURATION,      /* 1  bDescriptorType */
  LBYTE(USBD_COMP_CONFIG_DESC_SIZE),     /* 2  wTotalLength lo */
  HBYTE(USBD_COMP_CONFIG_DESC_SIZE),     /* 3  wTotalLength hi */
  USBD_COMP_NUM_INTERFACES,              /* 4  bNumInterfaces */
  0x01,                                  /* 5  bConfigurationValue */
  0x00,                                  /* 6  iConfiguration */
  0xC0,                                  /* 7  bmAttributes: self-powered */
  0x32,                                  /* 8  MaxPower 100 mA */

#ifdef CDC_DEBUG_ENABLED
  /* ════════════════════════════════════════════════════════════════════
   * IAD — WinUSB function  (interface 0)  — only needed for composite
   * ════════════════════════════════════════════════════════════════════ */
  0x08,                                  /* bLength */
  0x0B,                                  /* bDescriptorType: IAD */
  COMP_WINUSB_INTERFACE,                 /* bFirstInterface: 0 */
  0x01,                                  /* bInterfaceCount */
  0xFF,                                  /* bFunctionClass: vendor */
  0x00,                                  /* bFunctionSubClass */
  0x00,                                  /* bFunctionProtocol */
  0x00,                                  /* iFunction */
#endif

  /* ─── Interface 0 — WinUSB vendor-specific ─── */
  USB_DEVICE_IF_DESC_LEN,                /* bLength */
  USB_DESCIPTOR_TYPE_INTERFACE,          /* bDescriptorType */
  COMP_WINUSB_INTERFACE,                 /* bInterfaceNumber: 0 */
  0x00,                                  /* bAlternateSetting */
  0x02,                                  /* bNumEndpoints */
  0xFF,                                  /* bInterfaceClass: vendor */
  0x00,                                  /* bInterfaceSubClass */
  0x00,                                  /* bInterfaceProtocol */
  0x00,                                  /* iInterface */

  /* EP1 OUT — WinUSB bulk */
  USB_DEVICE_EPT_LEN,
  USB_DESCIPTOR_TYPE_ENDPOINT,
  COMP_WINUSB_BULK_OUT_EPT,              /* 0x01 */
  USB_EPT_DESC_BULK,
  LBYTE(COMP_WINUSB_OUT_MAXPACKET),
  HBYTE(COMP_WINUSB_OUT_MAXPACKET),
  0x00,

  /* EP1 IN — WinUSB bulk */
  USB_DEVICE_EPT_LEN,
  USB_DESCIPTOR_TYPE_ENDPOINT,
  COMP_WINUSB_BULK_IN_EPT,              /* 0x81 */
  USB_EPT_DESC_BULK,
  LBYTE(COMP_WINUSB_IN_MAXPACKET),
  HBYTE(COMP_WINUSB_IN_MAXPACKET),
  0x00,

#ifndef LA_ONLY_BUILD
  /* ─── Interface 1 — CMSIS-DAP v1 HID ─── */
  USB_DEVICE_IF_DESC_LEN,                /* bLength */
  USB_DESCIPTOR_TYPE_INTERFACE,          /* bDescriptorType */
  COMP_DAP_INTERFACE,                    /* bInterfaceNumber: 1 */
  0x00,                                  /* bAlternateSetting */
  0x02,                                  /* bNumEndpoints */
  0x03,                                  /* bInterfaceClass: HID */
  0x00,                                  /* bInterfaceSubClass */
  0x00,                                  /* bInterfaceProtocol */
  USB_INTERFACE_STRING,                  /* iInterface: "CMSIS-DAP" */

  /* HID class descriptor */
  0x09,                                  /* bLength */
  0x21,                                  /* bDescriptorType: HID */
  0x10, 0x01,                            /* bcdHID: 1.10 */
  0x00,                                  /* bCountryCode */
  0x01,                                  /* bNumDescriptors */
  0x22,                                  /* bDescriptorType: Report */
  COMP_DAP_HID_REPORT_SIZE & 0xFF,       /* wDescriptorLength lo */
  COMP_DAP_HID_REPORT_SIZE >> 8,         /* wDescriptorLength hi */

  /* EP2 OUT — DAP interrupt */
  USB_DEVICE_EPT_LEN,
  USB_DESCIPTOR_TYPE_ENDPOINT,
  COMP_DAP_BULK_OUT_EPT,                 /* 0x02 */
  USB_EPT_DESC_INTERRUPT,
  LBYTE(COMP_DAP_OUT_MAXPACKET),
  HBYTE(COMP_DAP_OUT_MAXPACKET),
  0x01,                                  /* bInterval: 1 ms (FS) */

  /* EP2 IN — DAP interrupt */
  USB_DEVICE_EPT_LEN,
  USB_DESCIPTOR_TYPE_ENDPOINT,
  COMP_DAP_BULK_IN_EPT,                  /* 0x82 */
  USB_EPT_DESC_INTERRUPT,
  LBYTE(COMP_DAP_IN_MAXPACKET),
  HBYTE(COMP_DAP_IN_MAXPACKET),
  0x01,                                  /* bInterval: 1 ms (FS) */
#endif /* !LA_ONLY_BUILD */

#ifdef CDC_DEBUG_ENABLED
  /* ════════════════════════════════════════════════════════════════════
   * IAD — CDC ACM function  (interfaces 2+3)
   * ════════════════════════════════════════════════════════════════════ */
  0x08,                                  /* bLength */
  0x0B,                                  /* bDescriptorType: IAD */
  COMP_CDC_COM_INTERFACE,                /* bFirstInterface: 1 */
  0x02,                                  /* bInterfaceCount */
  0x02,                                  /* bFunctionClass: CDC */
  0x02,                                  /* bFunctionSubClass: ACM */
  0x01,                                  /* bFunctionProtocol: AT Cmd */
  0x00,                                  /* iFunction */

  /* ─── Interface 1 — CDC Communication ─── */
  USB_DEVICE_IF_DESC_LEN,
  USB_DESCIPTOR_TYPE_INTERFACE,
  COMP_CDC_COM_INTERFACE,                /* bInterfaceNumber: 1 */
  0x00,                                  /* bAlternateSetting */
  0x01,                                  /* bNumEndpoints */
  USB_CLASS_CODE_CDC,                    /* bInterfaceClass: 0x02 */
  0x02,                                  /* bInterfaceSubClass: ACM */
  0x01,                                  /* bInterfaceProtocol: AT Cmd */
  0x00,                                  /* iInterface */

  /* CDC Header Functional Descriptor */
  0x05,                                  /* bLength */
  USBD_CDC_CS_INTERFACE,                 /* bDescriptorType: 0x24 */
  USBD_CDC_SUBTYPE_HEADER,              /* bDescriptorSubtype: 0x00 */
  0x10,                                  /* bcdCDC lo — 1.10 */
  0x01,                                  /* bcdCDC hi */

  /* CDC Call Management Functional Descriptor */
  0x05,
  USBD_CDC_CS_INTERFACE,
  USBD_CDC_SUBTYPE_CMF,                 /* 0x01 */
  0x00,                                  /* bmCapabilities */
  COMP_CDC_DATA_INTERFACE,              /* bDataInterface: 2 */

  /* CDC Abstract Control Management FD */
  0x04,
  USBD_CDC_CS_INTERFACE,
  USBD_CDC_SUBTYPE_ACM,                 /* 0x02 */
  0x02,                                  /* bmCapabilities: line coding + serial state */

  /* CDC Union Functional Descriptor */
  0x05,
  USBD_CDC_CS_INTERFACE,
  USBD_CDC_SUBTYPE_UFD,                 /* 0x06 */
  COMP_CDC_COM_INTERFACE,               /* bControlInterface: 1 */
  COMP_CDC_DATA_INTERFACE,              /* bSubordinateInterface0: 2 */

  /* EP3 IN — CDC interrupt */
  USB_DEVICE_EPT_LEN,
  USB_DESCIPTOR_TYPE_ENDPOINT,
  COMP_CDC_INT_EPT,                     /* 0x83 */
  USB_EPT_DESC_INTERRUPT,
  LBYTE(COMP_CDC_CMD_MAXPACKET),
  HBYTE(COMP_CDC_CMD_MAXPACKET),
  0xFF,                                  /* bInterval: 255 ms (FS) */

  /* ─── Interface 2 — CDC Data ─── */
  USB_DEVICE_IF_DESC_LEN,
  USB_DESCIPTOR_TYPE_INTERFACE,
  COMP_CDC_DATA_INTERFACE,              /* bInterfaceNumber: 2 */
  0x00,                                  /* bAlternateSetting */
  0x02,                                  /* bNumEndpoints */
  USB_CLASS_CODE_CDCDATA,               /* bInterfaceClass: 0x0A */
  0x00,                                  /* bInterfaceSubClass */
  0x00,                                  /* bInterfaceProtocol */
  0x00,                                  /* iInterface */

  /* EP2 IN — CDC bulk */
  USB_DEVICE_EPT_LEN,
  USB_DESCIPTOR_TYPE_ENDPOINT,
  COMP_CDC_BULK_IN_EPT,                 /* 0x82 */
  USB_EPT_DESC_BULK,
  LBYTE(COMP_CDC_IN_MAXPACKET),
  HBYTE(COMP_CDC_IN_MAXPACKET),
  0x00,

  /* EP2 OUT — CDC bulk */
  USB_DEVICE_EPT_LEN,
  USB_DESCIPTOR_TYPE_ENDPOINT,
  COMP_CDC_BULK_OUT_EPT,                /* 0x02 */
  USB_EPT_DESC_BULK,
  LBYTE(COMP_CDC_OUT_MAXPACKET),
  HBYTE(COMP_CDC_OUT_MAXPACKET),
  0x00,
#endif /* CDC_DEBUG_ENABLED */
};

/* ══════════════════════════════════════════════════════════════════════
 * Configuration descriptor (High-Speed) — same layout, 512-byte bulk
 * ══════════════════════════════════════════════════════════════════════ */

#if defined ( __ICCARM__ )
  #pragma data_alignment=4
#endif
ALIGNED_HEAD static uint8_t g_usbd_hs_configuration[USBD_COMP_CONFIG_DESC_SIZE] ALIGNED_TAIL =
{
  USB_DEVICE_CFG_DESC_LEN,
  USB_DESCIPTOR_TYPE_CONFIGURATION,
  LBYTE(USBD_COMP_CONFIG_DESC_SIZE),
  HBYTE(USBD_COMP_CONFIG_DESC_SIZE),
  USBD_COMP_NUM_INTERFACES, 0x01, 0x00, 0xC0, 0x32,

#ifdef CDC_DEBUG_ENABLED
  /* IAD WinUSB */
  0x08, 0x0B, COMP_WINUSB_INTERFACE, 0x01, 0xFF, 0x00, 0x00, 0x00,
#endif

  /* Interface 0 — WinUSB */
  USB_DEVICE_IF_DESC_LEN, USB_DESCIPTOR_TYPE_INTERFACE,
  COMP_WINUSB_INTERFACE, 0x00, 0x02, 0xFF, 0x00, 0x00, 0x00,

  /* EP1 OUT bulk 512 */
  USB_DEVICE_EPT_LEN, USB_DESCIPTOR_TYPE_ENDPOINT,
  COMP_WINUSB_BULK_OUT_EPT, USB_EPT_DESC_BULK,
  LBYTE(COMP_WINUSB_HS_OUT_MAXPACKET), HBYTE(COMP_WINUSB_HS_OUT_MAXPACKET), 0x00,

  /* EP1 IN bulk 512 */
  USB_DEVICE_EPT_LEN, USB_DESCIPTOR_TYPE_ENDPOINT,
  COMP_WINUSB_BULK_IN_EPT, USB_EPT_DESC_BULK,
  LBYTE(COMP_WINUSB_HS_IN_MAXPACKET), HBYTE(COMP_WINUSB_HS_IN_MAXPACKET), 0x00,

#ifndef LA_ONLY_BUILD
  /* Interface 1 — CMSIS-DAP v1 HID */
  USB_DEVICE_IF_DESC_LEN, USB_DESCIPTOR_TYPE_INTERFACE,
  COMP_DAP_INTERFACE, 0x00, 0x02, 0x03, 0x00, 0x00, USB_INTERFACE_STRING,

  /* HID class descriptor */
  0x09, 0x21, 0x10, 0x01, 0x00, 0x01, 0x22,
  COMP_DAP_HID_REPORT_SIZE & 0xFF, COMP_DAP_HID_REPORT_SIZE >> 8,

  /* EP2 OUT interrupt 64 */
  USB_DEVICE_EPT_LEN, USB_DESCIPTOR_TYPE_ENDPOINT,
  COMP_DAP_BULK_OUT_EPT, USB_EPT_DESC_INTERRUPT,
  LBYTE(COMP_DAP_OUT_MAXPACKET), HBYTE(COMP_DAP_OUT_MAXPACKET), 0x04,

  /* EP2 IN interrupt 64 */
  USB_DEVICE_EPT_LEN, USB_DESCIPTOR_TYPE_ENDPOINT,
  COMP_DAP_BULK_IN_EPT, USB_EPT_DESC_INTERRUPT,
  LBYTE(COMP_DAP_IN_MAXPACKET), HBYTE(COMP_DAP_IN_MAXPACKET), 0x04,
#endif /* !LA_ONLY_BUILD */

#ifdef CDC_DEBUG_ENABLED
  /* IAD CDC */
  0x08, 0x0B, COMP_CDC_COM_INTERFACE, 0x02, 0x02, 0x02, 0x01, 0x00,

  /* Interface 1 — CDC Comm */
  USB_DEVICE_IF_DESC_LEN, USB_DESCIPTOR_TYPE_INTERFACE,
  COMP_CDC_COM_INTERFACE, 0x00, 0x01, USB_CLASS_CODE_CDC, 0x02, 0x01, 0x00,

  /* CDC functional descriptors (unchanged) */
  0x05, USBD_CDC_CS_INTERFACE, USBD_CDC_SUBTYPE_HEADER, 0x10, 0x01,
  0x05, USBD_CDC_CS_INTERFACE, USBD_CDC_SUBTYPE_CMF, 0x00, COMP_CDC_DATA_INTERFACE,
  0x04, USBD_CDC_CS_INTERFACE, USBD_CDC_SUBTYPE_ACM, 0x02,
  0x05, USBD_CDC_CS_INTERFACE, USBD_CDC_SUBTYPE_UFD, COMP_CDC_COM_INTERFACE, COMP_CDC_DATA_INTERFACE,

  /* EP3 IN interrupt 8 */
  USB_DEVICE_EPT_LEN, USB_DESCIPTOR_TYPE_ENDPOINT,
  COMP_CDC_INT_EPT, USB_EPT_DESC_INTERRUPT,
  LBYTE(COMP_CDC_CMD_MAXPACKET), HBYTE(COMP_CDC_CMD_MAXPACKET), 0x10, /* bInterval: 16 (HS) */

  /* Interface 2 — CDC Data */
  USB_DEVICE_IF_DESC_LEN, USB_DESCIPTOR_TYPE_INTERFACE,
  COMP_CDC_DATA_INTERFACE, 0x00, 0x02, USB_CLASS_CODE_CDCDATA, 0x00, 0x00, 0x00,

  /* EP2 IN bulk 512 */
  USB_DEVICE_EPT_LEN, USB_DESCIPTOR_TYPE_ENDPOINT,
  COMP_CDC_BULK_IN_EPT, USB_EPT_DESC_BULK,
  LBYTE(COMP_CDC_HS_IN_MAXPACKET), HBYTE(COMP_CDC_HS_IN_MAXPACKET), 0x00,

  /* EP2 OUT bulk 512 */
  USB_DEVICE_EPT_LEN, USB_DESCIPTOR_TYPE_ENDPOINT,
  COMP_CDC_BULK_OUT_EPT, USB_EPT_DESC_BULK,
  LBYTE(COMP_CDC_HS_OUT_MAXPACKET), HBYTE(COMP_CDC_HS_OUT_MAXPACKET), 0x00,
#endif /* CDC_DEBUG_ENABLED */
};

/* ══════════════════════════════════════════════════════════════════════
 * Other-speed configuration (mirrors FS layout)
 * ══════════════════════════════════════════════════════════════════════ */

#if defined ( __ICCARM__ )
  #pragma data_alignment=4
#endif
ALIGNED_HEAD static uint8_t g_usbd_other_speed[USBD_COMP_CONFIG_DESC_SIZE] ALIGNED_TAIL =
{
  USB_DEVICE_CFG_DESC_LEN,
  USB_DESCIPTOR_TYPE_CONFIGURATION,
  LBYTE(USBD_COMP_CONFIG_DESC_SIZE),
  HBYTE(USBD_COMP_CONFIG_DESC_SIZE),
  USBD_COMP_NUM_INTERFACES, 0x01, 0x00, 0xC0, 0x32,

#ifdef CDC_DEBUG_ENABLED
  /* IAD WinUSB */
  0x08, 0x0B, COMP_WINUSB_INTERFACE, 0x01, 0xFF, 0x00, 0x00, 0x00,
#endif

  /* Interface 0 — WinUSB */
  USB_DEVICE_IF_DESC_LEN, USB_DESCIPTOR_TYPE_INTERFACE,
  COMP_WINUSB_INTERFACE, 0x00, 0x02, 0xFF, 0x00, 0x00, 0x00,

  /* EP1 OUT bulk 64 */
  USB_DEVICE_EPT_LEN, USB_DESCIPTOR_TYPE_ENDPOINT,
  COMP_WINUSB_BULK_OUT_EPT, USB_EPT_DESC_BULK,
  LBYTE(COMP_WINUSB_OUT_MAXPACKET), HBYTE(COMP_WINUSB_OUT_MAXPACKET), 0x00,

  /* EP1 IN bulk 64 */
  USB_DEVICE_EPT_LEN, USB_DESCIPTOR_TYPE_ENDPOINT,
  COMP_WINUSB_BULK_IN_EPT, USB_EPT_DESC_BULK,
  LBYTE(COMP_WINUSB_IN_MAXPACKET), HBYTE(COMP_WINUSB_IN_MAXPACKET), 0x00,

#ifndef LA_ONLY_BUILD
  /* Interface 1 — CMSIS-DAP v1 HID */
  USB_DEVICE_IF_DESC_LEN, USB_DESCIPTOR_TYPE_INTERFACE,
  COMP_DAP_INTERFACE, 0x00, 0x02, 0x03, 0x00, 0x00, USB_INTERFACE_STRING,

  /* HID class descriptor */
  0x09, 0x21, 0x10, 0x01, 0x00, 0x01, 0x22,
  COMP_DAP_HID_REPORT_SIZE & 0xFF, COMP_DAP_HID_REPORT_SIZE >> 8,

  /* EP2 OUT interrupt 64 */
  USB_DEVICE_EPT_LEN, USB_DESCIPTOR_TYPE_ENDPOINT,
  COMP_DAP_BULK_OUT_EPT, USB_EPT_DESC_INTERRUPT,
  LBYTE(COMP_DAP_OUT_MAXPACKET), HBYTE(COMP_DAP_OUT_MAXPACKET), 0x01,

  /* EP2 IN interrupt 64 */
  USB_DEVICE_EPT_LEN, USB_DESCIPTOR_TYPE_ENDPOINT,
  COMP_DAP_BULK_IN_EPT, USB_EPT_DESC_INTERRUPT,
  LBYTE(COMP_DAP_IN_MAXPACKET), HBYTE(COMP_DAP_IN_MAXPACKET), 0x01,
#endif /* !LA_ONLY_BUILD */

#ifdef CDC_DEBUG_ENABLED
  /* IAD CDC */
  0x08, 0x0B, COMP_CDC_COM_INTERFACE, 0x02, 0x02, 0x02, 0x01, 0x00,

  /* Interface 1 — CDC Comm */
  USB_DEVICE_IF_DESC_LEN, USB_DESCIPTOR_TYPE_INTERFACE,
  COMP_CDC_COM_INTERFACE, 0x00, 0x01, USB_CLASS_CODE_CDC, 0x02, 0x01, 0x00,

  0x05, USBD_CDC_CS_INTERFACE, USBD_CDC_SUBTYPE_HEADER, 0x10, 0x01,
  0x05, USBD_CDC_CS_INTERFACE, USBD_CDC_SUBTYPE_CMF, 0x00, COMP_CDC_DATA_INTERFACE,
  0x04, USBD_CDC_CS_INTERFACE, USBD_CDC_SUBTYPE_ACM, 0x02,
  0x05, USBD_CDC_CS_INTERFACE, USBD_CDC_SUBTYPE_UFD, COMP_CDC_COM_INTERFACE, COMP_CDC_DATA_INTERFACE,

  /* EP3 IN interrupt 8 */
  USB_DEVICE_EPT_LEN, USB_DESCIPTOR_TYPE_ENDPOINT,
  COMP_CDC_INT_EPT, USB_EPT_DESC_INTERRUPT,
  LBYTE(COMP_CDC_CMD_MAXPACKET), HBYTE(COMP_CDC_CMD_MAXPACKET), 0xFF,

  /* Interface 2 — CDC Data */
  USB_DEVICE_IF_DESC_LEN, USB_DESCIPTOR_TYPE_INTERFACE,
  COMP_CDC_DATA_INTERFACE, 0x00, 0x02, USB_CLASS_CODE_CDCDATA, 0x00, 0x00, 0x00,

  /* EP2 IN bulk 64 */
  USB_DEVICE_EPT_LEN, USB_DESCIPTOR_TYPE_ENDPOINT,
  COMP_CDC_BULK_IN_EPT, USB_EPT_DESC_BULK,
  LBYTE(COMP_CDC_IN_MAXPACKET), HBYTE(COMP_CDC_IN_MAXPACKET), 0x00,

  /* EP2 OUT bulk 64 */
  USB_DEVICE_EPT_LEN, USB_DESCIPTOR_TYPE_ENDPOINT,
  COMP_CDC_BULK_OUT_EPT, USB_EPT_DESC_BULK,
  LBYTE(COMP_CDC_OUT_MAXPACKET), HBYTE(COMP_CDC_OUT_MAXPACKET), 0x00,
#endif /* CDC_DEBUG_ENABLED */
};

/* ══════════════════════════════════════════════════════════════════════
 * Device qualifier
 * ══════════════════════════════════════════════════════════════════════ */

#if defined ( __ICCARM__ )
  #pragma data_alignment=4
#endif
ALIGNED_HEAD static uint8_t g_usbd_qualifier[USB_DEVICE_QUALIFIER_DESC_LEN] ALIGNED_TAIL =
{
  USB_DEVICE_QUALIFIER_DESC_LEN,
  USB_DESCIPTOR_TYPE_DEVICE_QUALIFIER,
  0x00, 0x02,                            /* bcdUSB 2.0 */
#ifdef CDC_DEBUG_ENABLED
  0xEF, 0x02, 0x01,                      /* class/sub/proto: IAD */
#else
  0xFF, 0x00, 0x00,                      /* class/sub/proto: vendor */
#endif
  0x40,                                  /* bMaxPacketSize0 */
  0x01,                                  /* bNumConfigurations */
  0x00,
};

/* ══════════════════════════════════════════════════════════════════════
 * String descriptors
 * ══════════════════════════════════════════════════════════════════════ */

#if defined ( __ICCARM__ )
  #pragma data_alignment=4
#endif
ALIGNED_HEAD static uint8_t g_string_lang_id[USBD_COMP_SIZ_STRING_LANGID] ALIGNED_TAIL =
{
  USBD_COMP_SIZ_STRING_LANGID,
  USB_DESCIPTOR_TYPE_STRING,
  0x09, 0x04,          /* English (US) */
};

#if defined ( __ICCARM__ )
  #pragma data_alignment=4
#endif
ALIGNED_HEAD static uint8_t g_string_serial[USBD_COMP_SIZ_STRING_SERIAL] ALIGNED_TAIL =
{
  USBD_COMP_SIZ_STRING_SERIAL,
  USB_DESCIPTOR_TYPE_STRING,
};

/* ══════════════════════════════════════════════════════════════════════
 * MS OS 1.0 descriptors  (WinUSB auto-install for interface 0)
 * ══════════════════════════════════════════════════════════════════════ */

#if USBD_SUPPORT_WINUSB == 1

/* OS String descriptor — tells Windows the vendor code is 0xA0 */
#if defined ( __ICCARM__ )
  #pragma data_alignment=4
#endif
ALIGNED_HEAD static uint8_t g_winusb_os_string[18] ALIGNED_TAIL =
{
  0x12,               /* bLength = 18 */
  USB_DESCIPTOR_TYPE_STRING,
  /* "MSFT100" in UTF-16LE */
  'M', 0, 'S', 0, 'F', 0, 'T', 0, '1', 0, '0', 0, '0', 0,
  WINUSB_BMS_VENDOR_CODE,               /* bMS_VendorCode */
  0x00,               /* bPad */
};

/* Extended Compat ID — 2 functions: interface 0 + 1 → "WINUSB" */
#if defined ( __ICCARM__ )
  #pragma data_alignment=4
#endif
ALIGNED_HEAD static uint8_t g_winusb_os_feature[40] ALIGNED_TAIL =
{
  /* Header  (16 bytes) */
  0x28, 0x00, 0x00, 0x00,               /* dwLength  = 40 */
  0x00, 0x01,                            /* bcdVersion = 1.00 */
  0x04, 0x00,                            /* wIndex: Extended Compat ID */
  0x01,                                  /* bCount: 1 function (LA only) */
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* reserved */

  /* Function section 0  (24 bytes) — LA WinUSB */
  0x00,                                  /* bFirstInterfaceNumber: 0 */
  0x01,                                  /* bReserved */
  'W', 'I', 'N', 'U', 'S', 'B', 0, 0,  /* compatibleID */
  0, 0, 0, 0, 0, 0, 0, 0,              /* subCompatibleID */
  0, 0, 0, 0, 0, 0,                     /* reserved */
};

/* Extended Properties — DeviceInterfaceGUID for interface 0 */
#if defined ( __ICCARM__ )
  #pragma data_alignment=4
#endif
ALIGNED_HEAD static uint8_t g_winusb_os_property[142] ALIGNED_TAIL =
{
  /* Header (10 bytes) */
  0x8E, 0x00, 0x00, 0x00,               /* dwLength = 142 */
  0x00, 0x01,                            /* bcdVersion = 1.00 */
  0x05, 0x00,                            /* wIndex: Extended Properties */
  0x01, 0x00,                            /* wCount: 1 property */

  /* Property section */
  0x84, 0x00, 0x00, 0x00,               /* dwSize = 132 */
  0x01, 0x00, 0x00, 0x00,               /* dwPropertyDataType: REG_SZ */

  /* wPropertyNameLength = 40 */
  0x28, 0x00,
  /* PropertyName: "DeviceInterfaceGUID" in UTF-16LE (40 bytes with null) */
  'D',0, 'e',0, 'v',0, 'i',0, 'c',0, 'e',0, 'I',0, 'n',0,
  't',0, 'e',0, 'r',0, 'f',0, 'a',0, 'c',0, 'e',0, 'G',0,
  'U',0, 'I',0, 'D',0, 0,0,

  /* wPropertyDataLength = 78 */
  0x4E, 0x00, 0x00, 0x00,
  /* PropertyData: GUID string in UTF-16LE (78 bytes with null)
     {6895393B-2900-47F4-98C0-76031A56885A} */
  '{',0, '6',0, '8',0, '9',0, '5',0, '3',0, '9',0, '3',0,
  'B',0, '-',0, '2',0, '9',0, '0',0, '0',0, '-',0, '4',0,
  '7',0, 'F',0, '4',0, '-',0, '9',0, '8',0, 'C',0, '0',0,
  '-',0, '7',0, '6',0, '0',0, '3',0, '1',0, 'A',0, '5',0,
  '6',0, '8',0, '8',0, '5',0, 'A',0, '}',0, 0,0,
};

#endif /* USBD_SUPPORT_WINUSB */

/* ──────────────────────────────────────────────────────────────────────
 * usbd_desc_t wrappers
 * ────────────────────────────────────────────────────────────────────── */

static usbd_desc_t device_descriptor    = { USB_DEVICE_DESC_LEN,              g_usbd_descriptor };
static usbd_desc_t config_descriptor    = { USBD_COMP_CONFIG_DESC_SIZE,       g_usbd_configuration };
static usbd_desc_t config_descriptor_hs = { USBD_COMP_CONFIG_DESC_SIZE,       g_usbd_hs_configuration };
static usbd_desc_t config_other_speed   = { USBD_COMP_CONFIG_DESC_SIZE,       g_usbd_other_speed };
static usbd_desc_t langid_descriptor    = { USBD_COMP_SIZ_STRING_LANGID,      g_string_lang_id };
static usbd_desc_t serial_descriptor    = { USBD_COMP_SIZ_STRING_SERIAL,      g_string_serial };
static usbd_desc_t device_qualifier     = { USB_DEVICE_QUALIFIER_DESC_LEN,    g_usbd_qualifier };
static usbd_desc_t vp_desc;

#if USBD_SUPPORT_WINUSB == 1
static usbd_desc_t os_string_desc  = { sizeof(g_winusb_os_string),   g_winusb_os_string };
static usbd_desc_t os_feature_desc = { sizeof(g_winusb_os_feature),  g_winusb_os_feature };
static usbd_desc_t os_property_desc= { sizeof(g_winusb_os_property), g_winusb_os_property };
#endif

/* ──────────────────────────────────────────────────────────────────────
 * Helper functions
 * ────────────────────────────────────────────────────────────────────── */

static uint16_t usbd_unicode_convert(uint8_t *string, uint8_t *unicode_buf)
{
  uint16_t str_len = 0, id_pos = 2;
  uint8_t *tmp = string;
  while (*tmp != '\0') { str_len++; unicode_buf[id_pos++] = *tmp++; unicode_buf[id_pos++] = 0x00; }
  str_len = str_len * 2 + 2;
  unicode_buf[0] = (uint8_t)str_len;
  unicode_buf[1] = USB_DESCIPTOR_TYPE_STRING;
  return str_len;
}

static void usbd_int_to_unicode(uint32_t value, uint8_t *pbuf, uint8_t len)
{
  uint8_t idx;
  for (idx = 0; idx < len; idx++) {
    if ((value >> 28) < 0xA)  pbuf[2*idx] = (value >> 28) + '0';
    else                      pbuf[2*idx] = (value >> 28) + 'A' - 10;
    value <<= 4;
    pbuf[2*idx + 1] = 0;
  }
}

/* 48-bit hash of the chip's full 96-bit UID, for the USB serial.
 *
 * FNV-1a (64-bit) over all 12 UID bytes so every bit of the unique ID
 * contributes, then a splitmix64 finalizer so the result avalanches (two chips
 * whose UIDs differ by a single bit get completely different serials — easy to
 * read apart, unlike the raw UID). We keep the low 48 bits → 12 hex.
 *
 * Sized for production: 48-bit space ⇒ ~1-in-5.6-million birthday-collision
 * chance across 10k units (≈1-in-56k at 100k). Go 64-bit if scaling to millions.
 */
static uint64_t uid_hash48(void)
{
  const uint8_t *p = (const uint8_t *)MCU_ID1;   /* ID1,ID2,ID3 = 12 contiguous bytes */
  uint64_t h = 0xcbf29ce484222325ull;            /* FNV-1a 64 offset basis */
  for (int i = 0; i < 12; i++) { h ^= p[i]; h *= 0x100000001b3ull; }
  h ^= h >> 30; h *= 0xbf58476d1ce4e5b9ull;      /* splitmix64 finalizer */
  h ^= h >> 27; h *= 0x94d049bb133111ebull;
  h ^= h >> 31;
  return h & 0xFFFFFFFFFFFFull;                  /* low 48 bits */
}

static void get_serial_num(void)
{
  uint64_t h = uid_hash48();
  /* 12 hex = top 16 bits (4 hex) then low 32 bits (8 hex). usbd_int_to_unicode
   * renders from the MSB of a 32-bit value, so left-justify the 16-bit part. */
  usbd_int_to_unicode((uint32_t)((h >> 32) & 0xFFFFu) << 16, &g_string_serial[2],  4);
  usbd_int_to_unicode((uint32_t)h,                           &g_string_serial[10], 8);
}

/* ──────────────────────────────────────────────────────────────────────
 * Getter callbacks
 * ────────────────────────────────────────────────────────────────────── */

static usbd_desc_t *get_device_descriptor(void)       { return &device_descriptor; }
static usbd_desc_t *get_device_qualifier(void)         { return &device_qualifier; }
static usbd_desc_t *get_device_configuration(void)     { return &config_descriptor; }
static usbd_desc_t *get_hs_device_configuration(void)  { return &config_descriptor_hs; }
static usbd_desc_t *get_device_other_speed(void)       { return &config_other_speed; }
static usbd_desc_t *get_device_lang_id(void)           { return &langid_descriptor; }

static usbd_desc_t *get_device_manufacturer_string(void) {
  vp_desc.length = usbd_unicode_convert((uint8_t *)USBD_COMP_DESC_MANUFACTURER_STRING, g_usbd_desc_buffer);
  vp_desc.descriptor = g_usbd_desc_buffer;
  return &vp_desc;
}
static usbd_desc_t *get_device_product_string(void) {
  vp_desc.length = usbd_unicode_convert((uint8_t *)USBD_COMP_DESC_PRODUCT_STRING, g_usbd_desc_buffer);
  vp_desc.descriptor = g_usbd_desc_buffer;
  return &vp_desc;
}
static usbd_desc_t *get_device_serial_string(void) {
  get_serial_num();
  return &serial_descriptor;
}
static usbd_desc_t *get_device_interface_string(void) {
  vp_desc.length = usbd_unicode_convert((uint8_t *)USBD_COMP_DESC_INTERFACE_STRING, g_usbd_desc_buffer);
  vp_desc.descriptor = g_usbd_desc_buffer;
  return &vp_desc;
}
static usbd_desc_t *get_device_config_string(void) {
  vp_desc.length = usbd_unicode_convert((uint8_t *)USBD_COMP_DESC_CONFIGURATION_STRING, g_usbd_desc_buffer);
  vp_desc.descriptor = g_usbd_desc_buffer;
  return &vp_desc;
}

#if USBD_SUPPORT_WINUSB == 1
static usbd_desc_t *get_device_winusb_os_string(void)   { return &os_string_desc; }
static usbd_desc_t *get_device_winusb_os_feature(void)   { return &os_feature_desc; }
static usbd_desc_t *get_device_winusb_os_property(void)  { return &os_property_desc; }
#endif
