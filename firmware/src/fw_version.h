/**
  **************************************************************************
  * @file     fw_version.h
  * @brief    Single source of truth for the firmware version.
  *
  * Included by:
  *   - main.c          — UART banner + boot-LED version digits
  *   - composite_desc.c — USB bcdDevice (reported at enumeration)
  *
  * Bump FW_VER_* here and every consumer follows automatically, including
  * what the host sees on the USB wire.
  **************************************************************************
  */
#ifndef FW_VERSION_H
#define FW_VERSION_H

#define FW_VER_MAJOR  1
#define FW_VER_MINOR  1
#define FW_VER_PATCH  2

/* String form, e.g. "1.0.5" — shared by main.c (UART banner / LED) and the USB
 * product string, so the human-readable version tracks FW_VER_* automatically. */
#define _FW_VSTR(x)   #x
#define _FW_VXSTR(x)  _FW_VSTR(x)
#define FW_VERSION_STR  _FW_VXSTR(FW_VER_MAJOR) "." _FW_VXSTR(FW_VER_MINOR) "." _FW_VXSTR(FW_VER_PATCH)

/* USB bcdDevice encoding: 0xMMmp — major in the high byte, minor and patch in
 * the two nibbles of the low byte. e.g. 1.1.0 -> 0x0110. Read by the host during
 * standard enumeration with NO functional interface opened; Windows surfaces it
 * in the hardware id as REV_0110 — the lowest-risk "is this board compatible?"
 * probe.
 * IMPORTANT: bcdDevice is BCD — Windows renders each nibble as a DECIMAL digit
 * ('0'+nibble), so every nibble (major-hi, major-lo, minor, patch) MUST be 0..9.
 * A nibble of 10..15 renders as junk (e.g. 0xA -> ':'), which breaks REV_ and
 * the version probe. (1.0.10 hit exactly this — hence 1.1.0.) So bump minor on
 * reaching .9, and keep major <= 9 with this simple shift encoding.
 */
#define FW_BCD_DEVICE     ( ((FW_VER_MAJOR) << 8) | ((FW_VER_MINOR) << 4) | (FW_VER_PATCH) )
#define FW_BCD_DEVICE_LO  ( (FW_BCD_DEVICE) & 0xFF )
#define FW_BCD_DEVICE_HI  ( ((FW_BCD_DEVICE) >> 8) & 0xFF )

#endif /* FW_VERSION_H */
