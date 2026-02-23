/*
 * Project TinyUSB override for RP2040/RP2350 builds.
 * Enables a second CDC interface for dedicated NMEA passthrough.
 */

#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------

#ifndef CFG_TUSB_MCU
#define CFG_TUSB_MCU OPT_MCU_RP2040
#endif

#define CFG_TUSB_RHPORT0_MODE OPT_MODE_DEVICE
#define CFG_TUSB_OS OPT_OS_PICO

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG 0
#endif

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN __attribute__((aligned(4)))
#endif

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------

#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE 64
#endif

//------------- CLASS -------------//
#define CFG_TUD_HID (2)
#define CFG_TUD_CDC (2)
#define CFG_TUD_MSC (1)
#define CFG_TUD_MIDI (1)
#define CFG_TUD_VENDOR (0)

#define CFG_TUD_CDC_RX_BUFSIZE (256)
#define CFG_TUD_CDC_TX_BUFSIZE (256)

#define CFG_TUD_MSC_EP_BUFSIZE (64)

#define CFG_TUD_HID_EP_BUFSIZE (64)

#define CFG_TUD_MIDI_RX_BUFSIZE (64)
#define CFG_TUD_MIDI_TX_BUFSIZE (64)

#ifdef __cplusplus
}
#endif

#endif /* _TUSB_CONFIG_H_ */
