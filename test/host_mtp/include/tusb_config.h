#pragma once
#include "custom_tusb_config.h"
#undef CFG_TUSB_MCU
#define CFG_TUSB_MCU OPT_MCU_NONE
#undef CFG_TUSB_OS
#define CFG_TUSB_OS OPT_OS_NONE
#define CFG_TUD_ENDPPOINT_MAX 16
#define TUP_DCD_ENDPOINT_MAX 16
