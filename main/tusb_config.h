// main/tusb_config.h
#pragma once
#include <sdkconfig.h>
#include "tusb_option.h"

#define CFG_TUSB_RHPORT0_MODE    (OPT_MODE_DEVICE)
#define CFG_TUD_ENDPOINT0_SIZE   64

// 啟用的裝置類別
#define CFG_TUD_CDC              1   // 啟用 CDC (COM Port)
#define CFG_TUD_MSC              0   // 停用 Mass Storage
#define CFG_TUD_HID              0   // 停用 HID
#define CFG_TUD_MIDI             0   // 停用 MIDI
#define CFG_TUD_VENDOR           0   // 停用 Vendor
#define CFG_TUD_AUDIO            0   // 停用 Audio

// Audio 功能已停用
