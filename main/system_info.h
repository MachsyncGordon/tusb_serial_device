#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief 系統資訊結構
 */
typedef struct {
    uint32_t flash_size;      // Flash 大小（位元組）
    uint32_t psram_size;      // PSRAM 大小（位元組）
    int psram_enabled;        // PSRAM 是否啟用（1=啟用，0=停用）
    const char* chip_model;   // 晶片型號
    uint32_t cpu_freq;        // CPU 頻率（Hz）
} system_info_t;

/**
 * @brief 獲取系統資訊
 * 
 * @param info 系統資訊結構指標
 * @return esp_err_t 
 *         - ESP_OK: 成功
 *         - ESP_ERR_INVALID_ARG: 參數錯誤
 */
esp_err_t get_system_info(system_info_t* info);
