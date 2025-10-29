#include "system_info.h"
#include "esp_system.h"
#include "esp_partition.h"        // 用於 flash 操作
#include "esp_heap_caps.h"
#include "esp_chip_info.h"
#include "esp_log.h"
#include "sdkconfig.h"            // 用於獲取配置值

static const char *TAG = "SYS_INFO";

esp_err_t get_system_info(system_info_t* info)
{
    if (!info) {
        return ESP_ERR_INVALID_ARG;
    }

    // 獲取晶片資訊
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    // 設置晶片型號
    switch (chip_info.model) {
        case CHIP_ESP32S3:
            info->chip_model = "ESP32-S3";
            break;
        default:
            info->chip_model = "Unknown";
            break;
    }

    // 獲取 Flash 大小
    const esp_partition_t *partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, NULL);
    if (partition != NULL) {
        // 使用分區表資訊來估算 Flash 大小
        info->flash_size = partition->address + partition->size;  // 這會得到至少的 Flash 大小
    } else {
        info->flash_size = 4 * 1024 * 1024;  // 預設 4MB
        ESP_LOGW(TAG, "無法獲取分區資訊，使用預設值");
    }
    ESP_LOGI(TAG, "Flash 大小: %d MB", info->flash_size / (1024 * 1024));

    // 檢查 PSRAM
    size_t psram_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    if (psram_size > 0) {
        info->psram_enabled = 1;
        info->psram_size = psram_size;
        ESP_LOGI(TAG, "PSRAM 大小: %d MB", info->psram_size / (1024 * 1024));
    } else {
        info->psram_enabled = 0;
        info->psram_size = 0;
        ESP_LOGI(TAG, "PSRAM 未啟用或未安裝");
    }

    // 獲取 CPU 頻率
    uint32_t cpu_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    info->cpu_freq = cpu_freq_mhz * 1000000;
    ESP_LOGI(TAG, "CPU 頻率: %d MHz", cpu_freq_mhz);

    return ESP_OK;
}
