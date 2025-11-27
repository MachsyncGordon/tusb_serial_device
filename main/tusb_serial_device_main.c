/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/* 
 * USB 序列埠裝置範例
 * 此範例展示如何使用 TinyUSB 堆疊實現一個 USB CDC-ACM (序列埠) 裝置
 * 當連接到電腦時，電腦會識別為一個虛擬序列埠
 * 程式會將收到的資料回傳給電腦，實現回音(echo)功能
 */

#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "sdkconfig.h"
#include "device/usbd.h"
#include "class/cdc/cdc_device.h"
#include "driver/gpio.h"
#include "lcd_spi_slave.h"

static const char *TAG = "example";

// USB 日誌輸出函數
int usb_log_print(const char* fmt, va_list args) {
    char buf[512];
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    if (len > 0) {
        tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, (uint8_t*)buf, len);
        tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, (uint8_t*)"\r\n", 2);
        tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
    }
    return len;
}
// 接收緩衝區，大小由 CONFIG_TINYUSB_CDC_RX_BUFSIZE 定義
static uint8_t rx_buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];

/**
 * @brief 應用程式訊息佇列
 * 用於在中斷和主任務之間傳遞接收到的資料
 */
static QueueHandle_t app_queue;
typedef struct {
    uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];     // 資料緩衝區
    size_t buf_len;                                     // 接收到的位元組數
    uint8_t itf;                                        // CDC 裝置介面索引
} app_message_t;

/**
 * @brief USB CDC 接收回調函數
 *
 * 當 USB CDC 裝置收到新資料時會觸發此回調
 * 只記錄接收到的資料，不做回應
 *
 * @param[in] itf   CDC 裝置索引
 * @param[in] event CDC 事件類型
 */
void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    // 讀取並清空接收緩衝區
    size_t rx_size = 0;
    esp_err_t ret = tinyusb_cdcacm_read(itf, rx_buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);
    
    if (ret == ESP_OK && rx_size > 0) {
        // 只記錄接收到的資料，不做任何回應
        ESP_LOGD(TAG, "接收到 %d 位元組資料（已忽略）", rx_size);
    }
}

/**
 * @brief USB CDC 線路狀態變更回調函數
 *
 * 當 USB CDC 裝置的 DTR 或 RTS 信號狀態改變時會觸發此回調
 * DTR (Data Terminal Ready) 和 RTS (Request To Send) 是序列通訊的控制信號
 *
 * @param[in] itf   CDC 裝置索引
 * @param[in] event CDC 事件類型
 */
void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    int dtr = event->line_state_changed_data.dtr;
    int rts = event->line_state_changed_data.rts;
    ESP_LOGI(TAG, "通道 %d 的線路狀態改變: DTR:%d, RTS:%d", itf, dtr, rts);
}

// 統計變數（全域，供兩個任務共用）
static volatile uint32_t g_spi_transactions = 0;
static volatile uint32_t g_spi_bytes = 0;
static volatile uint32_t g_spi_freq_count = 0;

// 資料累積緩衝區
static uint8_t accumulated_buffer[LCD_MAX_SIZE];
static size_t accumulated_len = 0;
static size_t expected_len = 0;  // 預期長度（F=9, P=5）

// 暫存 F 和 P 的值用於合併輸出
static char freq_value[8] = {0};  // 儲存頻率值 (例如 "26.781k")
static char power_value[5] = {0}; // 儲存電壓值 (例如 "16V")
static bool has_freq = false;

// SPI 統計報告任務（每秒發送統計到 USB）
static void spi_stats_task(void *pvParameters)
{
    ESP_LOGI(TAG, "### SPI 統計任務已啟動 ###");
    
    uint32_t counter = 0;
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));  // 每秒執行一次
        counter++;
        
        // 讀取統計資料
        uint32_t tx = g_spi_transactions;
        uint32_t bytes = g_spi_bytes;
        uint32_t freq = g_spi_freq_count;
        
        // 重置統計
        g_spi_transactions = 0;
        g_spi_bytes = 0;
        g_spi_freq_count = 0;
        
        // 簡化日誌（僅在有資料時記錄）
        if (tx > 0 || bytes > 0) {
            ESP_LOGD(TAG, "[#%lu] TX:%lu B:%lu F:%lu", counter, tx, bytes, freq);
        }
        
        // 不再發送統計到 USB（已停用）
    }
}

// LCD SPI Slave 處理任務
static void lcd_spi_slave_task(void *pvParameters)
{
    lcd_spi_slave_t *lcd_handle = (lcd_spi_slave_t *)pvParameters;
    
    ESP_LOGI(TAG, "### LCD SPI Slave 處理任務已啟動 ###");
    
    while (1) {
        // 處理 LCD SPI Slave（1秒超時）
        esp_err_t ret = lcd_spi_slave_process(lcd_handle);
        
        if (ret == ESP_ERR_TIMEOUT) {
            // 超時是正常的，表示沒有 SPI 資料
            continue;
        }
        
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPI 處理錯誤: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        // 成功接收到 SPI 資料
        // 檢查 CD 信號：只處理資料模式（CD=1）
        if (!gpio_get_level(LCD_PIN_CD)) {
            continue;  // 命令模式，忽略
        }
        
        // 使用實際傳輸長度
        size_t data_len = lcd_handle->last_rx_len;
        
        if (data_len > 0) {
            // 統計原始傳輸
            g_spi_transactions++;
            g_spi_bytes += data_len;
            
            // 累積資料到緩衝區（按固定格式）
            for (size_t i = 0; i < data_len; i++) {
                uint8_t byte = lcd_handle->rx_buffer[i];
                
                // 忽略不可打印字符
                if (byte < 0x20 || byte > 0x7E) {
                    continue;
                }
                
                // 檢測新包開始
                if (byte == 'F' || byte == 'P') {
                    // 重置累積
                    accumulated_len = 0;
                    accumulated_buffer[accumulated_len++] = byte;
                    // 設定預期長度：F=xx.xxxk (9字元), P=xxV/xxxV (5-6字元，偵測V結束)
                    expected_len = (byte == 'F') ? 9 : 6;  // P 設為最大6字元
                    continue;
                }
                
                // 累積字元
                if (accumulated_len > 0 && accumulated_len < LCD_MAX_SIZE) {
                    accumulated_buffer[accumulated_len++] = byte;
                    
                    // 對於 P 格式，偵測到 V 即完成；對於 F 格式，需達到固定長度
                    bool should_check = false;
                    if (accumulated_buffer[0] == 'F' && accumulated_len == expected_len) {
                        should_check = true;
                    } else if (accumulated_buffer[0] == 'P' && byte == 'V' && accumulated_len >= 4) {
                        should_check = true;
                    }
                    
                    // 檢查是否達到預期長度或偵測到結束字元
                    if (should_check) {
                        // 驗證格式
                        bool is_valid = false;
                        
                        if (accumulated_buffer[0] == 'F' && accumulated_len == 9) {
                            // 格式：F=xx.xxxk
                            // 位置：0=F, 1==, 2-7=數字/點, 8=k
                            if (accumulated_buffer[1] == '=' && accumulated_buffer[8] == 'k') {
                                is_valid = true;
                                // 驗證中間是數字和點
                                for (int j = 2; j < 8; j++) {
                                    char c = accumulated_buffer[j];
                                    if (!(c >= '0' && c <= '9') && c != '.') {
                                        is_valid = false;
                                        break;
                                    }
                                }
                            }
                        } else if (accumulated_buffer[0] == 'P' && accumulated_len >= 4) {
                            // 格式：P=xV, P=xxV 或 P=xxxV
                            // 找到 V 的位置
                            int v_pos = accumulated_len - 1;
                            if (accumulated_buffer[1] == '=' && accumulated_buffer[v_pos] == 'V') {
                                is_valid = true;
                                // 驗證中間是數字（從位置2到V之前）
                                for (int j = 2; j < v_pos; j++) {
                                    char c = accumulated_buffer[j];
                                    if (!(c >= '0' && c <= '9')) {
                                        is_valid = false;
                                        break;
                                    }
                                }
                            }
                        }
                        
                        // 處理有效資料
                        if (is_valid) {
                            if (accumulated_buffer[0] == 'F') {
                                // F=xx.xxxk → 提取 "xx.xxxk"
                                g_spi_freq_count++;
                                memcpy(freq_value, &accumulated_buffer[2], 6);  // 複製 "xx.xxx"
                                freq_value[6] = 'k';
                                freq_value[7] = '\0';
                                has_freq = true;
                            } else if (accumulated_buffer[0] == 'P' && has_freq) {
                                // P=xV, P=xxV 或 P=xxxV → 動態提取電壓值，補齊到3位數
                                int num_len = accumulated_len - 3;  // 減去 "P=V" 的長度
                                
                                // 根據數字位數補齊到3位（例如：5V→005V，16V→016V，81V→081V，100V→100V）
                                if (num_len == 1) {
                                    power_value[0] = '0';
                                    power_value[1] = '0';
                                    power_value[2] = accumulated_buffer[2];
                                    power_value[3] = 'V';
                                    power_value[4] = '\0';
                                } else if (num_len == 2) {
                                    power_value[0] = '0';
                                    power_value[1] = accumulated_buffer[2];
                                    power_value[2] = accumulated_buffer[3];
                                    power_value[3] = 'V';
                                    power_value[4] = '\0';
                                } else {  // num_len >= 3
                                    memcpy(power_value, &accumulated_buffer[2], num_len);
                                    power_value[num_len] = 'V';
                                    power_value[num_len + 1] = '\0';
                                }
                                
                                // 合併輸出：[DATA] 26.781k 100V
                                char output[64];
                                int len = snprintf(output, sizeof(output),
                                    "[DATA] %s %s\r\n", freq_value, power_value);
                                
                                tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, 
                                                          (uint8_t*)output, len);
                                tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
                                
                                // 重置標記
                                has_freq = false;
                            }
                        }
                        
                        // 重置累積
                        accumulated_len = 0;
                        expected_len = 0;
                    }
                }
            }
        }
    }
}

// LCD SPI Slave 句柄（靜態變數）
static lcd_spi_slave_t lcd_handle;

void app_main(void)
{
    ESP_LOGI(TAG, "=== USB CDC + SPI Slave 啟動 ===");
    
    // 建立 FreeRTOS 佇列
    app_queue = xQueueCreate(5, sizeof(app_message_t));
    assert(app_queue);
    app_message_t msg;
    
    // LCD SPI Slave 初始化
    ESP_LOGI(TAG, "LCD SPI Slave 初始化開始");
    ESP_ERROR_CHECK(lcd_spi_slave_init(&lcd_handle));
    
    // 建立 LCD SPI Slave 處理任務
    BaseType_t ret = xTaskCreatePinnedToCore(
        lcd_spi_slave_task,      // 任務函數
        "lcd_spi_slave",         // 任務名稱
        4096,                    // 堆疊大小
        &lcd_handle,             // 任務參數
        6,                       // 優先級（高於主任務）
        NULL,                    // 任務句柄
        1                        // 固定到 CPU 1
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "無法創建 LCD SPI Slave 任務");
    } else {
        ESP_LOGI(TAG, "LCD SPI Slave 任務已創建");
    }
    
    // 建立 SPI 統計任務
    ret = xTaskCreatePinnedToCore(
        spi_stats_task,          // 任務函數
        "spi_stats",             // 任務名稱
        3072,                    // 堆疊大小
        NULL,                    // 任務參數
        5,                       // 優先級
        NULL,                    // 任務句柄
        0                        // 固定到 CPU 0
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "無法創建 SPI 統計任務");
    } else {
        ESP_LOGI(TAG, "SPI 統計任務已創建");
    }

    ESP_LOGI(TAG, "USB 初始化開始");
    
    // 配置 TinyUSB 驅動程式
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,          // 使用預設的裝置描述符
        .string_descriptor = NULL,          // 使用預設的字串描述符
        .external_phy = false,              // 不使用外部 PHY
        .configuration_descriptor = NULL,    // 使用預設的配置描述符
        .self_powered = true,               // 設置為自供電模式
        .vbus_monitor_io = -1,              // 不監控 VBUS
    };

    // 安裝 TinyUSB 驅動程式
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    // 配置 CDC-ACM (序列埠)
    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,                    // 使用 USB 裝置 0
        .cdc_port = TINYUSB_CDC_ACM_0,                  // 使用 CDC-ACM 埠 0
        .rx_unread_buf_sz = 512,                        // 增加接收緩衝區大小
        .callback_rx = &tinyusb_cdc_rx_callback,        // 啟用接收回調
        .callback_rx_wanted_char = NULL,                // 不使用特定字元接收回調
        .callback_line_state_changed = &tinyusb_cdc_line_state_changed_callback,  // 啟用線路狀態回調
        .callback_line_coding_changed = NULL            // 不使用線路編碼回調
    };

    // 初始化 CDC-ACM
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));

#if (CONFIG_TINYUSB_CDC_COUNT > 1)
    // 如果啟用了多個 CDC 埠，初始化第二個 CDC-ACM 埠
    acm_cfg.cdc_port = TINYUSB_CDC_ACM_1;
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
    ESP_ERROR_CHECK(tinyusb_cdcacm_register_callback(
                        TINYUSB_CDC_ACM_1,
                        CDC_EVENT_LINE_STATE_CHANGED,
                        &tinyusb_cdc_line_state_changed_callback));
#endif

    ESP_LOGI(TAG, "USB 初始化完成");
    ESP_LOGI(TAG, "等待 USB 連接...");
    
    // 主迴圈：簡單等待
    while (1) {
        // 主任務只需保持運行，所有工作由其他任務完成
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
