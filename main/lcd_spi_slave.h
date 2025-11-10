#pragma once

#include "esp_err.h"
#include "driver/spi_slave.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ========= SPI Slave 硬體配置 =========
// SPI 腳位定義（請根據實際硬體連接調整）
#define LCD_SPI_HOST    SPI2_HOST
#define LCD_PIN_MISO    12      // SPI MISO 信號
#define LCD_PIN_MOSI    9      // SPI MOSI 信號
#define LCD_PIN_SCLK    8      // SPI SCLK 信號
#define LCD_PIN_CS      7       // SPI CS 信號
#define LCD_PIN_CD      10       // Command/Data 選擇信號（高電位=資料，低電位=命令）

// DMA 配置
#define LCD_DMA_CHAN    SPI_DMA_CH_AUTO  // ESP32-S3 需使用自動分配
#define LCD_QUEUE_SIZE  3
#define LCD_MAX_SIZE    64      // 最大傳輸大小（字節）

// ADSP-BF592 相容的命令格式
typedef enum {
    LCD_CMD_WRITE = 0x02,      // 寫入命令
    LCD_CMD_READ = 0x03,       // 讀取命令
    LCD_CMD_STATUS = 0x05      // 狀態查詢
} lcd_cmd_t;

// LCD 狀態結構
typedef struct {
    spi_slave_transaction_t trans;   // SPI 傳輸結構
    uint8_t rx_buffer[LCD_MAX_SIZE]; // 接收緩衝區
    uint8_t tx_buffer[LCD_MAX_SIZE]; // 發送緩衝區
    QueueHandle_t event_queue;       // 事件佇列
    bool initialized;                // 初始化標誌
    size_t last_rx_len;              // 上次接收的資料長度
} lcd_spi_slave_t;

/**
 * @brief 初始化 LCD SPI Slave
 * 
 * @param lcd_handle LCD 控制結構指標
 * @return esp_err_t 
 *         - ESP_OK: 成功
 *         - ESP_ERR_INVALID_ARG: 參數錯誤
 *         - ESP_ERR_NO_MEM: 記憶體不足
 */
esp_err_t lcd_spi_slave_init(lcd_spi_slave_t *lcd_handle);

/**
 * @brief 處理接收到的 SPI 資料
 * 
 * @param lcd_handle LCD 控制結構指標
 * @return esp_err_t 
 *         - ESP_OK: 成功
 *         - ESP_ERR_INVALID_ARG: 參數錯誤
 *         - ESP_ERR_TIMEOUT: 超時
 */
esp_err_t lcd_spi_slave_process(lcd_spi_slave_t *lcd_handle);

/**
 * @brief 釋放 LCD SPI Slave 資源
 * 
 * @param lcd_handle LCD 控制結構指標
 */
void lcd_spi_slave_deinit(lcd_spi_slave_t *lcd_handle);
