#include "lcd_spi_slave.h"
#include "esp_log.h"
#include "string.h"
#include "driver/gpio.h"
#include "esp_attr.h"

#ifndef CONFIG_LOG_MAXIMUM_LEVEL
#define CONFIG_LOG_MAXIMUM_LEVEL 3
#endif

static const char *TAG = "LCD_SPI";

static void IRAM_ATTR spi_post_setup_cb(spi_slave_transaction_t *trans)
{
    // 在每次傳輸開始前檢查 CD 信號
    bool is_data = gpio_get_level(LCD_PIN_CD);
    if (is_data) {
        ESP_EARLY_LOGV(TAG, "資料傳輸開始");
    } else {
        ESP_EARLY_LOGV(TAG, "命令傳輸開始");
    }
}

static void IRAM_ATTR spi_post_trans_cb(spi_slave_transaction_t *trans)
{
    // 在傳輸完成後可以處理 CD 信號狀態
    bool is_data = gpio_get_level(LCD_PIN_CD);
    if (is_data) {
        ESP_EARLY_LOGV(TAG, "資料傳輸完成");
    } else {
        ESP_EARLY_LOGV(TAG, "命令傳輸完成");
    }
}

esp_err_t lcd_spi_slave_init(lcd_spi_slave_t *lcd_handle)
{
    if (!lcd_handle) {
        return ESP_ERR_INVALID_ARG;
    }

    // 初始化結構
    memset(lcd_handle, 0, sizeof(lcd_spi_slave_t));

    // 配置 CD PIN
    gpio_config_t cd_conf = {
        .pin_bit_mask = (1ULL << LCD_PIN_CD),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&cd_conf);

    // 建立事件佇列
    lcd_handle->event_queue = xQueueCreate(LCD_QUEUE_SIZE, sizeof(spi_slave_transaction_t));
    if (!lcd_handle->event_queue) {
        ESP_LOGE(TAG, "無法建立事件佇列");
        return ESP_ERR_NO_MEM;
    }

    // 配置 SPI Slave
    spi_bus_config_t buscfg = {
        .miso_io_num = LCD_PIN_MISO,
        .mosi_io_num = LCD_PIN_MOSI,
        .sclk_io_num = LCD_PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_MAX_SIZE,
    };

    spi_slave_interface_config_t slvcfg = {
        .mode = 0,                    // CPOL=0, CPHA=0
        .spics_io_num = LCD_PIN_CS,
        .queue_size = LCD_QUEUE_SIZE,
        .flags = 0,
        .post_setup_cb = spi_post_setup_cb,  // 傳輸開始回調
        .post_trans_cb = spi_post_trans_cb   // 傳輸完成回調
    };

    // 初始化 SPI Slave
    esp_err_t ret = spi_slave_initialize(LCD_SPI_HOST, &buscfg, &slvcfg, LCD_DMA_CHAN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI Slave 初始化失敗");
        vQueueDelete(lcd_handle->event_queue);
        return ret;
    }

    // 準備第一個傳輸
    lcd_handle->trans.length = LCD_MAX_SIZE * 8;
    lcd_handle->trans.rx_buffer = lcd_handle->rx_buffer;
    lcd_handle->trans.tx_buffer = lcd_handle->tx_buffer;
    
    // 設置預設回應（狀態正常）
    memset(lcd_handle->tx_buffer, 0x00, LCD_MAX_SIZE);
    lcd_handle->tx_buffer[0] = 0x00; // 狀態正常

    // 啟動第一個傳輸
    ret = spi_slave_queue_trans(LCD_SPI_HOST, &lcd_handle->trans, portMAX_DELAY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "無法啟動 SPI 傳輸");
        spi_slave_free(LCD_SPI_HOST);
        vQueueDelete(lcd_handle->event_queue);
        return ret;
    }

    lcd_handle->initialized = true;
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "LCD SPI Slave 初始化完成");
    ESP_LOGI(TAG, "  MISO: GPIO %d", LCD_PIN_MISO);
    ESP_LOGI(TAG, "  MOSI: GPIO %d", LCD_PIN_MOSI);
    ESP_LOGI(TAG, "  SCLK: GPIO %d", LCD_PIN_SCLK);
    ESP_LOGI(TAG, "  CS:   GPIO %d", LCD_PIN_CS);
    ESP_LOGI(TAG, "  CD:   GPIO %d", LCD_PIN_CD);
    ESP_LOGI(TAG, "  Mode: 0 (CPOL=0, CPHA=0)");
    ESP_LOGI(TAG, "  等待 SPI 主設備資料中...");
    ESP_LOGI(TAG, "========================================");
    
    // 讀取初始 GPIO 狀態
    ESP_LOGI(TAG, "GPIO 初始狀態:");
    ESP_LOGI(TAG, "  CS=%d SCLK=%d MOSI=%d CD=%d",
             gpio_get_level(LCD_PIN_CS),
             gpio_get_level(LCD_PIN_SCLK),
             gpio_get_level(LCD_PIN_MOSI),
             gpio_get_level(LCD_PIN_CD));
    
    return ESP_OK;
}

esp_err_t lcd_spi_slave_process(lcd_spi_slave_t *lcd_handle)
{
    if (!lcd_handle || !lcd_handle->initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    spi_slave_transaction_t *trans;
    
    // 等待傳輸完成（設置1秒超時，讓我們能定期檢查狀態）
    esp_err_t ret = spi_slave_get_trans_result(LCD_SPI_HOST, &trans, pdMS_TO_TICKS(1000));
    if (ret == ESP_ERR_TIMEOUT) {
        // 超時，沒有收到 SPI 傳輸，這是正常的
        // 記錄 GPIO 狀態供調試
        static uint32_t timeout_count = 0;
        if (++timeout_count % 5 == 0) {  // 每5秒報告一次
            ESP_LOGD(TAG, "等待 SPI 資料 (CS=%d SCLK=%d MOSI=%d)",
                     gpio_get_level(LCD_PIN_CS),
                     gpio_get_level(LCD_PIN_SCLK),
                     gpio_get_level(LCD_PIN_MOSI));
        }
        return ESP_ERR_TIMEOUT;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "獲取 SPI 傳輸結果失敗: %s", esp_err_to_name(ret));
        return ret;
    }

    // 實際接收的位元組數
    size_t received_bytes = trans->trans_len / 8;
    lcd_handle->last_rx_len = received_bytes;
    
    // 快速處理，無日誌（最大化速度）
    bool is_data = gpio_get_level(LCD_PIN_CD);
    
    if (!is_data) {
        // 命令模式 - 簡單回應
        lcd_handle->tx_buffer[0] = 0x00;
    } else if (received_bytes > 0 && received_bytes <= LCD_MAX_SIZE) {
        // 資料模式 - 確保字串結尾
        lcd_handle->rx_buffer[received_bytes] = '\0';
    }

    // 準備下一次傳輸
    ret = spi_slave_queue_trans(LCD_SPI_HOST, &lcd_handle->trans, portMAX_DELAY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "無法啟動下一次 SPI 傳輸");
        return ret;
    }

    return ESP_OK;
}

void lcd_spi_slave_deinit(lcd_spi_slave_t *lcd_handle)
{
    if (!lcd_handle || !lcd_handle->initialized) {
        return;
    }

    spi_slave_free(LCD_SPI_HOST);
    vQueueDelete(lcd_handle->event_queue);
    memset(lcd_handle, 0, sizeof(lcd_spi_slave_t));
    
    ESP_LOGI(TAG, "LCD SPI Slave 已釋放");
}
