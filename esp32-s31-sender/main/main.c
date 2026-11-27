/* ============================================================================
 * ESP32-S31 摄像头采集端
 *
 * 摄像头 OV2640/OV5640 的接线完全沿用原工程（camera_preview.h 上半部分），
 * 原来接 ILI9341 的 42/43/44/45 脚改为 SPI 链路，把 240x240 RGB565
 * 画面逐行发给 STM32F103CBT6；屏幕本身已物理移到 STM32 那边。
 * ========================================================================== */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "camera_preview.h"

static const char *TAG = "APP";

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "CAM SENDER BUILD: %s %s", __DATE__, __TIME__);
    ESP_LOGI(TAG, "========================================");

    /* 摄像头未插好/供电不足时不 abort，否则芯片不断重启看起来像烧不进去。
     * 摄像头起不来时改跑 SPI 链路自测：发彩条而不是干等，
     * 这样能把"摄像头接触不良"和"两板连线不对"分开判断。 */
    esp_err_t err = camera_preview_init();
    if (err == ESP_OK) {
        err = camera_link_run();
    } else {
        ESP_LOGW(TAG, "camera init failed: %s -> SPI link self-test", esp_err_to_name(err));
        err = camera_link_selftest();
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "camera->STM32 link failed: %s; check wiring/power",
                 esp_err_to_name(err));
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
