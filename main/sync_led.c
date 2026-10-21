#include "sync_led.h"
#include "app_config.h"
#include "ptpv1.h"
#include "media_clock.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "sync_led";

static void sync_led_task(void *arg)
{
    (void)arg;
    for (uint32_t tick = 0;; tick++) {      // 50 ms per tick
        bool on;
        if (!g_ptpv1.have_master)  on = false;
        else if (!g_ptpv1.locked)  on = (tick & 1) == 0;          // 10 Hz flash: 5 Hz blink
        else if (!g_mclk.armed)    on = (tick % 20) < 10;         // 1 Hz
        else                       on = true;
        gpio_set_level(AP_PIN_SYNC_LED, on);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

esp_err_t sync_led_start(void)
{
    if (AP_PIN_SYNC_LED < 0) return ESP_OK;
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << AP_PIN_SYNC_LED,
        .mode = GPIO_MODE_OUTPUT,
    };
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) return err;
    gpio_set_level(AP_PIN_SYNC_LED, 0);
    BaseType_t ok = xTaskCreatePinnedToCore(sync_led_task, "sync_led", 2048, NULL,
                                            AP_PRIO_CONTROL, NULL, AP_CORE_CONTROL);
    ESP_LOGI(TAG, "sync LED on GPIO%d", AP_PIN_SYNC_LED);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}
