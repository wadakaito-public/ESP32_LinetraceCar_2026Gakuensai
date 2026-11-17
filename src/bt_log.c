#include <stdarg.h>
#include <stdatomic.h>
#include <stdio.h>
#include <unistd.h>

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs_flash.h"

typedef struct {
    char text[120];
} log_message_t;

static QueueHandle_t log_queue;
static atomic_int client_fd = -1;

static void spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    static uint32_t client_handle;

    switch (event) {
    case ESP_SPP_INIT_EVT:
        if (param->init.status == ESP_SPP_SUCCESS) {
            esp_spp_vfs_register();
        }
        break;

    case ESP_SPP_VFS_REGISTER_EVT:
        if (param->vfs_register.status == ESP_SPP_SUCCESS) {
            esp_spp_start_srv(ESP_SPP_SEC_AUTHENTICATE,
                              ESP_SPP_ROLE_SLAVE, 0, "DriveLog");
        }
        break;

    case ESP_SPP_START_EVT:
        if (param->start.status == ESP_SPP_SUCCESS) {
            esp_bt_gap_set_device_name("LinetraceLog");
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE,
                                     ESP_BT_GENERAL_DISCOVERABLE);
        }
        break;

    case ESP_SPP_SRV_OPEN_EVT:
        if (param->srv_open.status == ESP_SPP_SUCCESS) {
            client_handle = param->srv_open.handle;
            atomic_store(&client_fd, param->srv_open.fd);
        }
        break;

    case ESP_SPP_CLOSE_EVT:
        if (param->close.handle == client_handle) {
            atomic_store(&client_fd, -1);
        }
        break;

    default:
        break;
    }
}

static void log_sender_task(void *arg)
{
    (void)arg;
    log_message_t message;

    for (;;) {
        if (xQueueReceive(log_queue, &message, portMAX_DELAY) == pdTRUE) {
            int fd = atomic_load(&client_fd);
            if (fd >= 0) {
                int length = snprintf(NULL, 0, "%s", message.text);
                write(fd, message.text, length);
            }
        }
    }
}

void bt_log_start(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    esp_bt_controller_config_t controller = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&controller));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    log_queue = xQueueCreate(16, sizeof(log_message_t));
    configASSERT(log_queue != NULL);

    BaseType_t result = xTaskCreate(log_sender_task, "bt_log", 3072, NULL, 2, NULL);
    configASSERT(result == pdPASS);

    ESP_ERROR_CHECK(esp_spp_register_callback(spp_callback));

    const esp_spp_cfg_t spp = BT_SPP_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(esp_spp_enhanced_init(&spp));
}

void bt_log(const char *format, ...)
{
    if (log_queue == NULL || atomic_load(&client_fd) < 0) {
        return;
    }

    log_message_t message = {0};
    va_list args;

    va_start(args, format);
    vsnprintf(message.text, sizeof(message.text), format, args);
    va_end(args);

    xQueueSend(log_queue, &message, 0);
}