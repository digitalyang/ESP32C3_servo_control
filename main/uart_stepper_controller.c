/***
 * @file uart_stepper_controller.c
 * @brief uart stepper controller for esp32
 * @version 0.1
 * @platform Espressif ESP32
 * @date 2025-10-28
 * @todo 
*/

#include <stdio.h>
#include "include/uart_ctrl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "sdkconfig.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include <string.h>
#include "driver/gptimer.h"
#include "esp_err.h"
#include "math.h"

#define COMMAND_INTERVAL_MS 22 // 指令间隔 25ms
#define SINE_TABLE_SIZE (1000 / COMMAND_INTERVAL_MS)  // 对应采样点


static uint8_t sine_speed_table[SINE_TABLE_SIZE];

sine_speed_config_t g_cfg = 
{
    .sample_ms    = 1000,    // 1000ms 采样周期
    .frequency_hz = 1.0f, // 1Hz 频率
    .amplitude    = 250.0f,  // 10.0 振幅
    .offset       = 0.0f,       // 偏移量
};

// The handle of the timer used to generate the sine wave
static gptimer_handle_t g_timer = NULL;

// The current time in seconds
static float g_time_s = 0.0f;

// Flag of motor direction
static bool g_dir = true;

// Global queue for uart events
static QueueHandle_t stepper_cmd_queue = NULL;

stepper_cmd_t cmd_unidirectional = {
    .header = STEPPER_CMD_HEADER,
    .mode   = UNIDIRECTIONAL_MODE,
    .speed  = 0x60,
    .force  = 0x20,
    .time   = 0x20,
    .dir    = 0x01,
    .reserved = {0},   // 全部置 0
};

stepper_cmd_t cmd_stop = {
    .header = STEPPER_CMD_HEADER,
    .mode   = STOP_MODE,
    .speed  = 0x00,
    .force  = 0x00,
    .time   = 0x00,
    .dir    = 0x01,
    .reserved = {0},   // 全部置 0
};

stepper_cmd_t cmd_loop = {
    .header = STEPPER_CMD_HEADER,
    .mode   = LOOP_MODE,
    .speed  = 0x60,
    .force  = 0x20,
    .time   = 0x20,
    .dir    = 0x01,
    .reserved = {0},   // 全部置 0
};

stepper_cmd_t cmd_servo = {
    .header = STEPPER_CMD_HEADER,
    .mode   = SERVO_MODE,
    .speed  = 0x60,
    .force  = 0x20,
    .time   = 0x20,
    .dir    = 0x01,
    .reserved = {0},   // 全部置 0
};

stepper_cmd_t cmd_button_trigger = {
    .header = STEPPER_CMD_HEADER,
    .mode   = BUTTON_TRIGGER_MODE,
    .speed  = 0xfa,
    .force  = 0x20,
    .time   = 0x50,
    .dir    = 0x01,
    .reserved = {0},   // 全部置 0
};

// cmd: Ff 05 50 20 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
stepper_cmd_t cmd_start_clockwise = {
    .header = STEPPER_CMD_HEADER,
    .mode   = START_MODE_CW,
    .speed  = 0x60,
    .force  = 0x20,
    .time   = 0x20,
    .dir    = 0x01,
    .reserved = {0},   // 全部置 0
};

stepper_cmd_t cmd_uart_trigger = {
    .header = STEPPER_CMD_HEADER,
    .mode   = UART_TRIGGER_MODE,
    .speed  = 0x60,
    .force  = 0x20,
    .time   = 0x20,
    .dir    = 0x01,
    .reserved = {0},   // 全部置 0
};

stepper_cmd_t cmd_start_counterclockwise = {
    .header = STEPPER_CMD_HEADER,
    .mode   = START_MODE_CW,
    .speed  = 0x60,
    .force  = 0x20,
    .time   = 0x20,
    .dir    = 0x00,
    .reserved = {0},   // 全部置 0
};


stepper_cmd_t *stepper_cmd_buf[] = {&cmd_unidirectional, &cmd_stop, &cmd_loop, &cmd_servo, &cmd_button_trigger, &cmd_start_clockwise, &cmd_start_counterclockwise, &cmd_uart_trigger};

static const char *TAG = "uart_comm";

static uart_rx_callback_t s_rx_callback = NULL;
static uart_port_t s_uart_num = UART_NUM_1;
static QueueHandle_t s_uart_event_queue = NULL;

uart_comm_config_t uart_config = {
   .uart_num = UART_NUM_1,
   .tx_pin = 4,
   .rx_pin = 3,
   .baud_rate = 115200,
};

uart_comm_config_t *config = &uart_config;

void uart_event_task(void *pvParameters)
{
    stepper_cmd_t *cmd_ptr;
    while (1) {
        if (xQueueReceive(stepper_cmd_queue, &cmd_ptr, portMAX_DELAY)) {
            uart_comm_send((const uint8_t *)cmd_ptr, UART_EVENT_QUEUE_SIZE);

            free(cmd_ptr);
        }
    }
}

void send_stepper_command_from_isr(stepper_cmd_t *cmd)
{
    stepper_cmd_t *cmd_ptr = (stepper_cmd_t *)malloc(sizeof(stepper_cmd_t));
    if (!cmd_ptr) return;
    memcpy(cmd_ptr, cmd, sizeof(stepper_cmd_t));

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (xQueueSendFromISR(stepper_cmd_queue, &cmd_ptr, &xHigherPriorityTaskWoken) != pdPASS) {
        ESP_LOGE(TAG, "stepper_cmd_queue 发送失败");
        free(cmd_ptr);
    }

    // 如果发送后有高优先级任务被唤醒，则切换任务
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}
esp_err_t uart_comm_init()
{
    if (!config) return ESP_ERR_INVALID_ARG;

    s_uart_num = config->uart_num;

    if (!stepper_cmd_queue) {
        stepper_cmd_queue = xQueueCreate(128, sizeof(stepper_cmd_t *));
        if (!stepper_cmd_queue) {
            ESP_LOGE(TAG, "stepper_cmd_queue 创建失败");
            return ESP_FAIL;
        }
    }

    uart_config_t uart_config = {
        .baud_rate  = config->baud_rate,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_param_config(s_uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(s_uart_num, config->tx_pin, config->rx_pin,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_ERROR_CHECK(uart_driver_install(s_uart_num,
                                        UART_BUF_SIZE * 2,
                                        UART_BUF_SIZE * 2,
                                        UART_EVENT_QUEUE_SIZE,
                                        &s_uart_event_queue,
                                        0));

    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 10, NULL);

    ESP_LOGI(TAG, "UART%d initialized (TX=%d, RX=%d, %d bps)",
             s_uart_num, config->tx_pin, config->rx_pin, config->baud_rate);
    return ESP_OK;
}

esp_err_t uart_comm_register_rx_callback(uart_rx_callback_t callback)
{
    s_rx_callback = callback;
    return ESP_OK;
}

esp_err_t uart_comm_send(const uint8_t *data, size_t len)
{
    if (!data || len == 0) return ESP_ERR_INVALID_ARG;
    int w = uart_write_bytes(s_uart_num, (const char *)data, len);
    return (w == len) ? ESP_OK : ESP_FAIL;
}

esp_err_t uart_comm_send_string(const char *str, bool add_newline)
{
    if (!str) return ESP_ERR_INVALID_ARG;

    if (add_newline) {
        char buf[128];
        snprintf(buf, sizeof(buf), "%s\r\n", str);
        return uart_comm_send((const uint8_t *)buf, strlen(buf));
    } else {
        return uart_comm_send((const uint8_t *)str, strlen(str));
    }
}

void generate_sine_speed_table()
{
    for (int i = 0; i < SINE_TABLE_SIZE; i++) {
        float t = (float)i / SINE_TABLE_SIZE; // 0~1
        float radians = M_PI * t;     // 0~π
        float value = g_cfg.offset + g_cfg.amplitude * sinf(radians);

        if (value < 1.0f) value = 1.0f;
        if (value > 250.0f) value = 250.0f;

        sine_speed_table[i] = (uint8_t)value;
    }
}


static bool IRAM_ATTR gptimer_callback(gptimer_handle_t timer,
                                       const gptimer_alarm_event_data_t *edata,
                                       void *user_ctx)
{
    timer_ctx_t *ctx = (timer_ctx_t *)user_ctx;
    ctx->tick_1ms++;

    // 使用查找表
    static uint16_t index = 0;
    uint8_t speed = sine_speed_table[index];
    index = (index + 1) % SINE_TABLE_SIZE;

    // 每 25 次（1s）切换方向
    if (ctx->tick_1ms % SINE_TABLE_SIZE == 0) g_dir = !g_dir;

    stepper_cmd_t cmd = {
        .header = STEPPER_CMD_HEADER,
        .mode   = UART_TRIGGER_MODE,
        .speed  = speed,
        .force  = 0x60,
        .time   = 0x05,
        .dir    = g_dir,
        .reserved = {0},
    };

    send_stepper_command_from_isr(&cmd);  // ISR安全的发送函数

    return false;
}

void sine_speed_init()
{
    
    g_time_s = 0.0f;
    static timer_ctx_t ctx = {0};

    gptimer_config_t timer_config = 
    {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1 MHz, 即 1 tick = 1 µs
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &g_timer));

    // 每个1ms, 更新一次速度
    gptimer_alarm_config_t alarm_config = 
    {
        .reload_count = 0,      // 当警报事件发生时，定时器会自动重载到 0
        .alarm_count = COMMAND_INTERVAL_MS*1000, // 设置实际的警报周期，因为分辨率是 1us，所以 1000 代表 1ms
        .flags.auto_reload_on_alarm = true, // 使能自动重载功能
    };

    // 注册回调
    gptimer_event_callbacks_t cbs = 
    {
        .on_alarm = gptimer_callback,
    };

    // 设置定时器的警报动作
    gptimer_set_alarm_action(g_timer, &alarm_config);

    // 注册定时器事件回调函数，允许携带用户上下文
    gptimer_register_event_callbacks(g_timer, &cbs, &ctx);
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(g_timer, &cbs, &ctx));

    printf("sine_speed_init done\n");
}

void timer_start()
{
    ESP_ERROR_CHECK(gptimer_enable(g_timer));
    ESP_ERROR_CHECK(gptimer_start(g_timer));
}

void timer_stop()
{
    if (g_timer == NULL) {
        ESP_LOGW("TIMER", "g_timer is NULL, skip stop");
        return;
    }

    esp_err_t err = gptimer_stop(g_timer);
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGW("TIMER", "Timer already stopped");
        return;
    } else {
        ESP_ERROR_CHECK(err);
    }

    ESP_ERROR_CHECK(gptimer_disable(g_timer));
}



// // Slower than the speed of the stepper motor itself.
// void stepper_motor_move_slow(void)
// {
//     uart_comm_send((uint8_t*) stepper_cmd_buf[STOP_MODE], STEPPER_CMD_LEN);
//     vTaskDelay(pdMS_TO_TICKS(10));
// }