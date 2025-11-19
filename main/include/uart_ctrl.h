
#ifndef UART_CTRL_H
#define UART_CTRL_H

#include "esp_err.h"
#include "driver/uart.h"

#define STEPPER_CMD_HEADER 0xff
#define LOOP_MODE 0x01
#define UNIDIRECTIONAL_MODE 0x02
#define SERVO_MODE 0x03
#define BUTTON_TRIGGER_MODE 0x04
#define START_MODE_CW 0x05
#define STOP_MODE 0x06
#define UART_TRIGGER_MODE 0x07

#define UART_BUF_SIZE 1024
#define UART_EVENT_QUEUE_SIZE 20

#define FREQ_HZ      0.5             // 正弦变化频率（0.5Hz = 2s周期）
#define OMEGA        (2 * M_PI * FREQ_HZ)
#define V_MIN        1
#define V_MAX        250

#define STEPPER_CMD_LEN sizeof(stepper_cmd_t)

#define M_PI 3.14159265358979323846

/** UART 配置结构体 */
typedef struct {
    uart_port_t uart_num;       ///< UART编号（UART_NUM_0/1/2）
    int tx_pin;
    int rx_pin;
    int baud_rate;
} uart_comm_config_t;

typedef struct {
    uint32_t tick_1ms;  // 1ms tick 计数器
    uint8_t  current_speed;
} timer_ctx_t;

// __attribute__((packed)) makes the struct tightly packed by bytes, without any extra padding.
typedef struct __attribute__((packed)){
    uint8_t header;
    uint8_t mode;
    uint8_t speed;
    uint8_t force;
    // Real time = time * 20ms
    uint8_t time;
    // 00: clockwise, 01: counterclockwise
    uint8_t dir;
    uint8_t reserved[14];
} stepper_cmd_t;

extern stepper_cmd_t cmd_loop;
extern stepper_cmd_t cmd_unidirectional;
extern stepper_cmd_t cmd_servo;
extern stepper_cmd_t cmd_button_trigger;
extern stepper_cmd_t cmd_start;
extern stepper_cmd_t cmd_stop;
extern stepper_cmd_t cmd_uart_trigger;

extern stepper_cmd_t *stepper_cmd_buf[];   

typedef struct {
    float amplitude;      // 振幅
    float offset;         // 偏移量
    float frequency_hz;   // 正弦波频率
    float sample_ms;      // 定时周期（ms）
} sine_speed_config_t;

/** UART 接收回调函数类型 */
typedef void (*uart_rx_callback_t)(const uint8_t *data, size_t len);

/** 初始化并启动 UART 通信任务 */
esp_err_t uart_comm_init();

/** 注册接收回调 */
esp_err_t uart_comm_register_rx_callback(uart_rx_callback_t callback);

/** 发送数据 */
esp_err_t uart_comm_send(const uint8_t *data, size_t len);

/** 发送字符串（自动加 \r\n 可选） */
esp_err_t uart_comm_send_string(const char *str, bool add_newline);

void timer_start();

void timer_stop();

void sine_speed_init();

void uart_event_task(void *pvParameters);

void generate_sine_speed_table();

#endif // UART_CTRL_H