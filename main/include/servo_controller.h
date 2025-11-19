/***
 * @file servo_controller.h
 * @author digitalSheep (wcyang_int@safhez.com)
 * @brief Servo controller header file
 * @version 0.1
 * @platform Espressif ESP32
 * @date 2025-08-05
*/
#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#define SERVO_1_PIN 7
#define SERVO_2_PIN 10
#define SERVO_3_PIN 1
#define SERVO_4_PIN 0

// PWM and motions parameters
#define PWM_PERIOD_US 20000  //servo pwm period 20ms
#define PWM_FREQ_HZ 50      //servo pwm frequency 50Hz
#define LEDC_SPEED_MODE LEDC_LOW_SPEED_MODE
#define swing_range 8
#define swing_speed_low 1 // minimum degree step may be 0.5 degree, but no test. because dead zone is 5us and 0.5 degree change with 0.55us PWM period change.
#define swing_speed_high 3
#define vibrate_speed 20
#define vibrate_swing_range 90
#define swing_delay_low 500  // default swing delay 500ms
#define swing_delay_high 15

static const int servo_pin[4] = {SERVO_1_PIN, SERVO_2_PIN, SERVO_3_PIN, SERVO_4_PIN};
// motion modes
typedef enum {
    SERVO_HIGH_SPEED_MODE,
    SERVO_LOW_SPEED_MODE,
    SERVO_VIBRATE_MODE
} speed_mode_t;

// motion stage
typedef struct {
    speed_mode_t mode;
    ledc_channel_t servo_channel;
    int repeat_time;
    int range;
    int speed;
    int delay_ms;
    SemaphoreHandle_t semaphore; // synchronization
} motion_stage_t;

// motion sequence
typedef struct {
    motion_stage_t *stages;
    int stage_count;
} motion_sequence_t;

typedef enum {
    SERVO_UPPER_RIGHT_CORNER = 0,
    SERVO_UPPER_LEFT_CORNER = 1,
    SERVO_LOWER_LEFT_CORNER = 2,
    SERVO_LOWER_RIGHT_CORNER = 3,
    SERVO_ALL = 4,
} servo_e;

typedef enum {
    SERVO_CTRL_OP_OFF = 0,
    SERVO_CTRL_OP_TEST = 1,
    SERVO_CTRL_OP_UPPER_RIGHT_CORNER = 2,
    SERVO_CTRL_OP_UPPER_LEFT_CORNER = 3,
    SERVO_CTRL_OP_LOWER_LEFT_CORNER = 4,
    SERVO_CTRL_OP_LOWER_RIGHT_CORNER = 5,
    SERVO_CTRL_OP_ALL = 6,
    SERVO_CTRL_OP_ALL_ADVANCED = 7
} servo_ctrl_op_e;

// function declarations
/*****
 * void servo_setup(ledc_channel_t servo_channel, gpio_num_t servo_pin)
 * @brief servo setup function
 * @param servo_channel: the ledc channel of the servo
 * @param servo_pin: the gpio pin of the servo
****/
void servo_setup(ledc_channel_t servo_channel, gpio_num_t servo_pin);

/*****
 * @brief set servo angle function
 * @param angle: the angle to be set
 * @param servo_channel: the ledc channel of the servo
 * @return void 
*/
void set_servo_angle(float angle, ledc_channel_t servo_channel);  

/*****
 * void control_servo_motion(void *pvParameters)
 * @brief servo motion control task function
 * @param pvParameters: the parameters of the task
 * @return void 
*/
void control_servo_motion(void *pvParameters);

/*****
 * void create_servo_task(motion_stage_t *stage)
 * @brief create servo motion task function
 * @param stage: the motion stage to be controlled
 * @return void 
*/
void create_servo_task(motion_stage_t *stage);

/*****
 * void servo_init()
 * @brief servo initialization function
 * @return void 
*/
void servo_init();

/*****
 * motion_stage_t get_range_step_motion(int delay_ms, int range, int step)
 * @brief get motion stage with range and step
 * @param delay_ms: the delay time between each step
 * @param range: the range of motion
 * @param step: the step of motion
 * @return motion_stage_t: the motion stage with range and step
*/
motion_stage_t get_range_step_motion(int delay_ms, int range, int step);

/*****
 * SemaphoreHandle_t servo_motion_programmable(servo_e servo_num, motion_stage_t* stages, int stage_count, uint8_t block_inline)
 * @brief servo motion programmable function
 * @param servo_num: the servo number to be controlled
 * @param stages: the motion stages to be controlled
 * @param stage_count: the number of motion stages
 * @param block_inline: whether to block the function until all tasks are complete
 * @return SemaphoreHandle_t: the semaphore handle of the motion task   
*/
SemaphoreHandle_t servo_motion_programmable(servo_e servo_num, motion_stage_t* stages, int stage_count, uint8_t block_inline);

/*****
 * void servo_motion(servo_e servo_num, int delay_ms, int range, int step, uint8_t block_inline)
 * @brief servo motion function
 * @param servo_num: the servo number to be controlled
 * @param delay_ms: the delay time between each step
 * @param range: the range of motion
 * @param step: the step of motion
 * @param block_inline: whether to block the function until all tasks are complete
 * @return SemaphoreHandle_t: the semaphore handle of the motion task
*/
SemaphoreHandle_t servo_motion(servo_e servo_num, int delay_ms, int range, int step, uint8_t block_inline);

/*****
 * void servo_motion_all(int delay_ms, int range, int step)
 * @brief servo motion all function
 * @param delay_ms: the delay time between each step
 * @param range: the range of motion
 * @param step: the step of motion
 * @return void 
*/
void servo_motion_all(int delay_ms, int range, int step);

/*****
 * void servo_motion_all_simple(int delay_ms)
 * @brief servo motion all simple function
 * @param delay_ms: the delay time between each step
 * @return void 
*/
void servo_motion_all_simple(int delay_ms);

/*****
 * void servo_motion_all_programmable(motion_stage_t** stages, int* stages_count)
 * @brief servo motion all programmable function
 * @param stages: the motion stages to be controlled
 * @param stages_count: the number of motion stages
 * @return void 
*/
void servo_motion_all_programmable(motion_stage_t** stages, int* stages_count);

/*****
 * void servo_motion_customized()
 * @brief servo motion customized function
 * @return void 
*/
void servo_motion_customized();

/*****
 * void servo_motion_switch(int servo_switch)
 * @brief servo motion switch function
 * @param servo_switch: the switch number to be controlled
 * @return void 
*/
void servo_motion_switch(int servo_switch);
#endif // SERVO_CONTROLLER_H
