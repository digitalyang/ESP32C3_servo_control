/***
 * @file servo_controller.c
 * @author digitalSheep (wcyang_int@safhez.com)
 * @brief servo controller for esp32
 * @version 0.1
 * @platform Espressif ESP32
 * @date 2025-08-05
 * @todo 
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "include/servo_controller.h"
#include "sdkconfig.h"
#include "include/pwmsine.h"

static const int servo_channel[4] = {LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3};

volatile bool exitFlag = false;

// typedef struct {
//     ledc_channel_t channel;
//     uint32_t target_duty;
//     uint32_t delay_ms;
//     bool fade_up;
// } servo_fade_t;

// static bool fade_up = true;
// static servo_fade_t servo_fade_infos[SERVO_ALL];
// bool IRAM_ATTR ledc_fade_done_cb(const ledc_cb_param_t *param, void *user_arg)
// {
//     printf("ledc_fade_done_cb\n");
//     servo_fade_t *fade = (servo_fade_t *)user_arg;
//     if (param->event == LEDC_FADE_END_EVT)
//     {
//         fade->fade_up = !fade->fade_up;
//         uint32_t target = fade->fade_up ? fade->target_duty : (int)((90.3f * 2048.0f) / PWM_PERIOD_US);
//         ledc_set_fade_with_time(LEDC_SPEED_MODE, fade->channel, target, fade->delay_ms);
//         ledc_fade_start(LEDC_SPEED_MODE, fade->channel, LEDC_FADE_NO_WAIT);

//     }
//     return false;
// }

// void servo_fade_start(int delay_ms, ledc_channel_t servo_channel, int angle_duty)
// {
//     printf("servo_fade_start\n");
//     float pulse_width_us = 90.3f + (angle_duty * 312.5f / 160.0f);  // 从20度开始控制，以防0度识别不到

//     // duty = pulse_width / period * 2^分辨率
//     // 你当前设置的定时器周期 PWM_PERIOD_US（单位：us）
//     int duty_cycle = (int)((pulse_width_us * 2048.0f) / PWM_PERIOD_US);  // 2048 = 2^11（即 11 位分辨率）

//     servo_fade_t *servo_info = malloc(sizeof(servo_fade_t));
//     servo_fade_infos[0].target_duty = duty_cycle;
//     servo_fade_infos[0].delay_ms = delay_ms;
//     servo_fade_infos[0].fade_up = false;
//     printf("servo_channel = %d,target_duty = %ld,delay_ms = %ld,fade_up = %d\n", servo_info->channel, servo_info->target_duty, servo_info->delay_ms, servo_info->fade_up);
//     ledc_cbs_t callbacks =  
//     {
//         .fade_cb = ledc_fade_done_cb
//     };

//     ledc_cb_register(LEDC_SPEED_MODE, servo_channel, &callbacks, servo_info);

//     ledc_set_fade_with_time(LEDC_SPEED_MODE, servo_channel, angle_duty, delay_ms);
//     ledc_fade_start(LEDC_SPEED_MODE, servo_channel, LEDC_FADE_NO_WAIT);
// }

// // 初始化 servo fade
// void servo_fade_init()
// {
//     // 安装 fade 函数，只执行一次
//     ledc_fade_func_install(0);

//     // 注册每个舵机 fade 回调，只注册一次
//     ledc_cbs_t callbacks = {
//         .fade_cb = ledc_fade_done_cb
//     };
//     for (int i = 0; i < SERVO_ALL; i++)
//     {
//         servo_fade_infos[i].channel = i;       // LEDC channel
//         servo_fade_infos[i].fade_up = false;   // 初始状态
//         ledc_cb_register(LEDC_SPEED_MODE, i, &callbacks, &servo_fade_infos[i]);
//     }
// }

// initialize servo channel
void servo_setup(ledc_channel_t servo_channel, gpio_num_t servo_pin) 
{
    ledc_channel_config_t ledc_channel = 
    {
        .gpio_num = servo_pin,
        .speed_mode = LEDC_SPEED_MODE,
        .channel = servo_channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

// set servo angle(0~180 degrees)
void set_servo_angle(float angle, ledc_channel_t servo_channel) 
{
    // 限制角度范围：0.0 ~ 180.0
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;

    // 将角度转换为脉宽（单位：微秒，占空比2.5%-12.5%，范围 62.5us ~ 312.5us）
    double pulse_width_us = 600 + (angle * 2000 / 160.0f);  // 从20度开始控制，以防0度识别不到
    printf("pulse_width_us = %f\n", pulse_width_us);
    // duty = pulse_width / period * 2^分辨率
    // 你当前设置的定时器周期 PWM_PERIOD_US（单位：us）
    double duty_cycle = (double)((pulse_width_us * 2048.0f) / PWM_PERIOD_US);  // 2048 = 2^11（即 11 位分辨率）

    // 设置占空比
    ledc_set_duty(LEDC_SPEED_MODE, servo_channel, duty_cycle);
    ledc_update_duty(LEDC_SPEED_MODE, servo_channel);
}




// Single servo control tasks
void control_servo_motion(void *pvParameters)
{
    
    motion_stage_t *stage = (motion_stage_t *)pvParameters;
    printf("Starting motion for servo_channel %d\n", stage->servo_channel);

    int speed = (stage->speed == 0) ? ((stage->mode == SERVO_HIGH_SPEED_MODE) ? swing_speed_high : 
                ((stage->mode == SERVO_VIBRATE_MODE) ? vibrate_speed : swing_speed_low)) : stage->speed;
    int range = (stage->range == 0) ? ((stage->mode == SERVO_VIBRATE_MODE) ? vibrate_swing_range : swing_range) : stage->range;
    int delay_ms_temp = (stage->delay_ms <= 15) ? ((stage->mode == SERVO_HIGH_SPEED_MODE) ? swing_delay_high : ((stage->mode == SERVO_LOW_SPEED_MODE) ? swing_delay_low : swing_delay_high)) : stage->delay_ms;
    int repeat_time_temp = (stage->repeat_time == 0) ? 0 : stage->repeat_time;
    if (repeat_time_temp == 0)
    {
        while (1)
        {                
            if (exitFlag)
            {
                // Exiting task
                printf("Exiting task...\n");
                vTaskDelete(NULL);
            }

            for (int angle = 0; angle <= range; angle += speed) 
            {
                set_servo_angle(angle, stage->servo_channel);
                vTaskDelay(pdMS_TO_TICKS(delay_ms_temp / 2));
                printf("delay_ms_temp = %d,angle = %d,speed = %d\n", delay_ms_temp, angle, speed);
            }

            for (int angle = range; angle >= 0; angle -= speed) 
            {
                set_servo_angle(angle, stage->servo_channel);
                vTaskDelay(pdMS_TO_TICKS(delay_ms_temp / 2));
            }   

            // set_servo_angle_sine_boost(range,speed,stage->servo_channel,delay_ms_temp);
            // servo_fade_start(delay_ms_temp, stage->servo_channel, range);
                   
            vTaskDelay(1);  // 喂狗
        }
    }
    else
    {
        for (int i = 0; i < repeat_time_temp; i++)
        {
            // if (exitFlag)
            // {
            //     // Exiting task
            //     printf("Exiting task...\n");
            //     vTaskDelete(NULL);
            // }

            // for (int angle = 0; angle <= range; angle += speed) 
            // {
            //     set_servo_angle(angle, stage->servo_channel);
            //     vTaskDelay(pdMS_TO_TICKS(delay_ms_temp / 2));
            // }

            // for (int angle = range; angle >= 0; angle -= speed) 
            // {
            //     set_servo_angle(angle, stage->servo_channel);
            //     vTaskDelay(pdMS_TO_TICKS(delay_ms_temp / 2));
            // }
            //set_servo_angle_sine_boost(range,speed,stage->servo_channel,delay_ms_temp);
            //servo_fade_start(delay_ms_temp, stage->servo_channel, range);
            
        }    

        xSemaphoreGive(stage->semaphore);
        vTaskDelete(NULL);
        vTaskDelay(pdMS_TO_TICKS(1000)); // 喂狗
    }


   
}

// create servo task
void create_servo_task(motion_stage_t *stage)
{
    if (xTaskCreate(control_servo_motion, "control_servo_motion", 4096, stage, 5, NULL) != pdPASS)
    {
        printf("Failed to create task for servo_channel %d\r\n", stage->servo_channel);
    }
}

void servo_init() // Initialize all servos
{
    // Initialize the LEDC timer
    ledc_timer_config_t ledc_timer = 
    {
        .speed_mode = LEDC_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_11_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Initialize the servo
    for (int i = 0; i < SERVO_ALL; i++) servo_setup(servo_channel[i], servo_pin[i]);
    for (int i = 0; i < SERVO_ALL; i++) set_servo_angle(0, servo_channel[i]);

    // servo_fade_init();
}

// Generate motion object for a single servo
motion_stage_t get_range_step_motion(int delay_ms, int range, int step)
{
    motion_stage_t object = {
        .mode = SERVO_LOW_SPEED_MODE, 
        .range = range, 
        .speed = step, 
        .delay_ms = delay_ms
    };

    return object;
}

SemaphoreHandle_t servo_motion(servo_e servo_num, int delay_ms, int range, int step, uint8_t block_inline)
{
    motion_stage_t stages[1] = {0};
    stages[0] = get_range_step_motion(delay_ms, range, step);

    return servo_motion_programmable(servo_num, stages, 1, block_inline);
}

void servo_motion_all_simple(int delay_ms)
{
    servo_motion_all(delay_ms, swing_range, swing_speed_high);
}

void servo_motion_switch(int servo_switch) 
{
    if(servo_switch == 1) exitFlag = false;
    else exitFlag = true;                    
    
}

void servo_motion_all(int delay_ms, int range, int step)
{
    motion_stage_t motions[(int)SERVO_ALL][1];
    int stages_count[(int)SERVO_ALL];

    for (int i = 0; i < (int)SERVO_ALL; i++)
    {
        // Generate motions object for servos
        motions[i][0] = get_range_step_motion(delay_ms, range, step);
        motions[i][0].servo_channel = servo_channel[i];
        stages_count[i] = 1;
    }

    servo_motion_all_programmable((motion_stage_t**)motions, stages_count);
}

SemaphoreHandle_t servo_motion_programmable(servo_e servo_num, motion_stage_t* stages, int stage_count, uint8_t block_inline)
{
    SemaphoreHandle_t semaphore = xSemaphoreCreateCounting(1, 0);
    if (semaphore == NULL) {
        printf("Failed to create semaphore\n");
        return NULL;
    }
    // Generate stages object for the servo
    for (int i = 0; i < stage_count; i++)
    {
        stages[i].semaphore = semaphore;
        stages[i].servo_channel = servo_channel[(int)servo_num];
    }
    // Generate sequence object for the servo
    motion_sequence_t seq = {
        .stages = stages,
        .stage_count = stage_count
    };
    // Start all stages tasks
    for (int i = 0; i < seq.stage_count; i++)
    {
        create_servo_task(&seq.stages[i]); 
        printf("Servo %d motion stage %d created.\n", servo_num, i);
    }

    // Block until all tasks are complete
    if (block_inline)
    {
        xSemaphoreTake(semaphore, portMAX_DELAY);
        // Debug information
        printf("All servo motions of servo %d complated.\n", servo_num);

        return semaphore;
    }

    printf("Servo motions of servo %d and stage %d started but not blocked.\n", servo_num, stage_count);
    return semaphore;
}

void servo_motion_all_programmable(motion_stage_t** stages, int* stages_count) 
{
    SemaphoreHandle_t semaphores[(int)SERVO_ALL] = {0};

    // Get each servo's stages and its count
    for (int i = 0; i < (int)SERVO_ALL; i++)
    {
        servo_e effect_servo = (servo_e)i;
        semaphores[i] = servo_motion_programmable(effect_servo, stages[i], stages_count[i], false);
    }

    //Block until all tasks are complete
    for (int i = 0; i < (int)SERVO_ALL; i++)
    {
        xSemaphoreTake(semaphores[i], portMAX_DELAY);
    }

    printf("All servo motions completed.\n");
}

void servo_motion_customized()
{
    int stage_count[(int)SERVO_ALL] = {1, 1, 1, 1};

    motion_stage_t stage_upper_left[1] = {
        {.mode = SERVO_HIGH_SPEED_MODE, .range = swing_range,         .speed = swing_speed_high, .delay_ms = swing_delay_high,      .repeat_time = 0   } 
    };

    motion_stage_t stage_upper_right[1] = {
        {.mode = SERVO_VIBRATE_MODE,    .range = vibrate_swing_range, .speed = vibrate_speed,    .delay_ms = swing_delay_high,       .repeat_time = 0 }
    };

    motion_stage_t stages_lower_left[1] = {
        {.mode = SERVO_HIGH_SPEED_MODE, .range = swing_range,         .speed = swing_speed_high, .delay_ms = swing_delay_high,        .repeat_time = 0 }  
    };

    motion_stage_t stages_lower_right[1] = {
        {.mode = SERVO_HIGH_SPEED_MODE, .range = swing_range,          .speed = swing_speed_high, .delay_ms = swing_delay_high        , .repeat_time = 0 } 
    };

    motion_stage_t* stages[4] = {stage_upper_left, stage_upper_right, stages_lower_left, stages_lower_right};
    
    servo_motion_all_programmable(stages, stage_count);
}