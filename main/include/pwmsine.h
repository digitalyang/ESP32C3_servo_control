/***
 * @file pwmsine.h
 * @author digitalSheep (wcyang_int@safhez.com)
 * @brief Servo controller header file
 * @version 0.1
 * @platform Espressif ESP32
 * @date 2025-08-05
*/
#ifndef PWMSINE_H
#define PWMSINE_H

#include <stdint.h>

#define TABLE_SIZE 50  // 50个数据点
// PWM sine table
const uint16_t sine_table[TABLE_SIZE] = {1,2,5,8,11,16,21,26,32,38,44,50,56,62,68,74,80,89,92,95,98,100,100,98,95,92,89,80,74,68,62,56,50,44,38,32,26,21,16,11,8,5,2,1};

// Servo controller function prototypes
// 根据初始角度angle, 步长step, 逐渐增加到angle + angle_step, 并设置相应的占空比
void set_servo_angle_sine_boost(float angle, int angle_step, ledc_channel_t servo_channel, int delay_ms_temp) 
{
    // 限制角度范围：0.0 ~ 180.0
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;

    // 将角度转换为脉宽（单位：微秒，占空比2.5%-12.5%，范围 62.5us ~ 312.5us）
    float pulse_width_us = 90.3f + (angle * 312.5f / 160.0f);  // 从20度开始控制，以防0度识别不到
    printf("pulse_width_us = %f\n", pulse_width_us);

    // duty = pulse_width / period * 2^分辨率
    // 你当前设置的定时器周期 PWM_PERIOD_US（单位：us）
    double duty_cycle = (double)(pulse_width_us * 2048.0f) / PWM_PERIOD_US;  // 2048 = 2^11（即 11 位分辨率）
    
    for (int i = 0; i < (TABLE_SIZE - 1); i++) 
    {
        // 设置占空比
        ledc_set_duty(LEDC_SPEED_MODE, servo_channel, (double)(((90.3f * 2048.0f) / PWM_PERIOD_US) + ((duty_cycle - ((90.3f * 2048.0f) / PWM_PERIOD_US)) * sine_table[i] / 100)));  // 占空比逐渐增加到目标角度
        printf("duty = %f + %f\n", (90.3f * 2048.0f) / PWM_PERIOD_US,  ((duty_cycle - ((90.3f * 2048.0f) / PWM_PERIOD_US)) * sine_table[i] / 100));
        ledc_update_duty(LEDC_SPEED_MODE, servo_channel);

        // 延时
        vTaskDelay(pdMS_TO_TICKS(delay_ms_temp/20)); 
        printf("delay_us = %d\n", delay_ms_temp/20);
    }
}
#endif // PWMSINE_H
