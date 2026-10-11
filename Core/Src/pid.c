/*
 * pid.c
 *
 *  Created on: May 15, 2026
 *      Author: ebov
 */

#include "main.h"
#include "as5600.h"
#include <math.h>

extern TIM_HandleTypeDef htim1;
extern AS5600_TypeDef *as5600;

#define PWM_MAX 10799
#define PWM_MIN 2500

// PI katsayıları
float Kp = 50.0f;
float Ki = 0.02f;
float Kd = 0.0f;

// Komut paketi
typedef struct
{
    float target_final_angle;   // Son duracağı açı: 0-360
    int turn_count;             // Kaç tam tur atsın
    float speed_deg_per_sec;    // Hız: derece/saniye
    uint8_t start_motion;       // 1 yapınca hareket başlasın
} Motor_Command_t;

volatile Motor_Command_t motor_cmd = {
    .target_final_angle = 180.0f,
    .turn_count = 0,
    .speed_deg_per_sec = 180.0f,
    .start_motion = 0
};

// Debug paketi
typedef struct
{
    float error;
    float integral;
    float output;
    float pwm_value;
    uint16_t raw_angle;
} PID_Debug_t;

volatile PID_Debug_t pid_debug;

// Encoder / hareket değişkenleri
uint16_t raw_angle = 0;
float current_angle = 0.0f;
float last_angle = 0.0f;

int32_t revolution_counter = 0;

float total_current_angle = 0.0f;
float total_target_angle = 0.0f;
float total_command_angle = 0.0f;

uint8_t motion_active = 0;

// PI değişkenleri
float error = 0.0f;
float integral = 0.0f;
float output = 0.0f;

float dt = 0.01f;

float Angle_Difference(float target, float current)
{
    float diff = target - current;

    if (diff > 180.0f)
        diff -= 360.0f;

    if (diff < -180.0f)
        diff += 360.0f;

    return diff;
}

float Normalize_Angle(float angle)
{
    while (angle >= 360.0f)
        angle -= 360.0f;

    while (angle < 0.0f)
        angle += 360.0f;

    return angle;
}

float AS5600_ReadAngleDegree(void)
{
    AS5600_GetAngle(as5600, &raw_angle);
    return (raw_angle * 360.0f) / 4096.0f;
}

void Motor_Stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

    pid_debug.pwm_value = 0;
}

void Motor_SetPWM(float pwm)
{
    if (pwm > PWM_MAX)
        pwm = PWM_MAX;

    if (pwm < -PWM_MAX)
        pwm = -PWM_MAX;

    if (pwm > 0 && pwm < PWM_MIN)
        pwm = PWM_MIN;

    if (pwm < 0 && pwm > -PWM_MIN)
        pwm = -PWM_MIN;

    pid_debug.pwm_value = pwm;

    if (pwm > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)pwm);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    }
    else if (pwm < 0)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint16_t)(-pwm));
    }
    else
    {
        Motor_Stop();
    }
}

void Update_Total_Angle(void)
{
    current_angle = AS5600_ReadAngleDegree();

    float diff = current_angle - last_angle;

    if (diff > 180.0f)
        revolution_counter--;

    if (diff < -180.0f)
        revolution_counter++;

    total_current_angle = (revolution_counter * 360.0f) + current_angle;

    last_angle = current_angle;
}

void Start_Motion_Command(void)
{
    float start_total_angle = total_current_angle;
    float final_angle = Normalize_Angle(motor_cmd.target_final_angle);

    total_command_angle = start_total_angle
                        + (motor_cmd.turn_count * 360.0f)
                        + Angle_Difference(final_angle, current_angle);

    total_target_angle = start_total_angle;

    integral = 0.0f;
    error = 0.0f;
    output = 0.0f;

    motion_active = 1;
    motor_cmd.start_motion = 0;
}

void Update_Total_Target_With_Speed(void)
{
    float diff = total_command_angle - total_target_angle;
    float step = motor_cmd.speed_deg_per_sec * dt;

    if (step < 0.1f)
        step = 0.1f;

    if (fabs(diff) <= step)
    {
        total_target_angle = total_command_angle;
    }
    else
    {
        if (diff > 0)
            total_target_angle += step;
        else
            total_target_angle -= step;
    }
}

void PID_PositionControl(void)
{
    Update_Total_Angle();

    if (motor_cmd.start_motion == 1)
    {
        Start_Motion_Command();
    }

    if (motion_active == 0)
    {
        Motor_Stop();
        return;
    }

    Update_Total_Target_With_Speed();

    error = total_target_angle - total_current_angle;

    integral += error * dt;

    output = (Kp * error) + (Ki * integral);

    pid_debug.error = error;
    pid_debug.integral = integral;
    pid_debug.output = output;
    pid_debug.raw_angle = raw_angle;

    if (fabs(total_command_angle - total_current_angle) < 1.0f)
    {
        integral = 0.0f;
        motion_active = 0;
        Motor_Stop();
    }
    else
    {
        Motor_SetPWM(output);
    }
}
