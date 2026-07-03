/*
 * mg996r.c
 *
 *  Created on: 2026年5月13日
 *      Author: yun
 */

 /**
 * @file mg996r.c
 * @brief MG996R 舵机控制驱动实现
 */

#include "mg996r.h"
#include "tim.h"
#include <math.h>
#include <stdbool.h>

/* ==================== 静态变量 ==================== */
static float g_current_angle = 90.0f;          // 当前角度
static float g_calib_min = MG996R_MIN_ANGLE;   // 校准最小值
static float g_calib_max = MG996R_MAX_ANGLE;   // 校准最大值
static bool g_initialized = false;             // 初始化标志

/* ==================== 静态函数 ==================== */
/**
 * @brief 将角度转换为 CCR 值
 * @param angle: 角度值 (度)
 * @return CCR 值
 */
static uint32_t MG996R_AngleToCCR(float angle)
{
    // 限制角度范围
    if (angle < g_calib_min) angle = g_calib_min;
    if (angle > g_calib_max) angle = g_calib_max;
    
    // 线性插值计算 CCR 值
    // 公式：CCR = CCR_MIN + (angle / 180) × (CCR_MAX - CCR_MIN)
    float ratio = (angle - g_calib_min) / (g_calib_max - g_calib_min);
    uint32_t ccr = MG996R_CCR_MIN + 
                   (uint32_t)(ratio * (MG996R_CCR_MAX - MG996R_CCR_MIN));
    
    // 限制 CCR 范围
    if (ccr < MG996R_CCR_MIN) ccr = MG996R_CCR_MIN;
    if (ccr > MG996R_CCR_MAX) ccr = MG996R_CCR_MAX;
    
    return ccr;
}

/**
 * @brief 将 CCR 值转换为角度
 * @param ccr: CCR 值
 * @return 角度值 (度)
 */
static float MG996R_CCRToAngle(uint32_t ccr)
{
    // 限制 CCR 范围
    if (ccr < MG996R_CCR_MIN) ccr = MG996R_CCR_MIN;
    if (ccr > MG996R_CCR_MAX) ccr = MG996R_CCR_MAX;
    
    // 反向计算角度
    // 公式：angle = (CCR - CCR_MIN) / (CCR_MAX - CCR_MIN) × 180
    float ratio = (float)(ccr - MG996R_CCR_MIN) / 
                  (float)(MG996R_CCR_MAX - MG996R_CCR_MIN);
    float angle = g_calib_min + ratio * (g_calib_max - g_calib_min);
    
    return angle;
}

/* ==================== 公共函数 ==================== */

/**
 * @brief 初始化 MG996R 舵机
 * @retval HAL 状态
 */
HAL_StatusTypeDef MG996R_Init(void)
{
    if (g_initialized) {
        return HAL_OK;
    }
    
    // 1. 启动 TIM3 PWM Channel 1
    HAL_StatusTypeDef ret = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    if (ret != HAL_OK) {
        return ret;
    }
    
    // 2. 设置初始位置为中位
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, MG996R_CCR_CENTER);

    // 3. 更新状态
    g_current_angle = MG996R_CENTER_ANGLE;
    g_initialized = true;
    
    // 4. 等待舵机到达中位
    HAL_Delay(500);
    
    return HAL_OK;
}

/**
 * @brief 设置舵机角度
 * @param angle: 目标角度 (0° ~ 180°)
 */
void MG996R_SetAngle(float angle)
{
    if (!g_initialized) {
        return;
    }
    
    // 限制角度范围
    if (angle < g_calib_min) angle = g_calib_min;
    if (angle > g_calib_max) angle = g_calib_max;
    
    // 计算并设置 CCR 值
    uint32_t ccr = MG996R_AngleToCCR(angle);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ccr);
    
    // 更新当前角度
    g_current_angle = angle;
}

/**
 * @brief 获取当前舵机角度
 * @return 当前角度 (度)
 */
float MG996R_GetAngle(void)
{
    if (!g_initialized) {
        return g_current_angle;
    }
    
    // 从硬件读取当前 CCR 值
    uint32_t ccr = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);
    
    // 转换为角度
    g_current_angle = MG996R_CCRToAngle(ccr);
    
    return g_current_angle;
}

/**
 * @brief 平滑运动到目标角度（指定时间）
 * @param target_angle: 目标角度
 * @param duration_ms: 运动时间（毫秒）
 */
void MG996R_MoveTo(float target_angle, uint32_t duration_ms)
{
    if (!g_initialized) {
        return;
    }
    
    // 限制目标角度
    if (target_angle < g_calib_min) target_angle = g_calib_min;
    if (target_angle > g_calib_max) target_angle = g_calib_max;
    
    float start_angle = g_current_angle;
    float angle_diff = target_angle - start_angle;
    
    // 如果角度差很小，直接设置
    if (fabsf(angle_diff) < 0.5f) {
        MG996R_SetAngle(target_angle);
        return;
    }
    
    // 计算步数和步长
    // 舵机控制周期 20ms，每 20ms 更新一次
    uint32_t steps = duration_ms / 20;
    if (steps == 0) steps = 1;
    
    float step_angle = angle_diff / steps;
    
    // 逐步移动
    for (uint32_t i = 0; i < steps; i++) {
        float current = start_angle + step_angle * (i + 1);
        MG996R_SetAngle(current);
        HAL_Delay(20);
    }
    
    // 确保到达目标位置
    MG996R_SetAngle(target_angle);
}

/**
 * @brief 平滑运动到目标角度（指定速度）
 * @param target_angle: 目标角度
 * @param speed_dps: 速度（度/秒）
 */
void MG996R_MoveToSpeed(float target_angle, float speed_dps)
{
    if (!g_initialized) {
        return;
    }
    
    // 限制目标角度
    if (target_angle < g_calib_min) target_angle = g_calib_min;
    if (target_angle > g_calib_max) target_angle = g_calib_max;
    
    float angle_diff = fabsf(target_angle - g_current_angle);
    
    // 计算所需时间（毫秒）
    // 时间 = 角度差 / 速度 × 1000
    uint32_t duration_ms = (uint32_t)(angle_diff / speed_dps * 1000.0f);
    
    // 最小时间限制
    if (duration_ms < 100) duration_ms = 100;
    
    // 执行平滑运动
    MG996R_MoveTo(target_angle, duration_ms);
}

/**
 * @brief 回到中位
 */
void MG996R_Center(void)
{
    MG996R_MoveTo(MG996R_CENTER_ANGLE, 500);
}

/**
 * @brief 回到零位
 */
void MG996R_Zero(void)
{
    MG996R_MoveTo(g_calib_min, 1000);
}

/**
 * @brief 禁用舵机输出（停止 PWM）
 */
void MG996R_Disable(void)
{
    if (!g_initialized) {
        return;
    }
    
    // 停止 PWM 输出
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    g_initialized = false;
}

/**
 * @brief 设置校准参数
 * @param min_angle: 实际最小角度
 * @param max_angle: 实际最大角度
 */
void MG996R_SetCalibration(float min_angle, float max_angle)
{
    // 限制范围
    if (min_angle < 0) min_angle = 0;
    if (max_angle > 180) max_angle = 180;
    if (min_angle >= max_angle) return;
    
    g_calib_min = min_angle;
    g_calib_max = max_angle;
}

/**
 * @brief 获取校准参数
 * @param min_angle: 返回最小角度
 * @param max_angle: 返回最大角度
 */
void MG996R_GetCalibration(float *min_angle, float *max_angle)
{
    if (min_angle != NULL) {
        *min_angle = g_calib_min;
    }
    if (max_angle != NULL) {
        *max_angle = g_calib_max;
    }
}
