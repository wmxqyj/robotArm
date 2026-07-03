/*
 * mg996r.h
 *
 *  Created on: 2026年5月13日
 *      Author: yun
 */

 /**
 * @file mg996r.h
 * @brief MG996R 舵机控制驱动
 * 
 * 使用 TIM3 Channel 1 产生 PWM 信号
 * PWM 参数：周期 20ms (50Hz), 脉宽 0.5-2.5ms
 */

#ifndef __MG996R_H
#define __MG996R_H

#include "main.h"

/* ==================== 配置参数 ==================== */
// 舵机角度范围
#define MG996R_MIN_ANGLE        0.0f
#define MG996R_MAX_ANGLE        180.0f
#define MG996R_CENTER_ANGLE     90.0f

// PWM 参数（基于 84MHz 定时器时钟）
#define MG996R_PWM_PERIOD       20000       // ARR 值 (20ms)
#define MG996R_CCR_MIN          500         // 0.5ms → 0°
#define MG996R_CCR_MAX          1500        // 2.5ms → 180°
#define MG996R_CCR_CENTER       800        // 1.5ms → 90°

// 运动参数
#define MG996R_DEFAULT_SPEED    20.0f       // 默认速度 (度/秒)

/* ==================== 函数声明 ==================== */
// 初始化
HAL_StatusTypeDef MG996R_Init(void);

// 基本控制
void MG996R_SetAngle(float angle);
float MG996R_GetAngle(void);

// 平滑运动
void MG996R_MoveTo(float target_angle, uint32_t duration_ms);
void MG996R_MoveToSpeed(float target_angle, float speed_dps);

// 实用函数
void MG996R_Center(void);
void MG996R_Zero(void);
void MG996R_Disable(void);

// 校准功能
void MG996R_SetCalibration(float min_angle, float max_angle);
void MG996R_GetCalibration(float *min_angle, float *max_angle);

#endif /* __MG996R_H */
