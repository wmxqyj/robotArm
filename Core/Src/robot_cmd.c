/*
 * robot_cmd.c
 *
 *  Created on: 2026年4月16日
 *      Author: yun
 */

#include "robot.h"
#include "usart.h"
#include "string.h"
#include <stdio.h>
#include "robot_cmd.h"
#include "Emm_V5.h"

static int robot_abs_zero_handle(float *param);
static int robot_abs_restore_handle(float *param);
static int robot_soft_reset_handle(float *param);
static int robot_rel_rotate_handle(float *param);
static int robot_abs_rotate_handle(float *param);
static int robot_set_joints_handle(float *param);
static int robot_get_joints_handle(float *param);
static int robot_get_eepose_handle(float *param);
static int robot_get_ik_solution_handle(float *param);
static int robot_fk_handle(float *param);
static int robot_mg996r_set_angle_handle(float *param);
static int robot_mg996r_get_angle_handle(float *param);

static int robot_abs_zero_handle(float *param)
{
	(void)param;
	LOG("robot set absolute zero.\n");
	// int i = 4;
	// {
	for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++) {
		Emm_V5_Origin_Set_O(i + 1, true); // 存储设置
		vTaskDelay(10);
	}
	return pdPASS;
}

static int robot_abs_restore_handle(float *param)
{
	(void)param;
	LOG("robot restore absolute zero.\n");
	// int i = 4;
	// {
	for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++) {
		Emm_V5_Origin_Trigger_Return(i + 1, 0, false); // 单圈就近回零
		vTaskDelay(100);
	}
	return pdPASS;
}

static int robot_remote_enable_handle(float *param)
{
	robot_soft_reset_handle(param);	/* 复位 */
	ROBOT_STATUS_SET(g_robot.status, ROBOT_STATUS_RMODE_ENABLE);
	return robot_send_remote_event();
}

static int robot_remote_disable_handle(float *param)
{
	(void)param;
	ROBOT_STATUS_CLEAR(g_robot.status, ROBOT_STATUS_RMODE_ENABLE);
	robot_soft_reset_handle(param);	/* 复位 */
	return pdPASS;
}

static int robot_rel_rotate_handle(float *param)
{
	uint32_t joint_id = (uint32_t)param[0];
	return robot_send_rel_rotate_event(joint_id, param[1]);
}

static int robot_abs_rotate_handle(float *param)
{
	uint32_t joint_id = (uint32_t)param[0];
	return robot_send_abs_rotate_event(joint_id, param[1]);
}

static int robot_joints_sync_handle(float *param)
{
	return robot_send_auto_event((struct position *)param);
}


static int robot_soft_reset_handle(float *param)
{
	(void)param;
	return robot_send_reset_event(false);
}

static int robot_time_func_handle(float *param)
{
	return robot_send_time_func_event(param[0] * 1000);
}


static int robot_remote_event_handle(float *param)
{
	if (!ROBOT_STATUS_IS(g_robot.status, ROBOT_STATUS_RMODE_ENABLE)) {
		return pdPASS;
	}

	float vx = -param[0] * ROBOT_REMOTE_MAX_VELOCITY;
	float vy = param[1] * ROBOT_REMOTE_MAX_VELOCITY;
	float vz = (param[4] - param[5]) / 2 * ROBOT_REMOTE_MAX_VELOCITY;
	float rx = -param[3] * ROBOT_REMOTE_MAX_RPM;
	float ry = param[2] * ROBOT_REMOTE_MAX_RPM;

	taskENTER_CRITICAL();
	g_remote_control.vx = vx;
	g_remote_control.vy = vy;
	g_remote_control.vz = vz;
	g_remote_control.rx = rx;
	g_remote_control.ry = ry;
	taskEXIT_CRITICAL();

	return pdPASS;
}

static int robot_zero_handle(float *param)
{
	(void)param;
	LOG("robot reset zero.\n");
	// int i = 3;
	// {
	for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++) {
		Emm_V5_Reset_CurPos_To_Zero(i + 1);
		vTaskDelay(10);
	}
	return pdPASS;
}

static int robot_kin_verify_handle(float *param)
{
	(void)param;
	robot_kinematics_verify();
	return pdPASS;
}

/**
 * @brief 设置所有关节角度
 * 
 * 命令格式：set_joints angle1 angle2 angle3 angle4 angle5 angle6
 * 
 * @param param 6 个关节的目标角度（度）
 * @return int pdPASS 或 pdFAIL
 */
static int robot_set_joints_handle(float *param)
{
	float target_angles[6];
	
	// 1. 检查限位（在 cmd 层做简单验证）
	for (int i = 0; i < 6; i++) {
		if (param[i] < g_robot.joints[i].min_angle || 
			param[i] > g_robot.joints[i].max_angle) {
			LOG("ERROR: Joint %d target %.2f° out of limits\n", 
				i, param[i]);
			return pdFAIL;
		}
		target_angles[i] = param[i];
	}
	
	// 2. 发送事件到业务逻辑层
	return robot_send_set_joints_event(target_angles);
}

/**
 * @brief 获取所有关节的当前角度
 * 
 * 命令格式：get_joints
 * 
 * @param param 未使用
 * @return int pdPASS 或 pdFAIL
 */
static int robot_get_joints_handle(float *param)
{
	(void)param;
	float current_angle;
	
	// 读取并显示每个关节的当前角度
	LOG("Joints: ");
	for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++) {
		// 从电机读取当前角度
		if (robot_update_current_angle(i) != 0) {
			LOG("ERROR: Failed to read joint %d angle\n", i);
			return pdFAIL;
		}
		
		current_angle = g_robot.joints[i].current_angle;
		
		// 一行输出所有关节角度
		LOG("%5.2f ", current_angle);
	}
	LOG("\n");
	
	return pdPASS;
}

/**
 * @brief 获取机械臂末端位姿
 * 
 * 命令格式：get_eepose
 * 
 * @param param 未使用
 * @return int pdPASS 或 pdFAIL
 */
static int robot_get_eepose_handle(float *param)
{
	(void)param;
	return robot_send_get_eepose_event();
}

/**
 * @brief 计算正运动学（根据关节角度计算位姿）
 * 
 * 命令格式：fk j1 j2 j3 j4 j5 j6
 * 
 * @param param 关节角度 [j1, j2, j3, j4, j5, j6]
 * @return int pdPASS 或 pdFAIL
 */
static int robot_fk_handle(float *param)
{
	float result[6] = {0};
	
	// 调用正运动学函数
	int ret = robot_get_eepose(param, result);
	
	if (ret == 0) {
		LOG("FK Result: X=%.2f Y=%.2f Z=%.2f Roll=%.2f Pitch=%.2f Yaw=%.2f\n",
			result[0], result[1], result[2], result[3], result[4], result[5]);
	}
	
	return ret;
}

/**
 * @brief MG996R 舵机设置角度
 * 
 * 命令格式：mg_set angle [duration_ms]
 * 
 * @param param [0]=角度，[1]=时间 (ms)
 * @return int pdPASS 或 pdFAIL
 */
static int robot_mg996r_set_angle_handle(float *param)
{
	float angle = param[0];
	uint32_t duration_ms = (uint32_t)param[1];
	
	LOG("\nCommand: mg_set %.2f %d\n", angle, duration_ms);
	
	// 检查角度范围
	if (angle < 0 || angle > 180) {
		LOG("ERROR: Angle must be between 0 and 180\n");
		return pdFAIL;
	}
	
	// 发送事件到业务逻辑层
	return robot_send_mg996r_set_angle_event(angle, duration_ms);
}

/**
 * @brief MG996R 舵机获取角度
 * 
 * 命令格式：mg_get
 * 
 * @param param 未使用
 * @return int pdPASS 或 pdFAIL
 */
static int robot_mg996r_get_angle_handle(float *param)
{
	(void)param;
	
	LOG("\nCommand: mg_get\n");
	
	// 发送事件到业务逻辑层
	return robot_send_mg996r_get_angle_event();
}

/**
 * @brief 逆运动学求解，返回关节角度
 * 
 * 命令格式：get_ik x y z roll pitch yaw
 * 
 * @param param 位姿参数 [x, y, z, roll, pitch, yaw]
 * @return int pdPASS 或 pdFAIL
 */
static int robot_get_ik_solution_handle(float *param)
{
	float result[6] = {0};
	
	// 发送事件并等待结果
	int ret = robot_send_get_ik_solution_event(param, result);
	
	if (ret == pdPASS && result[0] != 0) {
		LOG("IK Solution: ");
		for (int i = 0; i < 6; i++) {
			LOG("%.2f° ", result[i]);
		}
		LOG("\n");
	}
	
	return ret;
}

static struct robot_cmd_info robot_uart1_cmd_table[] = {
	{"remote_event", robot_remote_event_handle},
	{"remote_enable", robot_remote_enable_handle},
	{"remote_disable", robot_remote_disable_handle},
	{"rel_rotate", robot_rel_rotate_handle},
	{"soft_reset", robot_soft_reset_handle},
	{"zero", robot_zero_handle},
	{"abs_zero", robot_abs_zero_handle},
	{"abs_restore", robot_abs_restore_handle},
	{"kin_verify", robot_kin_verify_handle},
	{"set_joints", robot_set_joints_handle},
	{"get_joints", robot_get_joints_handle},
	{"get_eepose", robot_get_eepose_handle},
	{"fk", robot_fk_handle},
	{"get_ik", robot_get_ik_solution_handle},
	{"mg_set", robot_mg996r_set_angle_handle},
	{"mg_get", robot_mg996r_get_angle_handle},
	// {"time_func", robot_time_func_handle},
	{NULL, NULL},
};

void robot_uart1_handle(struct robot_cmd *rb_cmd)
{
	static char event_type[20] = {0};
	float param[6] = {0};
	char *cmd = rb_cmd->cmd;
	int ret;
	
	// 调试打印
	// LOG("cmd: %s\n", cmd);

	ret = sscanf(cmd, "%19s %f %f %f %f %f %f", event_type, &param[0], &param[1], &param[2], &param[3], &param[4], &param[5]);
	// LOG("ret: %d, event_type: %s, param[0]: %.2f, param[1]: %.2f\n", ret, event_type, param[0], param[1]);
	if (ret < 1) { // 解析失败
        LOG("event_type parse error: %s\n", cmd);
        return;
    }

	for (int i = 0; robot_uart1_cmd_table[i].event_type != NULL; i++) {
		if (strcmp(event_type, robot_uart1_cmd_table[i].event_type) == 0) {
			ret = robot_uart1_cmd_table[i].cmd_func(param);
			if (ret != pdPASS) {
				LOG("[ERROR] [jid:%d] event_type:%s param:%.2f %.2f %.2f\n", event_type, param[0], param[1], param[2]);
				return;
			}
			return;
		}
	}

	LOG("uart cmd parse error: %s\n", cmd);
	return;
}

