#include "my_can.h"
#include "my_tool.h"

// 全局参数与常量定义
#define WHEEL_RADIUS     0.067f   // 轮子半径 (6.7 cm)
#define WHEEL_BASE_R     0.24f    // 车体到轮子接触点的等效半径 (24 cm)
#define SQRT2_DIV_2      0.70710678118f  // sqrt(2)/2

PID_State_t pid_3508_state[4] = {0}; // 4个电机的速度环 PID 状态
int16_t output_speed[4] = {0};       // 4个电机最终的电流/控制量（待发送到 CAN）

// ������Ʋ���
PID_Params_t pid_3508_params = 
{
    .kp = 0.8f,
    .ki = 0.6f,
    .kd = 0.0f,
    .max_output = 16384,      // 输出限幅（最大绝对值）
    .max_integral = 3000,     // 积分限幅（最大绝对值）
    .alpha = 0.3f,            // 微分滤波系数（用于抑制测量噪声）
};

// 电流环 PID 状态与参数（电流环作为速度环的内环）
PID_State_t pid_3508_current_state[4] = {0};
PID_Params_t pid_3508_current_params = 
{
    .kp = 0.95f,
    .ki = 0.95f,
    .kd = 0.0f,
    .max_output = 1000,
    .max_integral = 3000,
    .alpha = 0.3f
};


/**
 * @brief 计算电流环 PID 输出（用于驱动 CAN 发送）
 * @details 该函数将 motor->given_current 视为电流环的反馈值，使用当前的电流环参数计算输出。
 * @param motor_id 电机 ID（0-3）
 * @return 返回电流环的输出值（整型），通常作为 CAN 发送的电流命令
 */
int16_t PID_3508_Current(uint8_t motor_id)
{
    const motor_measure_t *motor = get_chassis_motor_measure_point(motor_id);

    // 更新电流环运行状态
    pid_3508_current_state[motor_id].actual = motor->given_current;
    pid_3508_current_state[motor_id].error[1] = pid_3508_current_state[motor_id].error[0];
    pid_3508_current_state[motor_id].error[0] = pid_3508_current_state[motor_id].target - 
        pid_3508_current_state[motor_id].actual;

    float delta_p = pid_3508_current_params.kp * (pid_3508_current_state[motor_id].error[0]
                                                  - pid_3508_current_state[motor_id].error[1]);
    float delta_i = pid_3508_current_params.ki * pid_3508_current_state[motor_id].error[0];

    pid_3508_current_state[motor_id].output += delta_p + delta_i;

    // 输出限幅保护
    if (pid_3508_current_state[motor_id].output > pid_3508_current_params.max_output)
        pid_3508_current_state[motor_id].output = pid_3508_current_params.max_output;
    else if (pid_3508_current_state[motor_id].output < -pid_3508_current_params.max_output)
        pid_3508_current_state[motor_id].output = -pid_3508_current_params.max_output;

    return pid_3508_current_state[motor_id].output;
}


/**
 * @brief 全向轮运动学：将平面速度转换为四个轮子的角速度
 * @param vx 车体前向线速度 (m/s)
 * @param vy 车体横向线速度 (m/s)
 * @param omega 车体角速度 (rad/s)
 * @param motor_speeds 输出数组，长度至少为4，返回每个轮子的角速度 (rad/s)
 */
void omni_wheel_kinematics(float vx, float vy, float omega,float *motor_speeds) 
{
    // 根据全向轮构型计算各轮角速度 (rad/s)，矩阵按 ID0..ID3 排列
    motor_speeds[0] = (-SQRT2_DIV_2 * vx - SQRT2_DIV_2 * vy + omega * WHEEL_BASE_R) / WHEEL_RADIUS; // ID1 前右
    motor_speeds[1] = (SQRT2_DIV_2 * vx - SQRT2_DIV_2 * vy + omega * WHEEL_BASE_R) / WHEEL_RADIUS; // ID2 后右
    motor_speeds[2] = (SQRT2_DIV_2 * vx + SQRT2_DIV_2 * vy + omega * WHEEL_BASE_R) / WHEEL_RADIUS; // ID3 后左
    motor_speeds[3] = (-SQRT2_DIV_2 * vx + SQRT2_DIV_2 * vy + omega * WHEEL_BASE_R) / WHEEL_RADIUS; // ID4 前左
}

/**
 * @brief 设置速度环目标值
 * @param motor_id 电机 ID (0-3)
 * @param target 目标转速 (单位与速度环实现相关，通常为 RPM 或内部单位)
 */
void PID_3508_SetTarget(uint8_t motor_id, int16_t target)
{
    pid_3508_state[motor_id].target = target;
}

/**
 * @brief 增量式速度环 PID 计算（速度环）
 * @param motor_id 电机 ID (0-3)
 * @return 返回速度环计算得到的输出（通常为期望电流或电流环目标）
 */
int16_t PID_3508_Incremental(uint8_t motor_id)
{
    const motor_measure_t *motor = get_chassis_motor_measure_point(motor_id);
    
     // 更新速度环运行状态
    pid_3508_state[motor_id].actual = motor->speed_rpm;
    pid_3508_state[motor_id].error[1] = pid_3508_state[motor_id].error[0];
    pid_3508_state[motor_id].error[0] = pid_3508_state[motor_id].target - 
        pid_3508_state[motor_id].actual;
    // ��������
    float delta_p = pid_3508_params.kp * (pid_3508_state[motor_id].error[0] 
			              - pid_3508_state[motor_id].error[1]);
		
    float delta_i = pid_3508_params.ki * pid_3508_state[motor_id].error[0];		  
		
    pid_3508_state[motor_id].output += delta_p + delta_i;
    
    return pid_3508_state[motor_id].output;
}
