#ifndef MY_TOOL_H
#define MY_TOOL_H

#include "struct_typedef.h"

// PID参数结构体：用于保存比例、积分、微分系数及输出/积分限幅与滤波系数
typedef struct 
{
    float kp;                    // 比例系数
    float ki;                    // 积分系数
    float kd;                    // 微分系数
    int16_t max_output;          // 输出限幅（最大绝对值）
    int16_t max_integral;        // 积分项限幅（最大绝对值）
    float alpha;                 // 微分项低通滤波系数（0~1），用于抑制噪声
} PID_Params_t;

// PID状态结构体：用于保存运行时误差、积分、输出及上次微分等
typedef struct 
{
    int16_t target;              // 目标值
    int16_t actual;              // 实际值（反馈值）
    int16_t error[3];            // 误差历史，error[0]为当前误差 e(k)，error[1]为 e(k-1)，error[2]为 e(k-2)
    int16_t integral;            // 积分累计值
    int16_t output;              // 控制器输出
    float last_derivative;      // 上一次的微分项 d(k-1)，用于滤波或梯形微分等实现
} PID_State_t;

// 外部变量声明
extern PID_State_t pid_3508_state[4];  // 4个电机（如M3508）的PID状态
extern int16_t output_speed[4];        // 4个电机的目标输出速度（或PWM/转速指令）
extern PID_Params_t pid_3508_params;   // 3508电机的PID参数（全局共享）
// 电流环 PID 状态与参数（级联控制中的内环）
extern PID_State_t pid_3508_current_state[4];
extern PID_Params_t pid_3508_current_params;

// 函数声明
int16_t PID_3508_Incremental(uint8_t motor_id);  // 增量式PID计算，motor_id取值0~3
void PID_3508_SetTarget(uint8_t motor_id, int16_t target);  // 设置指定电机的目标值
void omni_wheel_kinematics(float vx, float vy, float omega,float *motor_speeds) ;
int16_t PID_3508_Current(uint8_t motor_id);


#endif
