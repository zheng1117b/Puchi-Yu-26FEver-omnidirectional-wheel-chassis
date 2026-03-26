#include "my_tool.h"
#include "my_can.h"
#include "my_messagemy.h"
#include <math.h>
#include "my_printf.h"
#include "ins_task.h"
#include <stdbool.h>

// 状态机定义
enum ChassisMode {
    RC_MODE,            //遥控模式
    NAVIGATION_MODE     //导航模式
};

enum NavState {
    NAV_FORWARD,        // 前进
    NAV_TURN,           // 转弯
    NAV_BACKWARD        // 后退
};

static enum ChassisMode chassis_mode = RC_MODE;
static enum NavState nav_state = NAV_FORWARD;
static uint32_t nav_start_time = 0;
static bool ch3_was_not_770 = false;
static bool ch3_was_770 = false;

// 摆杆缩放因子：较小值->更高最大速度
// 示例：除以3时摇杆最大对应 speed_rpm ≈ 2000；除以6时约为1000
#define RC_JOYSTICK_DIVISOR 3

// 摇杆到线速度的缩放（每个摇杆单位对应的 m/s）
#define RC_TO_MPS   1.0f

// 电机目标 RPM 限幅（根据电机/减速箱规格调整）
#define MAX_MOTOR_RPM 2000

#define CLAMP(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

// 角度转弧度
#define DEG2RAD(x) ((x) * 3.1415926 / 180.0f)
 
/**
 * @brief 底盘电机转速设置函数
 */
void set_chassis_motor_speeds(void)
{
    // 触发检测和状态机更新
    if (chassis_mode == RC_MODE) {
        if (g_ch3 != 770) {
            ch3_was_not_770 = true;
        } else if (ch3_was_not_770 && !ch3_was_770) {
            ch3_was_770 = true;
        } else if (ch3_was_not_770 && ch3_was_770 && g_ch3 != 770) {
            // 触发导航模式
            chassis_mode = NAVIGATION_MODE;
            nav_state = NAV_FORWARD;
            nav_start_time = HAL_GetTick();
            ch3_was_not_770 = false;
            ch3_was_770 = false;
        }
    } else if (chassis_mode == NAVIGATION_MODE) {
        uint32_t current_time = HAL_GetTick();
        uint32_t elapsed = current_time - nav_start_time;
        if (nav_state == NAV_FORWARD && elapsed >= 6000) { // 6秒
            nav_state = NAV_TURN;
            nav_start_time = current_time;
        } else if (nav_state == NAV_TURN && elapsed >= 5000) { // 5秒
            nav_state = NAV_BACKWARD;
            nav_start_time = current_time;
        } else if (nav_state == NAV_BACKWARD && elapsed >= 3000) { // 3秒
            chassis_mode = RC_MODE;
        }
    }

    float motor_rpm[4] = {0};      // 四个轮子的角速度 (rad/s)

    if (chassis_mode == NAVIGATION_MODE) {
        // 导航模式：直接设置电机速度
        if (nav_state == NAV_FORWARD) {
            motor_rpm[0] = 800;
            motor_rpm[3] = 800;
            motor_rpm[1] = 0;
            motor_rpm[2] = 0;
        } else if (nav_state == NAV_TURN) {
            motor_rpm[0] = 0;
            motor_rpm[3] = 0;
            motor_rpm[1] = 800;
            motor_rpm[2] = 800;
        } else if (nav_state == NAV_BACKWARD) {
            motor_rpm[0] = -800;
            motor_rpm[3] = -800;
            motor_rpm[1] = 0;
            motor_rpm[2] = 0;
        }
    } else {
        // 遥控模式：原来的逻辑
		const fp32 *ins_angle = get_INS_angle_point();
		int ch4 = (int)(ins_angle[0] * 57.295779513f*50); // yaw 当前角度
	
        while (ch4 > 180) ch4 -= 360;
        while (ch4 < -180) ch4 += 360;
			
    
    int rocker_x = 0;
    int rocker_y = 0;
    // 获取遥控器数据并做初步缩放（使用缩放因子控制最大速度）
    rocker_x = g_ch2 / RC_JOYSTICK_DIVISOR;   // LX 调转速（横向）
    rocker_y = g_ch1 / RC_JOYSTICK_DIVISOR;   // LY 调转速（前后）

    // 把摇杆映射为世界坐标下的线速度（m/s）
    float vel_world_x = (float)rocker_y * RC_TO_MPS; // 前向
    float vel_world_y = (float)rocker_x * RC_TO_MPS; // 侧向
//    vel_world_x = CLAMP(vel_world_x, -MAX_VEL_MPS, MAX_VEL_MPS);
//    vel_world_y = CLAMP(vel_world_y, -MAX_VEL_MPS, MAX_VEL_MPS);

    // 旋转速度（由两种模式决定）：
    // - 正常：底盘跟随云台（闭环），根据云台 yaw 与底盘 yaw 差值计算 omega
    // - 小陀螺：当 g_ch3 == 770 时，强制给定固定旋转速度
    float omega = 0.0f;

    // 小陀螺模式优先
    if (g_ch3 == 770)
    {
        omega = 520.0f; // 小陀螺给定值（保持原有行为）
    }
    else
    {
      // 跟随云台：误差（期望 - 当前）
      float err = ch4 - g_ch4+4; // 偏移4度补偿误差
			if(fabs(err)<8||g_ch5==0)
			{
				err=0;
			}
					
        // 角度包裹到 [-180, 180]
        while (err > 180) err -= 360;
        while (err < -180) err += 360;
        
        // 比例系数：根据平台调参（默认值可改）
        const float YAW_TRACK_KP = 10.0f;
        omega = YAW_TRACK_KP * err;

        // 限幅，防止角速度过大（与小陀螺模式上限一致）
        omega = CLAMP(omega, -520.0f, 520.0f);
    }
    // 如果需要可启用摇杆对角速度的控制（取消下行注释并调整比例）
    // omega = (float)g_ch3 / RC_JOYSTICK_DIVISOR * (MAX_OMEGA_RADS / 100.0f);

    // 读取当前航向：为了让底盘按云台方向移动，需要把云台坐标速度映射到机体坐标。
    // 计算云台与机体的相对角度（弧度）：yaw_rel = yaw_gimbal - yaw_body
    float yaw_gimbal = DEG2RAD(g_ch4); 		// 云台目标/朝向（弧度）
    float yaw_body = ins_angle[0]*50;     // 机体当前航向（弧度）
    float yaw_rel = yaw_gimbal - yaw_body;

    // 使用旋转矩阵 R(yaw_rel) 将云台（世界/操作者）坐标下的速度转换到机体坐标：
    // [vbx]   [ cos -sin][vwx]
    // [vby] = [ sin  cos][vwy]
    float cos_y = cosf(yaw_rel);
    float sin_y = sinf(yaw_rel);
    float vel_body_x =  cos_y * vel_world_x - sin_y * vel_world_y;
    float vel_body_y =  sin_y * vel_world_x + cos_y * vel_world_y;

    // 计算各轮角速度（omni_wheel_kinematics 接受机体坐标速度：m/s，输出为 rad/s）
    omni_wheel_kinematics(vel_world_x, vel_world_y, omega, motor_rpm);


        // 级联控制：速度环计算期望电流 -> 电流环输出最终控制量
        // 说明：PID_3508_SetTarget 设置速度环目标，PID_3508_Incremental 返回速度环输出（作为电流环目标），
        // PID_3508_Current 根据电流环计算最终控制电流（output_speed）
        for(uint8_t i = 0; i < 4; i++)
        {
                // 速度环目标（这里直接使用 omni 计算的转速值，类型转换到 int16）
                PID_3508_SetTarget(i, (int16_t)motor_rpm[i]);
                // 速度环增量PID，输出为期望电流（内环目标）
                int16_t desired_current = PID_3508_Incremental(i);
                // 将期望电流写入电流环的 target，随后计算电流环输出
                pid_3508_current_state[i].target = desired_current;
                int16_t control_current = PID_3508_Current(i);
                output_speed[i] = control_current;
        }

        // 发送最终的电流控制量到底盘电机
        CAN_cmd_chassis(output_speed[0], output_speed[1], output_speed[2], output_speed[3]);
    }

}
