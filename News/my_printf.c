/*
 * 描述：轻量级的 printf 封装，用于格式化字符串并通过 USART6 + DMA 发送。
 * 作者：扑哧宇
 * 日期：2025-12-20
 * 说明：
 * 1) 使用静态发送缓冲区，避免占用过多栈空间。
 * 2) 使用 vsnprintf 防止缓冲区溢出。
 * 3) 该实现不是完全可重入：若多个任务（或中断）并发调用，且前一次 DMA 传输仍在进行，
 *    可能会覆盖静态缓冲区，造成打印内容错乱或内存覆盖。
 * 4) 在多任务场景下请使用互斥锁或消息队列进行串行化，或为每个任务分配独立缓冲区。
 * 5) 假设 usart6_tx_dma_enable 会启动非阻塞的 DMA 传输，并处理“前一传输仍在进行”的情况。
 * 优先建议：竞赛/测试时可关闭高频打印以避免影响控制环性能。
 */

#include <stdarg.h>
#include <stdio.h>
#include "main.h"
#include "usart.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "my_messagemy.h"
#include "my_can.h"
#include "my_tool.h"
#include "ins_task.h"

const RC_ctrl_t *local_rc_ctrl;

/*
* 功能：格式化字符串并通过 USART6​ 使用 DMA​ 发送。
* 参数：fmt：printf 风格的格式化字符串。
				...：与格式匹配的可变参数。
*/
void usart_printf(const char *fmt, ...)
{
  static uint8_t tx_buf[256];
  va_list ap;
  int ret;
  uint16_t len;

  va_start(ap, fmt);
  ret = vsnprintf((char *)tx_buf, sizeof(tx_buf), fmt, ap);
  va_end(ap);

  if (ret < 0) 
	{
    return;
  }

  if (ret >= (int)sizeof(tx_buf)) 
	{
    len = (uint16_t)(sizeof(tx_buf) - 1);
  } 
	else 
	{
    len = (uint16_t)ret;
  }

  tx_buf[len] = '\0';

  usart6_tx_dma_enable(tx_buf, len);
}


/*
 * 串口打印任务（FreeRTOS 任务）：周期性打印底盘电机转速等信息，便于调试
 * 注意：高频打印会占用 CPU/DMA 资源，影响控制环性能，仅用于调试
 */
void sp_printf(void const * argument)
{
  (void)argument;

  // 获取 4 个底盘电机的测量点
  const motor_measure_t *motor1 = get_chassis_motor_measure_point(0);
  const motor_measure_t *motor2 = get_chassis_motor_measure_point(1);
  const motor_measure_t *motor3 = get_chassis_motor_measure_point(2);
  const motor_measure_t *motor4 = get_chassis_motor_measure_point(3);
	
  while (1)
  {
		
	const fp32 *ins_angle = get_INS_angle_point();
  int ch4 = (int)(ins_angle[0] * 57.295779513f*50); // yaw 当前角度
			if (ch4 > 360 || ch4 < -360) 
			{
				ch4 %= 360;
				if (ch4 < 0) ch4 += 360;
			}
  // 当前输出格式: motor1_rpm,motor2_rpm,motor3_rpm,motor4_rpm,g_ch1\r\n
  // g_ch1 用于携带额外通道信息（例如摇杆或开关），便于调试遥控输入
	usart_printf("%d,%d\r\n",
                     g_ch5,
                     g_ch4
	);
//    /* 打印 INS 数据：角速度 (rad/s) 与欧拉角 (deg) */
//    // 获取当前角度数据（欧拉角）
//		//接收陀螺仪数据
//    fp32 ins_angle[3];
//    const fp32 *gyro_data = get_gyro_data_point();
//    ins_angle[0] = gyro_data[0] * 57.2958f; // rad -> deg
//    ins_angle[1] = gyro_data[1] * 57.2958f;
//    ins_angle[2] = gyro_data[2] * 57.2958f;
//		const fp32 *angle_data = get_INS_angle_point();
//    usart_printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
//		//陀螺仪解算数据
//		(int)(angle_data[0]*100),
//    (int)(angle_data[1]*100),
//    (int)(angle_data[2]*100),
//    //欧拉角数据
//    ins_angle[0],
//    ins_angle[1],
//    ins_angle[2]);

    osDelay(50); // 50 ms 周期
  }
}
