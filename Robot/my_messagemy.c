/*
* 描述：消息解析任务（示例），将来自相机/CAN 的信号解析为全局遥控/通道变量。
* 作者：扑哧宇
* 日期：2025-12-23
* 说明：
* 1. 本模块维护若干全局通道变量（g_ch1 .. g_ch5），供其它模块读取。
* 2. 对于多任务访问，若需要写安全请加入临界区或消息队列以防数据竞争。
*/

#include "main.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "my_can.h"
#include "my_printf.h"

/* 全局通道变量 */
int16_t g_ch1 = 0;
int16_t g_ch2 = 0;
int16_t g_ch3 = 0;
int g_ch4 = 0;
int16_t g_ch5 = 0;

/* 消息解析任务 */
void message(void const * argument)
{
	(void)argument;
	while (1)
	{
		/* 读取CAN 信号结构指针 */
		const camera_signal_t *signal = get_camera_signal_point();
//		/* 获取全局遥控器数据结构指针 */
//		const RC_ctrl_t *rc = get_remote_control_point();
//		g_ch1=rc->rc.ch[2];
//		g_ch2=rc->rc.ch[3];
//		g_ch3=rc->rc.ch[5];

	g_ch1 = -(signal->ch3_value);  // 前进
	g_ch2 =  signal->ch1_value;    // 横向
	g_ch3 =  signal->ch5_value;    // 旋转
	g_ch4 =  signal->ch6_value;
	g_ch5 =  signal->ch7_value;

	// usart_printf("%d,%d\r\n",
    //                  g_ch1,
    //                  g_ch2
	// );
	osDelay(50);
	}
}
