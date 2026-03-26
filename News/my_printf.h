/*
 * 文件: my_printf.h
 * 描述: 轻量级 printf 封装头文件，声明 `usart_printf` 与串口打印任务接口。
 * 作者: 扑哧宇
 * 日期: 2025-12-20
 */

#ifndef SP_H
#define SP_H
#include "remote_control.h"

extern const RC_ctrl_t *local_rc_ctrl;

void usart_printf(const char *fmt,...);
void sp_printf(void const * argument);

#endif
