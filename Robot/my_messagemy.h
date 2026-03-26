/*
 * 文件: my_messagemy.h
 * 描述: 消息解析模块头文件，声明来自相机/CAN 的通道变量及任务接口。
 * 作者: 扑哧宇
 * 日期: 2025-12-23
 */

#ifndef MY_MESSAGEMY_H
#define MY_MESSAGEMY_H
#include "main.h"

extern int16_t g_ch1;   // 前进/油门
extern int16_t g_ch2;   // 横向
extern int16_t g_ch3;   // 旋转
extern int g_ch4;       // 云台/航向角
extern int16_t g_ch5;   // 备用通道

void message(void const * argument);

#endif
