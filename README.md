# Crayon Careful Chass

这是一个基于STM32F4的机器人底盘控制项目，使用FreeRTOS进行任务调度。

## 项目结构

- **Chass/**: 底盘子系统，负责移动和电机控制。
- **INS/**: 惯性测量单元，处理IMU数据和AHRS。
- **News/**: 消息传输和遥控数据接收。
- **Robot/**: 机器人应用层逻辑。
- **bsp/**: 板级支持包。
- **Drivers/**: STM32 HAL驱动。
- **Middlewares/**: 中间件，如FreeRTOS。

## 构建

使用CMake或MDK-ARM构建。

## 作者

扑哧宇

## 更新记录

- 2026-03-22: 创建项目级README.md。