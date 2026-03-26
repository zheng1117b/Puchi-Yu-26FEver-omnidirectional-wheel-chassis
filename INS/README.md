INS 模块说明
=================
作者：扑哧宇
日期：2026.1.21

概述
----
INS 目录实现云台惯性测量单元（IMU）相关的驱动、中间件与任务。
主要功能：
- 通过 BMI088（陀螺仪 + 加速度计）和 IST8310（磁力计）采集传感器数据；
- 使用 SPI + DMA 从传感器读取数据以降低 CPU 负担；
- 使用 Mahony AHRS 实时融合陀螺、加速度与磁力计，输出四元数与欧拉角；
- 对 BMI088 的温度进行加热和 PID 控制以获得稳定输出。

目录结构与职责
----------------
- BMI088driver.c/h: 与 BMI088 寄存器直接交互，读取原始传感器寄存器。
- BMI088Middleware.c/h: 将 driver 读取的原始数据解析为物理量，并提供采样接口。
- BMI088reg.h: BMI088 寄存器地址定义。
- ist8310driver.c/h: IST8310 磁力计寄存器访问与初始化。
- ist8310driver_middleware.c/h: 磁力计数据转换与校准接口。
- MahonyAHRS.c/h: Mahony AHRS 算法实现（四元数融合）。
- pid.c/h: 通用 PID 实现（用于温度控制等）。
- bsp_spi.c/h: 板级 SPI 初始化与 DMA 启停封装。
- bsp_imu_pwm.c/h: IMU 加热 PWM 驱动控制。
- bsp_delay.c/h: 延时等小工具。
- INS_task.c/h: INS 任务，负责初始化、数据调度、AHRS 更新、温度 PID 控制等。

模块交互流程图（简化 ASCII 版本）
---------------------------------
1) INS_task（主流程）

    +-------------------------+
    | INS_task (init)         |
    | - 初始化 IST8310/BMI088  |
    | - 初始化 PID, AHRS      |
    +-----------+-------------+
                |
                v
    +-------------------------+
    | 等待 DRDY / 定时轮询    |
    +-----------+-------------+
                |
                v
    +-------------------------+
    | 接收数据 (DMA/ISR 唤醒) |
    | - 从驱动读取 gyro/accel |
    | - 调用 AHRS_update()    |
    | - 计算 Euler angles     |
    +-------------------------+

更新记录：

- 2026-03-22：添加更新记录部分。

2) BMI088 数据读取（DRDY -> DMA -> 处理）

    DRDY_GPIO_ISR -> set "accel/gyro update flag" -> imu_cmd_spi_dma() ->
    CSx = LOW -> SPI_DMA_enable(tx, rx, len) -> DMA 完成 IRQ ->
    DMA IRQ: set UPDATE flag, CSx = HIGH -> 触发任务唤醒 (SW interrupt)

3) DMA / IRQ 状态机（关键标志位）

    标志位集合(每次 DRDY 触发移位处理):
    - *_update_flag: DRDY 被触发（数据准备）
    - *_SPI_flag: SPI 正在进行 DMA 传输
    - *_UPDATE_flag: DMA 读回数据已就绪，需要解析
    - *_NOTIFY_flag: 需要通知/唤醒任务进行后处理

各文件流程简要说明
-----------------
- BMI088driver: 提供底层寄存器读写与原始数据读取。
- BMI088Middleware: 把原始寄存器数据转换为以 g、dps、摄氏度为单位的物理量，并做必要的尺度/偏置转换。
- ist8310 driver/middleware: 类似 BMI088，用于磁力计读取与校准。
- MahonyAHRS: 提供 `MahonyAHRSupdate(quat, gx,gy,gz, ax,ay,az, mx,my,mz)`，以及辅助的角度计算函数。
- pid: 提供 PID 初始化与计算函数，用于温度控制。
- bsp_spi: 提供 `SPI1_DMA_init`, `SPI1_DMA_enable`, `SPI1_DMA_disable` 等封装，隐藏 HAL DMA 细节。
- INS_task: 集成以上模块，完成传感器调度、滤波与上层提供角度数据接口。

调试/拓展建议
---------------
- 若希望降低延迟，可将 INS_task 改为事件驱动（使用 RTOS 通知或信号量），在 DMA 完成时只进行最小唤醒并在任务中处理后续逻辑；
- 若需要更稳健的温度控制，可添加温度滤波与防积分饱和策略；
- 若要支持更多 IMU（例如添加 MPU6500），请在 BMI088 middleware 中抽象统一传感器接口。

