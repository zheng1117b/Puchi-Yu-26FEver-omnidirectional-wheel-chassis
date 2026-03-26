/**
 ******************************************************************************
 * @file        INS_task.c
 * @brief       IMU/INS 子系统任务及相关 DMA/中断处理。
 * @author      扑哧宇
 * @date        2026.1.21
 * 本模块基于大疆官例实现 INS 任务，主要功能包括：
 *  - 初始化 BMI088（陀螺仪+加速度计）与 IST8310（磁力计）。
 *  - 利用 SPI+DMA 在 DRDY 中断触发下读取传感器数据。
 *  - 运行 Mahony AHRS 算法，将陀螺仪/加速度计/磁力计数据融合成四元数与欧拉角。
 *  - 通过 PID 控制 IMU 加热器，稳定传感器温度。
 *
 * 设计要点：
 *  - DRDY 的 GPIO EXTI 回调仅置位标志并尝试启动 DMA；繁重工作被推迟到 DMA 中断
 *    和 INS 任务中完成。
 *  - 使用多个 volatile 标志变量，为每个传感器通道编码小型状态机
 *    （DRDY → SPI → UPDATE → NOTIFY），以安全地协调 ISR 与任务上下文。
 *  - 启动 DMA 前会检查通道“EN”位，并在 ISR 上下文中使用关键段进行保护。
 *
 * 线程与同步概览：
 *  - DRDY EXTI → 置位 *_update_flag → 调用 imu_cmd_spi_dma()（ISR 安全）
 *  - imu_cmd_spi_dma() → 启动 SPI DMA 并拉低 CS 引脚
 *  - DMA 传输完成中断 → 置位 *_UPDATE_SHFITS 位，拉高 CS 引脚，再次调用
 *    imu_cmd_spi_dma() 以链式启动后续传输，并通过软件中断唤醒 INS 任务进行数据处理。
 *
 ******************************************************************************
 */

#include "INS_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_imu_pwm.h"
#include "bsp_spi.h"
#include "bmi088driver.h"
// 已移除 ist8310driver.h
#include "pid.h"
#include "MahonyAHRS.h"
#include "math.h"
#include "my_printf.h"

// 在 INS_task.c 的全局变量区定义
uint8_t ins_is_calibrated = 0; // 0: 校准中, 1: 校准完成可使用

/* 宏：把 PWM 设置封装，便于在代码中直接用 IMU_temp_PWM(x) 控制加热器占空比 */
#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)    // 调用底层 PWM 设置函数

/* 函数原型（文件内静态函数） */
static void imu_temp_control(fp32 temp);      // BMI088 温度控制（内部使用）
static void imu_cmd_spi_dma(void);            // 根据更新标志选择合适的 SPI DMA 读传感器

/* AHRS 接口封装（外部调用 MahonyAHRS） */
void AHRS_init(fp32 quat[4], fp32 accel[3]);
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3]);
void get_angle(fp32 quat[4], fp32 *yaw, fp32 *pitch, fp32 *roll);

extern SPI_HandleTypeDef hspi1;               // SPI1 在 HAL 中的句柄（在 main.c 或 stm32f4xx_hal_msp 中配置）
static TaskHandle_t INS_task_local_handler;   // 本任务的 FreeRTOS 任务句柄，用于 ISR 唤醒任务

/* --------------------------- DMA 缓冲区定义 ---------------------------
 * 这里的 tx_buf 在每次 SPI DMA 启动时会被发送出去（一般是读取寄存器指令 + 填充 0xFF）
 * rx_buf 用于接收从设备返回的数据（DMA 会填充）
 * SPI_DMA_*_LENGHT 是对应的缓冲区长度常量（在头文件中定义）
 * ------------------------------------------------------------------*/
uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] =
    {0x82,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; // 0x82: 假设为读取 gyro 起始寄存器的命令（视驱动设定）

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] =
    {0x92,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; // 0x92: accel 读取命令

uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2,0xFF,0xFF,0xFF}; // temp 读取命令

/* --------------------------- 更新标志（位域风格） ---------------------------
 * 使用多个 volatile 标志位变量来表示各个传感器、DMA、notify 的状态。
 * 这些变量被 ISR（外部中断）和 DMA 完成中断及任务上下文共同访问，因此需注意原子性。
 *
 * 约定（代码中常用的位偏移常量见头文件）:
 * - IMU_DR_SHFITS: 表示该传感器 DATA READY 中断发生（需要启动 SPI DMA）
 * - IMU_SPI_SHFITS: 表示该传感器正在通过 SPI DMA 传输（SPI BUS 占用中）
 * - IMU_UPDATE_SHFITS: 表示该传感器的 DMA 已完成，数据可由上层处理（task 中解析）
 * - IMU_NOTIFY_SHFITS: 表示发送通知给 INS_task（唤醒进行数据解析）
 * ------------------------------------------------------------------*/
volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
// 已移除 mag_update_flag
volatile uint8_t imu_start_dma_flag = 0; // 任务初始化后置 1，允许 DMA 被 ISR 启动

/* 传感器数据结构：实际数据会通过 BMI088/IST8310 驱动解析后填入这些结构 */
bmi088_real_data_t bmi088_real_data;
// 已移除 ist8310_real_data

/* 温控 PID：控制 BMI088 的加热器（保持目标温度，防止温漂）
 * first_temperate: 初始阶段标志（发动机还未稳定到目标温度）
 */
static uint8_t first_temperate;
static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
static pid_type_def imu_temp_pid;

/* AHRS / 角度相关数据：
 * - INS_quat: 四元数（q0..q3）
 * - INS_angle: 欧拉角 yaw/pitch/roll（单位：度）
 */
fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};      // euler angle, 单位 degree（代码内部 get_angle 转换为 57.3）

// ... 前面定义的缓冲区保持不变 ...

// 增加优化变量
static fp32 gyro_offset[3] = {0, 0, 0};
static fp32 gyro_filtered[3] = {0, 0, 0};
#define GYRO_LPF_ALPHA 0.85f   // 低通滤波系数，值越大越平滑但延迟越高 (0~1)

/**
  * @brief  IMU 任务主循环 (赛级终极优化版)
  */
void INS_task(void const *pvParameters)
{
    osDelay(INS_TASK_INIT_TIME);

    // 1. 初始化传感器 (带重试机制，防止上电波形不稳定导致初始化失败)
    while (BMI088_init()) osDelay(100);
    // 磁力计已被移除，纯 6 轴运行

    // 2. 初始化温控 PID
    PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);

    // 3. 准备 SPI DMA
    INS_task_local_handler = xTaskGetCurrentTaskHandle();
    SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
    imu_start_dma_flag = 1;

    uint8_t internal_is_calibrated = 0;
    uint16_t cali_count = 0;

    // 校准与滤波缓冲区
    static fp32 gyro_cali_sum[3] = {0};
    static fp32 accel_filtered[3] = {0};

    // 赛级滤波系数
    // 加速度计极易受摩擦轮和底盘震动干扰，需要较强的低通滤波 (保留 5% 新数据，95% 历史数据)
    const float accel_lpf_alpha = 0.05f;
    // 陀螺仪需要极低的延迟以保证云台 PID 稳定，因此只做轻微滤波防混叠 (保留 70% 新数据)
    const float gyro_lpf_alpha = 0.70f;

    while (1)
    {
        // 阻塞等待 BMI088 1KHz 的数据就绪信号
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS);

        // --- 1. 恒温控制 (核心：IMU 性能的基石) ---
        if (accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
            imu_temp_control(bmi088_real_data.temp);
        }

        // --- 2. 加速度计数据读取 ---
        if (accel_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET,
                                   bmi088_real_data.accel, &bmi088_real_data.time);
        }

        // --- 3. 陀螺仪数据处理与姿态解算 (高频回路) ---
        if (gyro_update_flag & (1 << IMU_NOTIFY_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_NOTIFY_SHFITS);
            BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);

            // 阶段 A：等待硬件温控彻底热透
            if (!first_temperate) {
                continue;
            }

            // 阶段 B：严格静态校准 (消除启动零偏)
            if (!internal_is_calibrated) {
                gyro_cali_sum[0] += bmi088_real_data.gyro[0];
                gyro_cali_sum[1] += bmi088_real_data.gyro[1];
                gyro_cali_sum[2] += bmi088_real_data.gyro[2];
                cali_count++;

                // 累加 1000 次 (1秒)，确保基础零偏极其准确
                if (cali_count >= 1000) {
                    gyro_offset[0] = gyro_cali_sum[0] / 1000.0f;
                    gyro_offset[1] = gyro_cali_sum[1] / 1000.0f;
                    gyro_offset[2] = gyro_cali_sum[2] / 1000.0f;

                    // 初始化加速度计滤波初值，防止开机瞬间出现解算跳变
                    accel_filtered[0] = bmi088_real_data.accel[0];
                    accel_filtered[1] = bmi088_real_data.accel[1];
                    accel_filtered[2] = bmi088_real_data.accel[2];

                    internal_is_calibrated = 1;
                    ins_is_calibrated = 1; // 释放全局标志，允许云台立刻上电发力

                    // 初始化 AHRS
                    AHRS_init(INS_quat, accel_filtered);
                }
                continue;
            }

            // --- 阶段 C：赛级实时滤波与解算 ---
            for(int i=0; i<3; i++) {
                // 1. 加速度计重度低通滤波：彻底滤除摩擦轮和底盘的高频机械震动，提取纯净重力向量
                accel_filtered[i] = accel_filtered[i] * (1.0f - accel_lpf_alpha) + bmi088_real_data.accel[i] * accel_lpf_alpha;

                // 2. 扣除初始静态零偏 (剔除动态吃零偏的危险逻辑)
                float gyro_raw = bmi088_real_data.gyro[i] - gyro_offset[i];

                // 3. 陀螺仪轻微低通滤波：防止采样混叠，同时保证极低延迟响应拉枪指令
                // 注意：这里去除了导致卡顿和稳态误差的死区(deadzone)逻辑
                gyro_filtered[i] = gyro_filtered[i] * (1.0f - gyro_lpf_alpha) + gyro_raw * gyro_lpf_alpha;

                // 二次过滤极端微小的浮点尾数防止 Mahony 内部出现下溢计算异常
                if (fabsf(gyro_filtered[i]) < 0.0001f) gyro_filtered[i] = 0.0f;
            }

            // --- 4. 姿态更新 (纯 6 轴运行) ---
            // 必须传入滤波后的 accel_filtered，否则 Pitch 和 Roll 会因为震动而疯狂抖动和缓慢漂移
            AHRS_update(INS_quat, 0.001f, gyro_filtered, accel_filtered);

            // --- 5. 获取欧拉角 ---
            get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET,
                      INS_angle + INS_PITCH_ADDRESS_OFFSET,
                      INS_angle + INS_ROLL_ADDRESS_OFFSET);

            // usart_printf("%.2f,%.2f,%.2f\n", INS_angle[0], INS_angle[1], INS_angle[2]);
        }
    }
}

void INS_Get_Angle(fp32 *yaw, fp32 *pitch, fp32 *roll)
{
    *yaw = INS_angle[0];
    *pitch = INS_angle[1];
    *roll = INS_angle[2];
}

void INS_Get_Gyro(fp32 *g_yaw, fp32 *g_pitch, fp32 *g_roll)
{
    *g_yaw = bmi088_real_data.gyro[0];
    *g_pitch = bmi088_real_data.gyro[1];
    *g_roll = bmi088_real_data.gyro[2];
}

/**
  * @brief          get the euler angle, 0:yaw, 1:pitch, 2:roll unit rad
  * @param[in]      none
  * @retval         the point of INS_angle
  */
extern const fp32 *get_INS_angle_point(void)
{
    return INS_angle;
}


/* --------------------------- AHRS 接口实现 --------------------------- */
/* 这里封装对四元数的初始化与更新的简单调用，便于未来替换算法 */

/**
 * @brief  AHRS init: 把四元数设置为单位四元数（无旋转）
 * @param  quat: 长度 4 的数组，按 q0 q1 q2 q3 存放
 * @param  accel: 加速度计
 */
void AHRS_init(fp32 quat[4], fp32 accel[3])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;
}

/**
 * @brief  AHRS update: 调用 MahonyAHRS 更新四元数
 * @param  quat: 四元数数组（输入输出）
 * @param  time: 采样时间间隔（秒）（此实现中未直接使用 time，而是在 Mahony 内部使用固定滤波率）
 * @param  gyro: 角速度（单位：？？），需与 Mahony 函数所需单位一致（通常 rad/s）
 * @param  accel: 加速度（单位：g）
 */
// void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3])
// {
//     // MahonyAHRSupdate 内部会更新 quat 数组（基于传入的传感器数据）
//     MahonyAHRSupdate(quat, gyro[0], gyro[1], gyro[2],
//                      accel[0], accel[1], accel[2],
//                      0.0f, 0.0f, 0.0f);
// }

/**
 * @brief  AHRS update: 切换为仅使用 IMU (陀螺仪 + 加速度计)
 */
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3])
{
    // 忽略传入的 mag 数据，直接调用 IMU 更新函数
    // 这样 Yaw 轴将完全基于陀螺仪积分，不会被错误的磁场数据“拽走”
    MahonyAHRSupdateIMU(quat, gyro[0], gyro[1], gyro[2],
                        accel[0], accel[1], accel[2]);
}

/**
 * @brief  将四元数转换为 Euler 角（度）
 * @param  q: 四元数
 * @param  yaw, pitch, roll: 输出对应角度指针（单位：度）
 *
 * 注意：返回值采用角度制（通过乘以 57.3 将弧度转度）
 * 公式来源：常见四元数->欧拉角转换（注意坐标系和符号约定）
 */
void get_angle(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll)
{
    // yaw (Z axis rotation)
    *yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                  2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.3f;

    // pitch (Y axis rotation)
    *pitch = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2])) * 57.3f;

    // roll (X axis rotation)
    *roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
                   2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.3f;
}

/* --------------------------- 温控 PID 实现 ---------------------------
 * imu_temp_control: 根据当前温度控制 BMI088 的加热器 PWM。
 *
 * 逻辑说明：
 * - 当 first_temperate == 0（初始尚未稳温），会一直把 PWM 打到最大，直到检测到温度达到某一阈值并稳定一段时间后，
 * 把 first_temperate 置 1 并启用 PID 控制。此处的 temp_constant_time 用来统计温度稳定持续时间，避免瞬态误判。
 * - PID_calc 使用的是位置式 PID，控制目标为 45.0f（目标温度），输出被限制在设置的最大 PWM 范围内。
 *
 * 参数与限制：
 * - temp: 传感器读回的温度（单位与驱动约定，一般为摄氏度）
 * ------------------------------------------------------------------*/
// 在 INS_task 外部增加零偏微调变量
static fp32 gyro_bias_dynamic[3] = {0};

static void imu_temp_control(fp32 temp)
{
    static uint16_t temp_constant_time = 0;
    // RM 赛级建议恒温设定在 45.0 度到 50.0 度，必须高于环境最高温
    const fp32 target_temp = 45.0f;

    if (first_temperate)
    {
        PID_calc(&imu_temp_pid, temp, target_temp);
        if (imu_temp_pid.out < 0.0f) imu_temp_pid.out = 0.0f;
        IMU_temp_PWM((uint16_t)imu_temp_pid.out);
    }
    else
    {
        IMU_temp_PWM(TEMPERATURE_PID_MAX_OUT - 1); // 全力加热
        // 在 imu_temp_control 函数中
        // 在 imu_temp_control 函数中
        if (temp > target_temp - 0.2f)
        {
            temp_constant_time++;
            // 【优化】将 2000 (2秒) 降低到 500 (0.5秒)
            // 只要达到目标温度附近并维持 0.5 秒，立刻放行，允许进入下一步校准
            if (temp_constant_time > 500)
            {
                first_temperate = 1;
            }
        }
    }
}

/* --------------------------- 外部中断回调（GPIO EXTI） ---------------------------
 * 本回调由 HAL 库在 GPIO 引脚产生中断时调用（在 stm32xx_it.c 或 HAL 层链入）。
 *
 * 约定：
 * - INT1_ACCEL_Pin: BMI088 accel 的 data ready 引脚
 * - INT1_GYRO_Pin: BMI088 gyro 的 data ready 引脚
 * - GPIO_PIN_0: 软件中断（在 DMA 完成后生成，用于唤醒任务）
 *
 * 处理流程：
 * 1) 加速计/陀螺仪 data ready：在对应的标志上设置 IMU_DR_SHFITS，若允许 DMA 启动（imu_start_dma_flag），则调用 imu_cmd_spi_dma
 * 2) GPIO_PIN_0：这是任务唤醒用的软件中断，调用 vTaskNotifyGiveFromISR 唤醒 INS_task
 *
 * 注意：
 * - 在 ISR 中尽量避免耗时操作（例如解析数据），这里 ISR 只设置标志并在必要时启动 DMA
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == INT1_ACCEL_Pin)
    {
        /* accel data ready：设置 DR 位并准备 SPI DMA */
        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS; // accel 同时触发温度读取

        if (imu_start_dma_flag)
        {
            imu_cmd_spi_dma(); // 在 ISR 内尝试启动 SPI DMA（该函数会处理竞态与 DMA 占用检查）
        }
    }
    else if (GPIO_Pin == INT1_GYRO_Pin)
    {
        /* gyro data ready */
        gyro_update_flag |= 1 << IMU_DR_SHFITS;

        if (imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if (GPIO_Pin == GPIO_PIN_0)
    {
        /* 软件中断：用于从 ISR 唤醒 INS_task（由 DMA 完成时触发） */
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

/* --------------------------- 基于标志的 SPI DMA 启动逻辑 ---------------------------
 * imu_cmd_spi_dma() 在 ISR 中被调用（当某个传感器触发 DR 中断时），通过检查各个标志与 DMA 状态来决定：
 * - 哪个 sensor 的 DMA 优先级高（一般先 gyro 再 accel 再 temp）
 * - 只有当当前 SPI 的 TX/RX DMA 通道未使能时（即 SPI 空闲）才启动下一次 DMA
 *
 * 设计要点与注意：
 * - 该函数在 ISR 中调用（taskENTER_CRITICAL_FROM_ISR），因此需要用 ISR 版本的临界区
 * - 检查 DMA 的方式是读取 HAL 的 DMA CR 寄存器判断 DMA 是否正在工作：
 * !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN)
 * 表示 TX 或 RX DMA 通道没有被使能
 * - 启动 DMA 前要根据对应的片选（CS）拉低，DMA 完成后在 DMA IRQ 中拉高 CS
 */
static void imu_cmd_spi_dma(void)
{
    UBaseType_t uxSavedInterruptStatus;
    uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR(); // 进入临界段（ISR 版本）

    /* 情形 1：gyro 优先启动（当有 gyro DR 且 SPI 空闲且 accel/temp 未占用 SPI） */
    if ((gyro_update_flag & (1 << IMU_DR_SHFITS)) &&
        !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) &&
        !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        // 从 DR -> SPI 状态迁移
        gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
        gyro_update_flag |= (1 << IMU_SPI_SHFITS);

        // 选中 gyro CS（拉低）并启动 DMA
        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus); // 退出临界段
        return;
    }

    /* 情形 2：accel 优先（如果 accel 有 DR 且 SPI 空闲且 gyro 未占用 SPI） */
    if ((accel_update_flag & (1 << IMU_DR_SHFITS)) &&
        !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) &&
        !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);

        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }

    /* 情形 3：accel_temp（温度寄存器）读取，优先级最低 */
    if ((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) &&
        !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) &&
        !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);

        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }

    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus); // 若没有条件满足则退出临界段
}

/* --------------------------- DMA IRQ Handler ---------------------------
 * 当 DMA RX 完成（传输完成标志）时，该中断会调用。通常这个 IRQ 是由 SPI RX 的 DMA 流触发。
 *
 * 流程：
 * 1) 检查 RX 完成标志并清除
 * 2) 根据哪个 SPI_TRANSFER 状态被占用（gyro/accel/accel_temp 的 IMU_SPI_SHFITS），把该状态转换为 IMU_UPDATE_SHFITS
 * 并拉高对应的 CS（结束 SPI 传输）
 * 3) 再次调用 imu_cmd_spi_dma() 尝试启动下一个等待的传输（实现链式传输）
 * 4) 如果有数据准备好（IMU_UPDATE_SHFITS），把它转换为 IMU_NOTIFY_SHFITS 并通过软件中断触发任务唤醒
 *
 * 注意：
 * - 这里使用 __HAL_DMA_GET_FLAG 与 __HAL_DMA_CLEAR_FLAG 等 HAL 宏来检查/清除 DMA 传输完成标志
 * - 触发任务唤醒使用 __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0)，这是一个软件触发的外部中断，随后会在 HAL_GPIO_EXTI_Callback 里处理唤醒逻辑
 */
void DMA2_Stream2_IRQHandler(void)
{
    if (__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
        // 清除 DMA 传输完成标志
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        /* 如果是 gyro 的 SPI 传输刚完成（IMU_SPI_SHFITS 被置位），切换到 IMU_UPDATE_SHFITS */
        if (gyro_update_flag & (1 << IMU_SPI_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
            gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

            // 结束 gyro CS（拉高），释放总线
            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
        }

        /* accel 传输完成处理 */
        if (accel_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }

        /* accel temperature 传输完成处理 */
        if (accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }

        /* 尝试启动下一个等待的 SPI DMA（链式传输），例如：gyro->accel->temp */
        imu_cmd_spi_dma();

        /* 如果有任何数据已迁移到 IMU_UPDATE_SHFITS（表示缓冲区可解析），将其映射为 NOTIFY 并通过软件中断唤醒任务 */
        if (gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            gyro_update_flag |= (1 << IMU_NOTIFY_SHFITS);

            // 通过模拟 EXTI 触发来唤醒任务（会在 HAL_GPIO_EXTI_Callback 中调用 vTaskNotifyGiveFromISR）
            __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
        }
    }
}
