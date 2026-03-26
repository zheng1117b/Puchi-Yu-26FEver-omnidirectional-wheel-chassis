/**
 * @file remote_control.c
 * @brief 遥控数据接收与解析（SBUS 协议），基于 UART + DMA 的空闲中断接收方案
 * @details 本模块负责从串口接收 SBUS 数据帧并解析到 RC_ctrl_t 结构，适配遥控器通道、鼠标与按键。
 *          使用 DMA 循环接收并在 UART 空闲中断时计算本次接收长度；当帧长正确时调用解析函数。
 * @note  此文件改写自 DJI 示例，已本地化注释为中文。
 */

#include "remote_control.h"

#include "main.h"


extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

/**
 * @brief 将一帧 SBUS 原始数据解析为 RC_ctrl_t
 * @param sbus_buf 指向 SBUS 原始数据缓冲区（至少 18 字节）
 * @param rc_ctrl  解析后填充的遥控器数据结构
 */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

// 遥控数据结构实例
RC_ctrl_t rc_ctrl;

// SBUS 接收缓冲区：SBUS 一帧为 18 字节，这里使用双缓冲（36 字节）以防止 DMA 越界
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/**
 * @brief 初始化遥控接收（配置 DMA 双缓冲等）
 */
void remote_control_init(void)
{
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}
/**
 * @brief 获取遥控数据结构指针（只读）
 * @return 返回全局的 RC_ctrl_t 指针
 */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}


//�����ж�
void USART3_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE) // 接收中断（RXNE）——字节接收就绪
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */
    
            //disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            // 获取本次接收数据长度：设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //�趨������1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
              // 长度匹配则解析到 rc_ctrl
              sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //�趨������0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
              // 长度匹配则解析到 rc_ctrl
              sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
            }
        }
    }
}


/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          ң����Э�����
  * @param[in]      sbus_buf: ԭ������ָ��
  * @param[out]     rc_ctrl: ң��������ָ
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    // 解析通道、开关、鼠标和按键数据
    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        // 通道0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; // 通道1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          // 通道2
               (sbus_buf[4] << 10)) & 0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; // 通道3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                      // 左开关
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                 // 右开关
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    // 鼠标 X
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    // 鼠标 Y
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  // 鼠标 Z
    rc_ctrl->mouse.press_l = sbus_buf[12];                                 // 鼠标左键
    rc_ctrl->mouse.press_r = sbus_buf[13];                                 // 鼠标右键
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    // 键盘按键位图
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 // 通道4（保留）

    // 去掉通道偏移量，得到以 0 为中值的通道数值
    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}
