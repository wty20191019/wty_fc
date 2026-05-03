/* 示例：如何在 STM32 工程中集成 ppm_port
 * 说明：此文件为示例代码，需根据目标工程调整头文件和定时器实现
 */

#include "ppm_port.h"
#include "stm32f10x.h" /* 根据平台调整 */

/* 外部：示例使用一个定时器 + 溢出计数构造微秒时间戳
 * 原项目的 TIME_ISR_CNT 与 TIM4->CNT 的组合示例：
 *   return 10000U * TIME_ISR_CNT + TIM4->CNT;
 * 这里仅给出示例函数原型，具体实现请在目标工程中提供
 */
extern volatile uint32_t TIME_ISR_CNT; /* 由定时器 ISR 增量 */

static uint32_t local_get_time_us(void)
{
    /* 下面为示例，假设定时器计数器单位为 1us，且 TIME_ISR_CNT 为较大周期的计数单位
     * 请根据目标工程的定时器频率修改
     */
    return 10000U * TIME_ISR_CNT + TIM4->CNT;
}

static void local_frame_ready_cb(const uint16_t *data, uint8_t len)
{
    /* 当解析到完整一帧（10 通道）时会回调到此处，len == 10
     * 在此处理或将数据复制到应用层缓冲区
     */
    (void)data; (void)len;
    /* 例如：通知任务、设置标志等 */
}

/* 在系统初始化时调用 */
void ppm_integration_init(void)
{
    ppm_port_init(local_get_time_us, local_frame_ready_cb);

    /* 配置 EXTI/引脚/中断（与原工程相同），示例：PA8 -> EXTI8
     * 用户需在此处或工程中配置中断优先级、GPIO、SYSCFG 等。
     */
}

/* 在中断向量中调用（示例：EXTI9_5_IRQHandler）
 * 注意：此函数示例不清除中断挂起位，目标工程中需要在调用前/后清除。
 */
void EXTI9_5_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line8) != RESET)
    {
        /* 清除挂起位（目标工程的 HAL/LL 或 标准库 函数） */
        EXTI_ClearITPendingBit(EXTI_Line8);

        /* 调用可移植 PPM 解析入口 */
        ppm_port_exti_handler();
    }
}

/* 可选：主循环读取最新帧（如果不使用回调） */
void app_poll_ppm(void)
{
    uint16_t buf[10];
    uint8_t got = ppm_port_get_frame(buf, 10);
    if (got)
    {
        /* 处理 buf */
    }
}
