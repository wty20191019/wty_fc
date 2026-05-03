/* ppm_port.c - 可移植 PPM 驱动实现
 * 使用说明见 README_PPM_port.txt
 */

#include "ppm_port.h"
#include <string.h>

static u32 (*s_get_time_us)(void) = NULL;
static void (*s_frame_ready_cb)(const u16*, u8) = NULL;

static u16 s_buf[10];
static u16 s_frame[10];
static volatile u8 s_sample_cnt = 0;
static volatile u8 s_is_okay = 0;
static volatile u32 s_last_time = 0;
static volatile u32 s_time = 0;
static volatile u8 s_new_frame = 0;
static volatile u16 s_isr_cnt = 0;


// 初始化 PPM 驱动，外部调用
void ppm_port_init(u32 (*get_time_us)(void), void (*frame_ready_cb)(const u16* data, u8 len))
{
    s_get_time_us = get_time_us;
    s_frame_ready_cb = frame_ready_cb;
    s_sample_cnt = 0;
    s_is_okay = 0;
    s_last_time = 0;
    s_time = 0;
    s_new_frame = 0;
    s_isr_cnt = 0;
}


// 在外部中断（PPM 引脚）处理函数中调用，驱动不直接操作中断寄存器
void ppm_port_exti_handler(void)
{
    if (s_get_time_us == NULL) return;
    s_last_time = s_time;
    s_time = s_get_time_us();
    u32 delta = s_time - s_last_time;

    if (s_isr_cnt < 100) s_isr_cnt++;

    if (s_is_okay)
    {
        if (delta >= 800 && delta <= 2200)
        {
            if (s_sample_cnt < 10)
            {
                s_buf[s_sample_cnt++] = (u16)delta;
                if (s_sample_cnt >= 10)
                {
                    memcpy(s_frame, s_buf, sizeof(s_frame));
                    s_new_frame = 1;
                    s_is_okay = 0;
                    if (s_frame_ready_cb) s_frame_ready_cb(s_frame, 10);
                }
            }
        }
        else
        {
            if (delta >= 3000)
            {
                /* 帧结束，开始新一帧解析 */
                s_is_okay = 1;
                s_sample_cnt = 0;
            }
            else
            {
                s_is_okay = 0;
            }
        }
    }
    else if (delta >= 2500)
    {
        /* 认为是帧起始的长脉冲 */
        s_is_okay = 1;
        s_sample_cnt = 0;
    }
}

// 非必须接口，提供给外部读取最新帧数据
u8 ppm_port_get_frame(u16 *out_buf, u8 max_len)
{
    if ((out_buf == NULL) || (!s_new_frame)) return 0;
    u8 copy_len = (max_len < 10) ? max_len : 10;
    memcpy(out_buf, s_frame, copy_len * sizeof(u16));
    s_new_frame = 0;
    return copy_len;
}
