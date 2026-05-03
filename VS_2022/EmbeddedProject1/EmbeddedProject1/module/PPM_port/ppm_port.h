/* ppm_port.h - 可移植 PPM 驱动接口
 * 目标：最小化与平台相关代码的耦合，通过回调提供时间源和帧就绪通知
 */
#ifndef PPM_PORT_H
#define PPM_PORT_H

#include <stdint.h>
#include <stddef.h>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

/* 初始化 PPM 驱动
 * get_time_us: 返回当前时间（单位微秒）的函数指针，必须单调递增
 * frame_ready_cb: 当解析出一帧（10 通道）时被调用，可为 NULL
 */
void ppm_port_init(u32 (*get_time_us)(void), void (*frame_ready_cb)(const u16* data, u8 len));

/* 在外部中断（PPM 引脚）处理函数中调用，驱动不直接操作中断寄存器 */
void ppm_port_exti_handler(void);

/* 非必须：从驱动读取最新帧数据
 * 返回复制到 out_buf 的通道数（0 表示无新帧）
 */
u8 ppm_port_get_frame(u16 *out_buf, u8 max_len);

#endif /* PPM_PORT_H */
