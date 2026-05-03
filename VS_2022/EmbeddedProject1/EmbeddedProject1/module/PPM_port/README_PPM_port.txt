PPM_port 集成说明（简要，中文）

文件：
 - ppm_port.h / ppm_port.c : 可移植 PPM 驱动
 - ppm_example_integration.c : 集成示例（STM32F1 标准库风格）
 - README_PPM_port.txt : 本说明

集成步骤：
1) 将 ppm_port.h / ppm_port.c 拷贝到目标工程合适目录并加入工程编译。

2) 提供时间源函数 get_time_us():
   - 必须返回微秒单位的时间戳，且单调递增。
   - 可以使用定时器计数器 + 溢出计数构成（示例：10000*TIME_ISR_CNT + TIM4->CNT）。

3) 在系统初始化时调用：
   ppm_port_init(get_time_us, frame_ready_cb);
   - frame_ready_cb 可为 NULL；如果提供，在解析到完整一帧（10 通道）时会被调用。

4) 在 PPM 引脚的 EXTI 中断处理函数中调用：
   - 在清除中断挂起位（硬件相关）后，调用 ppm_port_exti_handler();
   - 示例：在 EXTI9_5_IRQHandler 内检查并清除 EXTI_Line8 的中断，然后调用 ppm_port_exti_handler();

5) 读取数据：
   - 可选方式 A（推荐）：使用回调 frame_ready_cb 接收解析后的 10 通道数据。
   - 可选方式 B：轮询 ppm_port_get_frame(out_buf, 10)，若返回非 0 表示新帧已就绪并复制到 out_buf。

6) 注意事项：
   - 驱动仅负责 PPM 信号的边沿时间差解析，硬件相关（中断线、定时器）由移植者在目标工程中实现。
   - PPM 信号参数（通道脉宽范围、帧间隔阈值）与原实现保持一致（800~2200us 通道，>=2500/3000us 视作帧间隔），如需调整可修改 ppm_port.c 内的阈值常量。

示例已提供在 ppm_example_integration.c 中，拷贝并根据目标平台适配即可。

---
中国民航大学 飞凡创翼 团队 原 PPM 实现移植封装（示例）

