# IMU_FilterPortable 使用说明（完全解耦版）

## 1. 模块定位

IMU_FilterPortable 是一个纯 C 的二阶低通滤波模块（Biquad LPF）。

特点：
- 不依赖任何 IMU 业务字段（K/B/重力系数等）
- 不依赖项目自定义类型（如 Vector3f）
- 仅依赖标准头 `<stdint.h>` 和 `<math.h>`
- 头文件内联实现，跨工程复制即用

文件：
- `IMU_FilterPortable.h`

## 2. 核心接口

### 2.1 系数设计
```c
void IMU_FilterPortable_DesignLP2(float sampleHz, float cutoffHz,
                                  IMU_FilterPortableBiquadCoeff *coeff);
```
用途：根据采样率和截止频率，生成二阶低通滤波器系数。

参数建议：
- `sampleHz`：采样频率（Hz）
- `cutoffHz`：截止频率（Hz）
- 通常满足 `0 < cutoffHz < sampleHz / 2`

### 2.2 状态初始化
```c
void IMU_FilterPortable_Init(IMU_FilterPortableBiquadState *state,
                             uint16_t warmupLimit);
```
用途：初始化滤波器状态。

说明：
- `warmupLimit` 为预热样本数
- 预热期间输出等于输入，避免上电初期瞬态

### 2.3 单点处理
```c
float IMU_FilterPortable_Process(float input,
                                 IMU_FilterPortableBiquadState *state,
                                 const IMU_FilterPortableBiquadCoeff *coeff);
```
用途：输入一个采样点，输出一个滤波点。

## 3. 最小移植示例

```c
#include "IMU_FilterPortable.h"

static IMU_FilterPortableBiquadCoeff lpfCoeff;
static IMU_FilterPortableBiquadState lpfState;

void FilterInit(void)
{
    IMU_FilterPortable_DesignLP2(200.0f, 10.0f, &lpfCoeff);
    IMU_FilterPortable_Init(&lpfState, 100U);
}

float FilterRun(float x)
{
    return IMU_FilterPortable_Process(x, &lpfState, &lpfCoeff);
}
```

## 4. 三轴数据建议

三轴建议使用三套独立状态：
- X/Y/Z 共用一套系数（若截止频率一致）
- X/Y/Z 各自独立状态，避免互相污染历史数据

## 5. 与业务逻辑的边界

建议把业务逻辑放在模块外：
- 标定修正：`origin = K * raw - B * scale`
- 单位转换：LSB 到 g、deg/s
- 数据融合：姿态/惯导/控制

本模块只负责“给定输入序列 -> 输出滤波序列”。

## 6. 注意事项

- 如果目标平台 `math` 性能有限，初始化阶段可离线计算系数后固化。
- 如果采样率变化，需要重新调用 `IMU_FilterPortable_DesignLP2`。
- 预热样本数可按系统启动时间和噪声情况调节。
