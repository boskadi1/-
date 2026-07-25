# v1.54 + OpenMV v2.6 夹取后立即反转寻线

这是加入电机编码器速度 PID 之前的完整控制版本。

## 控制方式

- 循迹使用 P 项和 D 项，不包含 I 项。
- 保留循迹 D 项及连续拟合线处理。
- 左右轮使用独立基础速度和直接 PWM 控制。
- 不使用 v1.55 以后加入的编码器轮速 PID 内环。

## 夹取后的图像切换

配套 OpenMV 程序：

`openmv/OpenMV_pickup_immediate_line_v2.6.py`

OpenMV 上电时使用正常 RGB 图像识别红色货物：

1. 红块进入夹取窗口并得到 `GRIP=2`。
2. 夹爪合拢后红块连续三帧进入近区，即 `GRIP=3`。
3. OpenMV立即自行切换到灰度二值反转寻线。
4. 切换不依赖 STM32 到 OpenMV 的 `MODE,LINE` 指令；该指令仍作为第二条切换路径保留。
5. 切换成功后串口输出：

   `MODE=LINE AUTO_AFTER_PICKUP`

## 文件

- `stm32/`：完整 STM32CubeIDE 工程源码。
- `openmv/`：OpenMV MicroPython 程序。
- `firmware/stm_unload_zone_recognition_v154.elf`：已编译 STM32 固件。

## 已验证

- STM32 固件编译通过：text 42088、data 92、bss 3540。
- OpenMV v2.6 通过 Python 语法检查。
