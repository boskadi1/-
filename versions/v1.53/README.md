# v1.53：RGB 卸货对齐与夹爪复位

本版本保留 v1.52 的循迹和车体搜索逻辑，修复卸货阶段在黑白反转图像中无法区分红色货物与黑色落货圆块的问题。

## 卸货流程

1. 循迹图像检测到卸货区入口。
2. STM32 向 OpenMV 发送 `MODE,UNLOAD`。
3. OpenMV 从黑白循迹模式切回正常 RGB 图像。
4. 摄像头固定回到装货角度 `10°/30°`，云台左右不转动。
5. OpenMV 同时检测夹爪内红色货物和地面黑色圆块。
6. 以“黑圆中心 − 红色货物中心”的偏差控制车体左右及前后运动。
7. 未识别时仍执行 v1.52 的左70、右60前进1秒和车体左右原地搜索。
8. 两个中心对齐并稳定300 ms后停车卸货。
9. 夹爪释放货物后，夹爪及升降舵机回到上电初始化角度。
10. 状态进入 `DONE`，电机保持停止，任务不再重新开始。

## 文件

- STM32CubeIDE工程：`stm32/`
- OpenMV程序：`openmv/OpenMV_rgb_load_unload_align_v2.5.py`
- 已编译固件：`firmware/stm_unload_zone_recognition_v153.elf`

## 新增通信指令

```text
STM32 -> OpenMV
MODE,UNLOAD

OpenMV -> STM32
MV,UNLOAD,horizontal,vertical,area,position_state,valid
```

卸货模式的 `horizontal` 和 `vertical` 表示黑色圆块相对红色货物中心的偏差，不再是相对固定图像窗口的偏差。

## 主要可调参数

OpenMV：

- `UNLOAD_BLACK_THRESHOLD`
- `UNLOAD_ALIGN_Y_TOLERANCE`
- `UNLOAD_MIN_WIDTH` / `UNLOAD_MAX_WIDTH`
- `UNLOAD_FILL_MIN_PERCENT` / `UNLOAD_FILL_MAX_PERCENT`

STM32：

- `CAR_UNLOAD_ALIGN_DEADZONE_DEG`：默认4°
- `CAR_TARGET_STABLE_MS`：默认300 ms
- `GRAB_CLAW_INIT_DEG`：夹爪初始化角度
- `GRAB_LIFT_INIT_DEG`：夹爪升降初始化角度
