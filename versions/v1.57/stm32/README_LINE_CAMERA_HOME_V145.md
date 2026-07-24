# v1.45 循迹前云台回初始位置

夹取阶段云台保持夹爪俯视角。夹取完成后：

1. STM32向OpenMV发送 `MODE,LINE`。
2. 云台缓慢回到上电初始位置：水平 `10°`、俯仰 `10°`。
3. 云台回位期间电机保持停止。
4. 水平和俯仰均到位后，才执行循迹控制。

参数位于 `Core/Inc/car_config.h`：

```c
#define CAMERA_LINE_PAN_DEG  CAMERA_PAN_INITIAL_DEG
#define CAMERA_LINE_TILT_DEG CAMERA_SCAN_MIDDLE_DEG
```

如需单独调整循迹视角，可直接将这两个宏改成具体角度。
