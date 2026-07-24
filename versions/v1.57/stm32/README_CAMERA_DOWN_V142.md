# v1.42 上电云台向下初始化

上电动作：

1. 云台水平舵机保持中间位置。
2. 俯仰舵机先在中位保持 300 ms。
3. 俯仰舵机每 20 ms 变化 1°，从中位缓慢转到 `-30°` 的夹爪俯视角。
4. 云台没有到达俯视角之前，底盘保持停止。
5. 到位后才允许根据 OpenMV 红块数据接近和夹取。

参数位于 `Core/Inc/car_config.h`：

```c
#define CAMERA_PICKUP_PAN_DEG          10
#define CAMERA_PICKUP_TILT_DEG        (-30)
#define CAMERA_BOOT_CENTER_HOLD_MS     300U
```

如果实车在 `-30°` 时向上转，说明俯仰舵机安装方向相反，将
`CAMERA_PICKUP_TILT_DEG` 改为正角度，例如 `30`，并确认机械结构不会卡住。

配套 OpenMV 使用 sensor 版本：`../OpenMV_sensor_visual_grab_then_line_v2.2.py`。
