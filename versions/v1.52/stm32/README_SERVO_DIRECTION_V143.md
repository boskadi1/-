# v1.43 夹爪与云台方向修正

## 夹爪

夹爪当前安装方向与参考工程相反，因此改为：

```c
#define GRAB_CLAW_OPEN_DEG  (-38)
#define GRAB_CLAW_CLOSE_DEG (-75)
```

先用蓝牙逐步测试：

- `z`：夹爪逻辑角度增加 5°。
- `x`：夹爪逻辑角度减少 5°，当前机构应向闭合方向运动。
- `p`：执行完整夹取动作。
- `d`：执行完整放下动作。

如果 `-75°` 夹持力不足，可每次减少 5°，最低不要小于 `GRAB_CLAW_MIN_DEG=-90°`。

## 云台俯仰

逻辑上仍使用负角度表示向下，但通过输出方向参数匹配反装舵机：

```c
#define CAMERA_TILT_OUTPUT_SIGN       (-1)
#define CAMERA_TILT_CENTER_OFFSET_DEG   0
#define CAMERA_PICKUP_TILT_DEG        (-30)
```

- 蓝牙 `k`：云台向下 5°。
- 蓝牙 `i`：云台向上 5°。
- 如果仍然方向相反，将 `CAMERA_TILT_OUTPUT_SIGN` 改为 `1`。
- 如果方向正确但中位偏斜，调整 `CAMERA_TILT_CENTER_OFFSET_DEG`，建议每次调整 5°。

测试舵机时先架空小车并断开电机动力，确认机构没有到达机械极限。
