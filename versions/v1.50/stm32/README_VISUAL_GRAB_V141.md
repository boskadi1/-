# v1.41 俯视夹爪视觉夹取后循迹

本工程是从 v1.40 独立复制的新版本，未覆盖 v1.40、v1.39 或原始工程。

## 工作流程

1. 上电后摄像头水平舵机回到中间，俯仰舵机转到 `CAMERA_PICKUP_TILT_DEG`。
2. OpenMV 保持 RGB565 彩色图像，在画面中识别红色物块。
3. OpenMV 画出代表两爪之间有效空间的夹取窗口。
4. 红块位于窗口上方：小车对准并缓慢前进。
5. 红块中心进入夹取窗口：发送 `GRIP_READY`。
6. STM32 连续收到稳定状态 300 ms 后停车并闭合、抬升夹爪。
7. 夹取动作完成后发送 `MODE,LINE`，OpenMV 切换到灰度二值连续拟合循迹。

## 必须在实车上标定

打开 OpenMV IDE 查看彩色图像，将红块人工放到两爪正好能够夹取的位置，然后修改
`OpenMV_visual_grab_then_line_v2.1.py` 中：

```python
GRIP_ZONE_X_MIN = 110
GRIP_ZONE_X_MAX = 210
GRIP_ZONE_Y_MIN = 150
GRIP_ZONE_Y_MAX = 215
```

让画面中的黄色矩形位于两个张开的夹爪之间。红块进入有效位置后矩形会变成绿色，调试输出中的 `GRIP` 会变成 `2`。

摄像头俯视角在 `Core/Inc/car_config.h` 中调整：

```c
#define CAMERA_PICKUP_PAN_DEG   10
#define CAMERA_PICKUP_TILT_DEG (-30)
```

如果摄像头实际向相反方向转动，请将 `-30` 改为正值并逐步试验，避免一次转动角度过大导致机构碰撞。

## GRIP 状态

- `0`：红块未横向进入夹爪窗口，小车只进行方向对准。
- `1`：红块在夹爪窗口前方，小车缓慢前进。
- `2`：红块进入夹爪窗口，稳定后开始夹取。
- `3`：红块越过夹爪窗口，小车停车，避免继续碰撞。

配套 OpenMV 文件：`../OpenMV_visual_grab_then_line_v2.1.py`。
