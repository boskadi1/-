# v1.40 红色物块夹取后循迹

本目录由 `stm_start_grab_v139` 独立复制生成，原工程和原 OpenMV 文件均未覆盖。

## 启动流程

1. STM32 上电进入 `CAR_MODE_VISION_LINE`，夹爪保持初始张开状态。
2. STM32 每 500 ms 向 OpenMV 发送 `MODE,RED`。
3. OpenMV 使用 RGB565 正常彩色图像识别红色物块，并发送
   `MV,LOAD,<水平角>,<垂直角>,<像素数>,<有效>`。
4. STM32 根据水平角转向，根据红块像素数判断距离；达到夹取位置并稳定后停车夹取。
5. 夹爪动作完成且 `GrabServo_HasObject()` 置位后，STM32 发送 `MODE,LINE`。
6. OpenMV 切换为灰度二值化和连续拟合循迹，发送 `MV,LINE,...`，小车进入循迹。

## 现场需要标定

- `Core/Inc/car_config.h` 中的 `CAR_LOAD_READY_AREA`：红块达到该像素数后开始夹取，当前为 6000。
- OpenMV 文件中的 `RED_THRESHOLD`：红色 LAB 阈值，当前为 `(20, 85, 20, 90, 5, 75)`。
- OpenMV 文件中的 `THRESHOLD`：循迹灰度二值化阈值，保持 v1.9 的 `(0, 100)`。

注意：当前“夹取成功”由夹爪动作流程完成来确认，并没有使用压力、限位或光电传感器。若需要判断物块是否真的在夹爪内，需要增加相应传感器输入。

配套 OpenMV 文件：`../OpenMV_red_pick_then_line_v2.0.py`。
