# v1.53 RGB 卸货对齐

配套 OpenMV 程序：

`../OpenMV_rgb_load_unload_align_v2.5.py`

## 工作流程

1. 装货阶段使用正常 RGB 图像识别红色货物。
2. 夹取完成后切换到 `MODE,LINE`，使用灰度二值反转图循迹。
3. 识别到卸货区入口后，STM32 发送 `MODE,UNLOAD`。
4. 摄像头回到装货时的固定角度 `10°/30°`，不进行云台左右搜索。
5. OpenMV 在正常 RGB 图像中同时识别夹爪内的红色货物和地面的黑色圆块。
6. STM32 根据黑色圆块中心相对红色货物中心的偏差控制车体左右转动及前后靠近。
7. 两者对齐并稳定 300 ms 后停车、放下并松开货物。
8. 夹爪与升降舵机回到上电初始化角度，随后进入 `DONE`，电机保持停止。

## 串口模式

- `MODE,RED`：装货 RGB 模式
- `MODE,LINE`：循迹黑白模式
- `MODE,UNLOAD`：卸货 RGB 对齐模式

OpenMV 卸货数据仍为：

`MV,UNLOAD,<水平偏差>,<垂直偏差>,<黑圆面积>,<前后状态>,<有效>`

## 主要调试参数

OpenMV v2.5：

- `UNLOAD_BLACK_THRESHOLD`：黑圆 LAB 阈值
- `UNLOAD_ALIGN_Y_TOLERANCE`：红色货物与黑圆的纵向对齐容差
- `UNLOAD_MIN_WIDTH` / `UNLOAD_MAX_WIDTH`：黑圆尺寸范围
- `UNLOAD_FILL_MIN_PERCENT` / `UNLOAD_FILL_MAX_PERCENT`：黑圆填充率范围

STM32：

- `CAR_UNLOAD_ALIGN_DEADZONE_DEG`：卸货左右对齐死区
- `CAR_TARGET_STABLE_MS`：对齐稳定等待时间
- `GRAB_CLAW_INIT_DEG` / `GRAB_LIFT_INIT_DEG`：任务结束后的夹爪初始位置
