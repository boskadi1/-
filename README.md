# STM32 + OpenMV 自主小车

## 当前推荐版本：v1.57

v1.57 使用 OpenMV v2.7 与 STM32 双横线状态机：

- 第一条横向黑线是寻线起始线，完整越过后才开始 PID 循迹。
- 第二条横向黑线是卸货区入口，识别后保持摄像头 `10°/10°` 并以左70、右60缓慢直行。
- 黑色圆块进入提前标定区域后，摄像头才下翻至 `10°/30°`。
- 下翻后不再依赖被红色货物遮挡的黑圆图像，而是按标定时间预测前进并卸货。
- 保留左右轮独立速度闭环、50 Hz 编码器反馈和蓝牙轮速遥测。

推荐固件：

```text
versions/v1.57/firmware/stm_cross_predict_unload_v157.elf
```

配套 OpenMV：

```text
versions/v1.57/openmv/OpenMV_cross_predict_unload_v2.7.py
```

## v1.56

v1.56 是高频轮速反馈版本：

- 左右轮使用独立编码器速度 PID 闭环。
- 编码器采样和电机速度闭环由 20 Hz 提高到 50 Hz。
- 蓝牙轮速遥测由 2 Hz 提高到 10 Hz。
- USART2 使用中断发送和4条消息队列，避免蓝牙发送阻塞循迹控制。

对应固件：

```text
versions/v1.56/firmware/stm_line_pid_speed_feedback_50hz_v156.elf
```

本仓库用于保存自主运输小车从 **v1.50** 开始的稳定版本及后续更新。

## 目录结构

```text
versions/
├─ v1.50/         首个稳定上传版本
├─ v1.51/         卸货入口转云台修复
├─ v1.52/         车体左右搜索版本
└─ v1.53/         当前推荐版本：RGB卸货对齐
   ├─ stm32/      STM32CubeIDE 工程源码
   ├─ openmv/     OpenMV MicroPython 程序
   └─ firmware/   已编译的 STM32 ELF 固件
```

后续版本继续使用独立版本目录，避免新参数覆盖已经验证的版本。

## 当前推荐版本：v1.53

v1.53 在 v1.52 的搜索流程基础上修复卸货图像模式：

- 到达卸货区后发送新增的 `MODE,UNLOAD`，退出循迹黑白反转图。
- OpenMV恢复正常RGB图像，同时区分红色货物和黑色落货圆块。
- 摄像头固定在装货角度 `10°/30°`，云台左右不扫描。
- 车体根据黑圆中心相对红色货物中心的误差进行左右、前后对齐。
- 未识别到圆块时保留左70、右60前进1秒及车体左右原地搜索。
- 对齐稳定300 ms后释放货物，夹爪和升降舵机回到初始化角度。
- 完成后进入 `DONE` 并永久停车。

推荐固件：

```text
versions/v1.53/firmware/stm_unload_zone_recognition_v153.elf
```

## v1.50 功能

- 上电后摄像头转到装货视角，识别红色物块并完成夹取。
- 夹取成功后以基础轮速直行 2 秒，再切换到循迹模式。
- 使用 OpenMV 二值化、连续采样和拟合结果进行循迹。
- STM32 使用保留 D 项的 PD 控制计算左右轮差速。
- 左右轮基础速度独立设置，用于补偿相同电压下的转速差。
- 急弯采用“刹停—原地转向—刹停—缓慢直行”的分段动作。
- 识别卸货区入口的宽横带后，以左轮 70、右轮 60 缓慢寻找圆形落货点。
- 摄像头由循迹角度 `10°/10°` 转到装货角度 `10°/30°` 后，进行第二次视觉对齐。
- 二次对齐时支持低速左右转向、前进、后退；目标丢失时停车。
- 圆形落货点稳定对准 300 ms 后才执行卸货。

## 任务流程

```text
上电
  → 红色物块识别
  → 低速对准并夹取
  → 基础轮速前进 2 秒
  → 摄像头回到循迹角度
  → PD 循迹
  → 识别卸货区宽横带
  → 低速寻找圆形落货点
  → 摄像头转到 10°/30°
  → 二次视觉对齐
  → 稳定确认 300 ms
  → 卸货
```

## OpenMV

程序位置：

```text
versions/v1.50/openmv/OpenMV_sensor_load_exit_unload_circle_v2.4.py
```

硬件和通信参数：

- OpenMV Cam H7 Plus / OpenMV 5.0.0
- 图像分辨率：`320 × 240`
- UART：`UART3`
- 波特率：`115200`

主要通信：

```text
STM32 → OpenMV
MODE,RED
MODE,LINE

OpenMV → STM32
MV,LOAD,horizontal,vertical,area,grip_state,valid
MV,LINE,offset,heading,confidence,valid
MV,UNLOAD_ENTRY,valid
MV,UNLOAD,horizontal,vertical,area,position_state,valid
```

卸货区入口需要连续 3 帧检测到宽横带。圆形落货点的默认有效窗口：

```python
UNLOAD_ZONE_X_MIN = 110
UNLOAD_ZONE_X_MAX = 210
UNLOAD_ZONE_Y_MIN = 145
UNLOAD_ZONE_Y_MAX = 210
```

如果实车中圆形位置偏上或偏下，优先调整 OpenMV 文件中的上述四个参数。

## STM32

工程位置：

```text
versions/v1.50/stm32/
```

目标芯片：

```text
STM32F407ZGT6
```

使用 STM32CubeIDE 导入工程后即可编译。已经验证的固件位于：

```text
versions/v1.50/firmware/stm_unload_zone_recognition_v150.elf
```

关键参数集中在：

```text
versions/v1.50/stm32/Core/Inc/car_config.h
```

当前主要参数：

| 参数 | 数值 | 用途 |
|---|---:|---|
| `MOTOR_FORWARD_LEFT_SPEED` | 83 | 左轮基础前进速度 |
| `MOTOR_FORWARD_RIGHT_SPEED` | 70 | 右轮基础前进速度 |
| `MOTOR_LOAD_APPROACH_LEFT_SPEED` | 70 | 装货和卸货对齐左轮速度 |
| `MOTOR_LOAD_APPROACH_RIGHT_SPEED` | 60 | 装货和卸货对齐右轮速度 |
| `CAMERA_LINE_PAN_DEG` | 10° | 循迹水平角度 |
| `CAMERA_LINE_TILT_DEG` | 10° | 循迹俯仰角度 |
| `CAMERA_PICKUP_PAN_DEG` | 10° | 装卸货水平角度 |
| `CAMERA_PICKUP_TILT_DEG` | 30° | 装卸货俯仰角度 |
| `CAR_TARGET_ALIGN_DEADZONE_DEG` | 8° | 二次对齐横向死区 |
| `CAR_TARGET_STABLE_MS` | 300 ms | 对准稳定确认时间 |

## v1.50 卸货对齐逻辑

1. 循迹视角检测到卸货区入口后，不在入口处刹停。
2. 小车低速前进寻找独立的圆形/椭圆形落货点。
3. 圆形进入第一次有效窗口后停车，摄像头转到 `10°/30°`。
4. 云台到位后，必须使用新视角重新收到有效圆形数据。
5. 横向偏差超过死区时低速原地转向。
6. 圆形偏远时低速前进，过近时低速后退。
7. 目标丢失或前方出现紧急障碍时停车。
8. 新视角下连续稳定 300 ms 后执行卸货。

## 更新约定

- 已验证版本不直接覆盖。
- 每次功能更新新建版本目录。
- 同步更新 `CHANGELOG.md`。
- 提交前至少完成 OpenMV 语法检查和 STM32 编译。
