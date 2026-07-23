# v1.52 卸货区车体扫转搜索

## 搜索流程

```text
识别卸货区入口
  → 摄像头转到10°/30°并保持不动
  → 左70、右60慢速直行1秒
  → 未识别圆块：车体原地左转350 ms
  → 仍未识别：车体原地右转700 ms
  → 此后左右各700 ms交替扫转
  → 任意时刻识别圆块：立即进入视觉对齐
  → 左右修正、前进靠近或后退
  → 稳定300 ms
  → 卸货
```

## 转向参数

车体扫转直接调用已有的：

```c
CAR_ACTION_TURN_LEFT
CAR_ACTION_TURN_RIGHT
Motor_GetInPlaceTurnTargets()
```

轮速幅值：

```c
MOTOR_TURN_LEFT_WHEEL_SPEED  = 92
MOTOR_TURN_RIGHT_WHEEL_SPEED = 70
```

这些参数与原有蓝牙/自动原地转向使用同一套转向函数。

## 可调参数

位于 `stm32/Core/Inc/car_config.h`：

```c
CAR_UNLOAD_INITIAL_FORWARD_MS = 1000
CAR_UNLOAD_SWEEP_SIDE_MS      = 350
CAR_UNLOAD_SWEEP_CROSS_MS     = 700
```

## 使用文件

```text
openmv/OpenMV_sensor_load_exit_unload_circle_v2.4.py
firmware/stm_unload_zone_recognition_v152.elf
stm32/
```

