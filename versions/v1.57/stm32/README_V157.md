# v1.57 双横线与预测卸货

配套 OpenMV：

`OpenMV_cross_predict_unload_v2.7.py`

## 任务时序

1. 上电后以正常 RGB 图像识别并夹取红色物块。
2. 夹取成功后切换为黑白循迹图像，摄像头回到循迹角度 10°/10°。
3. 小车以基础速度 83/70 前进。第一条横向黑线只作为寻线起始线。
4. OpenMV 必须连续确认横线出现 3 帧，并确认横线离开 3 帧；完整越过第一条线后才进入 PID 循迹。
5. 第二次识别到横向黑线时进入卸货搜索，但摄像头仍保持 10°/10°，小车以 70/60 缓慢直行。
6. 黑色圆块进入提前标定区域后立即停车，并将摄像头下翻到装货角度 10°/30°。
7. 下翻后不再依赖被红色货物遮挡的黑圆图像，而是以 70/60 预测前进一段标定时间，停车稳定后释放夹爪并结束任务。

## 需要现场标定的参数

OpenMV 文件：

- `UNLOAD_ZONE_X_MIN / UNLOAD_ZONE_X_MAX`：黑圆提前触发区的横向范围。
- `UNLOAD_ZONE_Y_MIN / UNLOAD_ZONE_Y_MAX`：黑圆提前触发区的纵向范围，当前为 115～165。若摄像头下翻太晚，应减小这两个值。
- `UNLOAD_ENTRY_MIN_WHITE`：横线最少白色像素数。
- `UNLOAD_ENTRY_CONFIRM_FRAMES`：横线出现确认帧数。
- `UNLOAD_ENTRY_CLEAR_FRAMES`：横线离开确认帧数。

STM32 `Core/Inc/car_config.h`：

- `CAR_START_LINE_CROSS_TIMEOUT_MS`：等待越过第一条横线的最大时间。
- `CAR_UNLOAD_PREDICT_FORWARD_MS`：摄像头下翻到位后的预测前进时间，当前 350 ms。
- `CAR_UNLOAD_PREDICT_SETTLE_MS`：预测前进后的停车稳定时间，当前 200 ms。
- `MOTOR_LOAD_APPROACH_LEFT_SPEED / RIGHT_SPEED`：卸货区低速直行，当前左 70、右 60。

如果最终落点偏前，减小 `CAR_UNLOAD_PREDICT_FORWARD_MS`；偏后则增大。建议每次以 50 ms 为步长调整。
