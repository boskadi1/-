# v1.54 装货完成后立即切换循迹图像

本版本不上传 GitHub，仅保存在本地。

修复内容：

- 夹取动作完成并确认 `GrabServo_HasObject() != 0` 后，STM32 立即向 OpenMV 发送 `MODE,LINE`。
- OpenMV 随即从正常 RGB 装货图像切换到灰度二值反转图像。
- 原有夹取成功后左83、右70直行2秒的动作保持不变。
- 2秒结束时再次确认 `MODE,LINE`，并进入循迹摄像头角度和循迹状态。

配套 OpenMV 程序仍为：

`../OpenMV_rgb_load_unload_align_v2.5.py`
