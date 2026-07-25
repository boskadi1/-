# v1.61 寻线模式切换修复

配套 OpenMV 程序：

`OpenMV_timed_start_rgb_unload_v3.3.py`

## 本版修复

1. 夹取完成后立刻以基础轮速直行 4 秒，不再等待云台角度必须精确等于
   10°/10°，避免一直停留在 `LEAVE_LOAD_ZONE`。
2. 4 秒到达后 STM32 强制发送 `MODE,LINE`，进入 `LINE_ENTRY`。
3. OpenMV 每 200 ms 回传当前模式：
   `MV,MODE,RED/LINE/START/UNLOAD`。
4. STM32 收到 `MV,MODE,LINE` 后才使用寻线数据驱动车轮，避免模式未切换时
   误用旧图像数据。
5. 调试输出新增 `VM=请求模式/相机回报模式/确认有效`。

模式编号：RED=0、START=1、LINE=2、UNLOAD=3。

正常切换时 STM32 调试串口应依次出现：

`STAGE=LEAVE_LOAD_ZONE,VM=0/0/1`

`STAGE=LINE_ENTRY,VM=2/2/1`

随后识别到线：

`STAGE=LINE_NAVIGATE,LINE=.../.../.../1/...`

OpenMV IDE 串口应出现 `MODE=LINE`。
