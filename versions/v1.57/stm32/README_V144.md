# v1.44 夹爪位置互换与云台反向

本版本在 v1.43 的独立副本中修改，未覆盖旧版本。

夹爪位置：

```c
#define GRAB_CLAW_INIT_DEG  (-75)
#define GRAB_CLAW_OPEN_DEG  (-75)
#define GRAB_CLAW_CLOSE_DEG (-38)
```

即上电位于原来的夹取角度 `-75°`，执行夹取时移动到原来的初始位置 `-38°`，释放时返回 `-75°`。

云台俯仰输出方向：

```c
#define CAMERA_TILT_OUTPUT_SIGN 1
```

上电转向夹爪视角时，实际旋转方向与 v1.43 相反。
