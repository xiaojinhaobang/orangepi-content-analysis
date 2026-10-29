# qr_vision

D 题二维码识别节点。解码内核来自 `学习资料/02_上位机与视觉/opencv_qr.py`，
已在本板实测：9/9 正确，单帧 `fast` 15~80 ms、全变体 245~493 ms。

## 数据流

```text
/dev/video0 (V4L2 MJPG)  ──>  qr_vision_node  ──>  /qr/result   (std_msgs/String, JSON)
                                                    /qr/code     (std_msgs/Int32, 1~24)
                                                    /qr/debug_image (sensor_msgs/Image)
                                                    /laser/trigger  (std_msgs/Bool, 0.5s 脉冲)
```

## 参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `mode` | `camera` | `camera` 用 V4L2 取帧；`image_dir` 回放图片（无相机自测用） |
| `device` | `/dev/video0` | 相机设备节点 |
| `width` / `height` / `fps` | `1280` / `720` / `30` | 采集分辨率与帧率 |
| `rotate` | `0` | `0/90/180/270` 画面旋转 |
| `image_dir` | `""` | `image_dir` 模式的图片目录 |
| `fast_decode` | `true` | 只解原图（快） |
| `variant_every` | `5` | 每 N 帧补跑一次全变体预处理（慢但召回高） |
| `confirm_frames` | `2` | 连续 N 帧同码才发布（SOLUTION.md §10 建议 ≥2） |
| `cooldown_sec` | `2.0` | 同码重复发布的最小间隔 |
| `laser_pulse_sec` | `0.5` | 激光笔点亮时长（题目要求 0.5 s 左右） |
| `publish_debug` | `true` | 是否发布标注图 |
| `serial_enabled` | `false` | 是否由本节点独占串口发帧 |
| `serial_port` | `""` | 留空且 `serial_enabled=true` 会告警跳过 |

## 运行

```bash
# 真机
ros2 launch qr_vision qr_vision.launch.py

# 无相机自测（回放测试图）
ros2 run qr_vision qr_vision_node --ros-args -p mode:=image_dir -p image_dir:=/home/orangepi/qr_vision/test

# 看结果
ros2 topic echo /qr/result
```

## 坐标映射

`code 1~24` 按 A/B/C/D 面各 6 位顺序映射：`1~6 → A1~A6`，`7~12 → B1~B6`，
`13~18 → C1~C6`，`19~24 → D1~D6`。若赛题实际贴法与顺序不同，改 `qr_decode.py`
里的 `code_to_position()` 即可。

## 串口发帧（默认关闭）

帧格式与 `serial_comm` 一致：`AA FF ID LEN DATA SUM ADD`，校验算法逐字节
`SUM += byte; ADD += SUM`（均按 256 取模）。数据为 3 字节 `[code, face, slot]`，
帧 ID 由 `serial_frame_id` 指定，默认 `0x41`。

注意：`/dev/ttyS6` 已由 `uart_to_stm32` 以 921600 占用，两处不能同时打开。
推荐做法是本节点只发话题，由 `uart_to_stm32` 增加一路 `0x41` 发送。

## 未做（待硬件到位）

- 激光笔实际驱动：本节点只发 `/laser/trigger`，落地要么 STM32 GPIO，要么 Pi GPIO。
- 三维定位：只有像素坐标，转机体坐标需要相机内外参或深度信息。
