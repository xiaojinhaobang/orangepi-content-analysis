# 2026-10-26 B 下排 5cm 冻结快照

本目录冻结修改前的当前运行代码，用于回退和后续提速/视觉优化对比。快照不包含飞行数据、日志或固件二进制。

## 运行身份

| 组件 | SHA-256 |
|---|---|
| mission_node.py | `d27929b4c43eed476011125c0e1f82ad2ed1074ddc7dab0d0594c99b86e9861a` |
| body_vel_adapter.py | `15c494e560f26364bc7d53a5899b6dcb33ab5e6e7286c77039fcf9422c183ced` |
| arm_trigger.py | `42277656870d219b4c89517ab2595fe325ebcadb9b764303aa10a18882ee5b8d` |
| autostart_stack.sh | `c36e3c90ff0b1ad8d19499b8d8e43a5d3fc7c948e796d9dc53bd78da21fbb000` |
| fcstat_log.py | `aa90b6001b800cffb561a88fa95a93bcc04cbb455a1f6c1aa7b3759fc0d68cc3` |
| uart_to_stm32.cpp | `6996080f6fe380a9a2f57683609e28efcec0eaebe6e85b30bb2609eb5fd02534` |
| uart_to_stm32.hpp | `a270c2b19d2e276015902e408b0adbf931be4c9bcd216ce5be59366a6d0010ef` |
| waypoint_nav.py | `7d6d8fa3ba541bc4202994e35827c95663298e0c045e1e35ff9d47725d0ec974` |
| pos_hold.py | `f2a8b8af0f13b66e9f01d2edec107ac949efa6df72f391726399dfc0289ef5ab` |
| laser_driver.py | `8e56ff24fc36b5d93945e8af42d512b0797f0f30540102c62bfe08367660db11` |
| qr_vision_node.py | `be808e43155f053ac2d234486655aa12b26270e52804c82ffa1d368c7c2aba41` |

## F407

| 文件 | SHA-256 |
|---|---|
| ANO_LX.c | `07bbfe8280b72c2265f598912b3253030b0de9612dce46da4c54fee31df609a5` |
| AnoDTRasp.c | `a8553ed9d83b5c5b0355ac752c22ce71a9d70247635403d2d2578ca438c3ce2a` |
| LX_FC_Fun.c | `b8b49a4330486c9c67f49bcdc2c963213296111ecf859605a964a1d22a2c65e9` |

片内为扩展诊断固件：HEX `5D88DDDE66780AE9CF5287AE504D75007B1706BE3B3428E3A239D19F47EA0627`，回读 61,596 字节通过。

## 当前行为

- mission 仅把 B 下排（B4/B5/B6）及 B 下排进/出和换面过渡从 103cm 下移到 98cm。
- A 上下排、B 上排及激光判定阈值保持原值。
- 该快照是后续连续飞行、提速和视觉打点优化前的回退基线。

