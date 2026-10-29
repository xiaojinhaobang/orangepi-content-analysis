# 20261029 成功版本源码快照

本目录保存香橙派与 32 位飞控端当前源码，不包含飞行日志、备份文件、固件镜像或构建产物。

## 香橙派

- `orangepi/diag/`：`/home/orangepi/diag/` 的源码、脚本和配置（已排除 `backups/`、`__pycache__/`、`*.bak*`、`*.log`）
- `orangepi/ros2_ws/src/`：本次涉及的 ROS 包源码
  - `uart_to_stm32`
  - `my_launch`
  - `pid_control_pkg`
  - `qr_vision`
  - `pid_controller`
  - `activity_control_pkg`

## 32 位飞控端

- `f407/fc407_takeoff_guard/`：`ANO_LX.c`、`AnoDTRasp.c`、`LX_FC_Fun.c`
- `f407/fc407_patch/`：`AnoDTRasp.c/.h`

## 关键源码 SHA256

```text
8A716E0720967B84ACA330BEBDCC3E52838D109A1BF54FFEFB359D19053F5318  orangepi/diag/mission_node.py
514CB3451CF225416048237F2F2F2A1A4FD85F958219779D4F9DCBEF80C42714  orangepi/diag/body_vel_adapter.py
1241388450AF4B38DD88425E050BEEC1051CFAE991718AE637777696B5729260  orangepi/diag/waypoint_nav.py
07BBFE8280B72C2265F598912B3253030B0DE9612DCE46DA4C54FEE31DF609A5  f407/fc407_takeoff_guard/ANO_LX.c
A8553ED9D83B5C5B0355AC752C22CE71A9D70247635403D2D2578CA438C3CE2A  f407/fc407_takeoff_guard/AnoDTRasp.c
B8B49A4330486C9C67F49BCDC2C963213296111ECF859605A964A1D22A2C65E9  f407/fc407_takeoff_guard/LX_FC_Fun.c
001287F41CFBFC6D5699BBEE35E1A3A0CC3ACBD1EE48F6E4EEDBE915E9374900  f407/fc407_patch/AnoDTRasp.c
A11475FA3EFE0F94AC822DC2A00DF8FB0F132DE620045A5849B7A4C62B8902AD  f407/fc407_patch/AnoDTRasp.h
```

源码快照取自 2026-10-29 的 OrangePi live 目录；未在本目录保存二进制产物。
