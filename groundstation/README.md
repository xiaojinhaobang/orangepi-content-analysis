# D题 · 立体货架盘点无人机 · 地面站

2024 电赛 D 题地面站上位机（Python + PySide6）。电脑端显示盘点结果，
通信预留 **WiFi(UDP) / 蓝牙(串口)** 两条链路，内置**模拟机载端**可在无真机时完整调试。

## 快速开始

```bash
python main.py            # 启动地面站（停在遍历盘点页, 点"开始模拟"才跑）
python test_smoke.py      # 无头冒烟测试（无需显示器）
```

界面操作：**启动后不自动跑模拟**，点底部"模拟机载端 → 开始模拟"才开始，再点一次立即停止。
遍历盘点 / 定点盘点两个页面用右上角按钮互切，**切页只切页，不自动启停模拟**。
定点盘点页可输入目标编号（1~24，输入什么就定点盘点什么，留空随机抽二维码）→ 点"开始定点盘点"。
> 托盘右上角 ● 是虚拟 LED，每盘点到一个货物亮 1s（对应题目"LED 亮灭一次"）。

## 目录结构

```
main.py            入口
protocol.py        帧协议 v0（编解码 + 增量解析器）
links.py           链路抽象: UdpLink(WiFi) / SerialLink(蓝牙串口)
simulator.py       模拟机载端: 时间线(起飞/盘点24个/降落) + 独立UDP进程
models.py          盘点数据模型 + 无人机状态
led_controller.py  LED 抽象: 电脑端用 VirtualLed(屏幕灯), 可替换为 USB 灯
ui/
  main_window.py   主窗口: 链路选择/连接、帧路由、日志、状态栏
  page_control.py  遍历盘点页(24格总览/查询/清空复位/指示灯/切定点按钮)
  page_targeted.py 定点盘点页(目标货物/阶段/识别传回/大航线图/切遍历按钮)
  page_route.py    航线图控件(仓库俯视图 + 规划航线, 被定点页复用)
```

## 帧协议 v0（后续可调）

```
0xAA 0x55 | type u8 | len u8 | payload | sum u8
sum = (type + len + sum(payload)) & 0xFF
```

| type | 名称 | payload |
|---|---|---|
| 0x01 | EVENT 盘点事件 | 编号u8, 面u8(0=A..3=D), 位u8(1..6), 时间戳u32 |
| 0x02 | STATUS 状态 | 状态u8(0就绪1起飞2飞行3降落4落地), 高度u16(cm), 电量u8 |
| 0x03 | ROUTE 航线 | 点数u8, 每点{x i16, y i16}(cm) |
| 0x04 | REPORT 识别报送 | 编号u8 |
| 0x10 | QUERY 查询(下行) | 编号u8 |
| 0x11 | START 启动(下行) | 模式u8(0遍历1定点) |

构建/解析函数都在 `protocol.py`，机载端可 import 复用。

## 与真机联调

**WiFi(UDP)**：地面站选 `UDP (WiFi)`，填机载 IP 与端口（默认 8888）。机载端
把 `protocol.build_*` 打的帧发到该 IP:端口即可。地面站"开始模拟"走的就是这条真实
UDP 回路，可在无真机时验证链路。

**蓝牙(串口)**：机载端蓝牙模块配对后在 Windows 上表现为 COM 口，地面站选
`串口 (蓝牙/USB)` 填 COM 号与波特率。机载端实现等价于 `links.SerialLink.send`。

**独立模拟进程**（验证链路用）：
```bash
python simulator.py --udp --port 8888 --speed 5
```

**机载端雏形 `drone_link_test.py`**（UDP 真发, 即"以后搬到 RDX X5 上的程序"）：
- 开机即周期发 STATUS(高度/电量/状态) —— 地面站实时刷新, 不依赖内置模拟
- 收到地面站下行 START 就自动跑遍历/定点流程; 只依赖 stdlib+protocol+layout(无 Qt), 可直接搬去 RDX X5
```bash
# 本地单机: 不开 GUI 模拟, 自动真发(先遍历建立库存, 再定点演示)
python drone_link_test.py --run traverse --run targeted
# 无网络自检
python drone_link_test.py --selfcheck
# 两机(电脑跑地面站 UDP, RDX X5 跑本程序): 电脑填机载 IP, 机载侧 --
python drone_link_test.py --pc <电脑IP> --port 8888
```
> 协议限制(已核实): `MAX_PAYLOAD=64` => 一帧 ROUTE 最多 15 个航点(整条遍历 30 点 121B 发不出去)。
> 定点航线(≤5 点)由地面站规划, 没问题; 若机载侧要发大航线需改协议(扩 MAX_PAYLOAD/多帧拼接)。

## 待办 / 需校准项

- [x] 航线图货架布局已按题图 1 校准（2026-09-05 像素实测修正）：仓库 500×400，起飞区(75,325)、降落区(425,75)，货架1 杆 x=150 / 货架2 杆 x=350（长 y=100~300），A/C 朝西、B/D 朝东，全部在 `layout.py`；旧值 x=250/375 为误读已改
- [x] LED 方案已定: 用页面屏幕指示灯(`led_controller.VirtualLed`, 每盘点一个货物亮1s), 不接硬件
- [ ] 时间戳 0x01 帧为 u32 毫秒，与机载端时钟对齐（仅展示用，不影响判分）
- [x] 要求 2 定点盘点：`ui/page_targeted.py` 定点盘点页已完成（目标大标签/阶段/定点航线图高亮），模拟器 `mode="targeted"` 全流程联调通过
