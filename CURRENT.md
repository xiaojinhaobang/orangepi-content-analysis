## 2026-10-26 16:56 连续扫描/提速候选已部署（待飞）
- 基线已推送到 `https://github.com/xiaojinhaobang/orangepi-content-analysis`，提交`24be96a`。旧live备份：mission `d27929b4…`、BVA `15c494e5…` → `backups/*_pre_fast_continuous_20261026.py`。
- mission候选SHA `e23cc6bf…3fc2`：每排只保留进/出点，取消逐格航点；换排hold1.0→0.2s、换面hold0.5→0.2s；打点速度门20→26cm/s；新增基于连续两帧dx速率的预测过线打点（含“刚跨线但在瞄准窗内”的即时触发）。
- BVA候选SHA `b73217a7…8a64`：首段/过渡vff20→24、扫描17→24、过渡及扫描加速20→30cm/s²、扫描近端10→5cm、短段6→3cm、扫描横向纠偏3→4、扫描航向PD上限5→6°/s。起飞命令/解锁/100cm交棒/Z控制顺序未改。
- 验证：两文件py_compile PASS；`mission_node.py --check`仅生成进/出点航线；预测单测覆盖B4型稀疏帧、快速跨线和dy超限拒绝PASS。未做物理飞行验证，70s耗时尚未实测。
- 部署：flight-link重启后active/running，BVA日志确认首段/扫描24cm/s；mission PID10324单实例、BVA PID10104、`/velocity_map`48–49Hz。FC仍mode1/unlock0、PWM0、高度5cm，未发START或飞行。

## 2026-10-26 10:32 B下排扫描高度下移5cm
- 用户确认B面码比A面低，要求先下移5cm。候选文件 `mission_candidates/20261026_b_lower_minus5/mission_node.py`，基于运行版`a2eb8e05…`，新SHA `D27929B4…861A`。
- 仅新增`B_LOWER_SCAN_DOWN_CM=5.0`及`row_camera_z()`：B下排相机目标111.5→106.5，机体航高103→98cm；A上/下排和B上排不变。
- 板上离线`--check`验证：A45/A5/A6=103、B3/B2/B1=142、B4/B5/B6=98；B下排进/出点及换面过渡也随之为98。未修改MARK_DY_TOL、激光逻辑、起飞链或F407。
- 已备份板上旧mission `a2eb8e05…` → `backups/mission_pre_b_lower_minus5_20261026.py`；live mission已替换为`D27929B4…`并通过py_compile，任务节点PID15705单实例等待`/gs/start`。FC仍mode1/locked/PWM0。
- 待实飞验证B4/B5/B6是否触发激光及光斑上下偏差；本步仅为软件/航线验证，不代表物理打点通过。

## 2026-10-25 21:00 成功轮激光漏发定位到竖直门
- B4/B5/B6均识别成功并产生`EVENT`，但没有`/laser/trigger`日志；不是二维码识别失败，也不是激光驱动未响应。
- mission运行版`MARK_DY_TOL=0.35`，`_on_qr_center_cross()`在`abs(dy)>0.35`时直接清空横向track，窄瞄准窗兜底也要求`abs(aim_dy)<=0.35`。
- B4观测dy≈0.39–0.42、B5≈0.36–0.48、B6≈0.40–0.45，全部持续超过0.35，因此连“过激光中线”都无法成立；B5虽然在t885.4附近dx由+0.02到-0.07已接近目标-0.06，仍被dy0.45挡住。
- 对照B3打点时dy=+0.05、B2=+0.32、B1=+0.20，均在门内并成功点亮。根因是B4/B5/B6所在行的视觉竖直偏差/瞄准基准不匹配，不是激光器、订阅或驱动故障。
- 航线高度对照：漏打的B4/B5/B6均为B下排、z=103cm；B上排B1/B2/B3为z=142cm且均打点成功；A下排A4/A5/A6同为z=103但仍成功。数据不是“整个B面统一偏移”，而是B下排在画面中比A下排再低约0.15–0.2归一化量（按代码注释约4–5cm）。
- 当前不盲改；下一步应确认B4/B5/B6光斑相对码偏高还是偏低，再决定按面/按行设置aim_dy或调相机安装/高度。

## 2026-10-25 20:57 成功轮对比：自降未复现
- 数据位置 `flight_data/20261025_full_AB_success_205321/`。主rec SHA `3A9A3120…7D29`、fcstat SHA `5130484D…6E10`、mission.log SHA `34CDE65D…9E882`、journal SHA `D7D45A92…73A9`。
- 全程只发送一次`0x66[1,142]`，无降落子命令；`cmd1`仅0/4/5，`cmd1=6`为零条；`mission_end_req=0`、`onekey_land_calls=0`。飞行约106s后由操作手收油门落地。
- 自降轮在+4.5s、144cm进入`cmd1=6`；成功轮同版本起飞后升至142cm并持续飞行，未触发。说明该现象至少是间歇/状态相关，不是当前任务代码的确定性行为。
- 成功轮飞行段最低电压14.19V、最高PWM76.9%，比自降轮触发时15.61V更低但仍未自降；`rc_failsafe=0`。`cmd_lost/fbk_lost`仅在t193.73–193.78短暂为1，随后恢复且未触发降落。进一步削弱低电压/RC/短时速度超时的直接因果。
- QR识别12个货位；日志有12个EVENT、9次激光触发（B4/B5/B6未记录“激光打点”），不能写成12/12打点成功。
- 新电量boot MainPID1131，链路/任务/arm/fcstat均运行。故障未复现，需继续保留第二层诊断等下一次事件抓取。

## 2026-10-25 20:45 第二层诊断已刷写并实测
- F407 `0x0E`由6字节扩展为20字节：前5项保持兼容，新增`rc_in.fail_safe`、`mission_end_req`、`OneKey_Land()`调用计数、`cmd_lost/fbk_lost`、takeoff/flown/takeoff-request状态及CH3/CH5/CH6原始值。
- `onekey_land_calls`在`LX_FC_Fun.c:OneKey_Land()`函数入口递增；若核心自降时该计数不变，可排除F407主动调用降落。
- Keil构建PASS（0 error/0 warning，Code=59296B、RO=2112B、RW=592B、ZI=9280B）。源SHA：AnoDTRasp `DA07C13D…C5F`、LX_FC_Fun `F93A7EF8…276`；备份`*.bak-20261025-diag2`。候选HEX `5D88DDDE…0627`、BIN `253BB52D…BD03`（61,596B）。
- 用户明确授权后刷写：61,596B `Verify successful`，完整刷前1MiB备份`flash_before_fc_extdiag_20261025.bin`（SHA `81EDD365…5623`）。flight-link重启后MainPID 19218；新`uart_to_stm32`运行PID19341，新fcstat PID20478。
- `/fc_diag`实测为17项，CSV表头含`rc_failsafe,mission_end_req,land_calls,cmd_lost,fbk_lost,takeoff_if,flown_flag,tk_req,tk_fail,ch3,ch5,ch6`。地面值：15.44V、rc_lost0、rc_failsafe0、land_calls0、cmd_lost0、fbk_lost0、各状态0、CH3=1032、CH5=1004、CH6=1004。
- 未发START/解锁/飞行。飞前仍应整机断电重启以清除核心旧`cmdCID=16/cmd1=4`状态，再确认CH6低位和新增字段。

## 2026-10-25 20:00 带健康诊断的起飞自降
- 原始数据已保存 `flight_data/20261025_selfland_diag_200022/`：fcstat、mission/qr/wpn日志、主rec、journal和任务CSV。全程UART只有一次`Sent 0x66 sub=0x01 height=142`，无`0x66[2]`。
- 起飞受理后`+0.22s cmd1=5`；`+4.5s`、高度144cm时核心报`cmd1=6`，随后下降并落地。起飞前/自降中`mission_step`均为1，F407没有调用`OneKey_Land()`的路径。
- `cmd1=6`同帧：`volt=15.61V, rc_lost=0, imu_state=0, bar_sta=0, mag_sta=0`，PWM约50–61%；起飞爬升期最低电压15.47V，最高PWM71.8%。没有观测到低电压、RC失联或传感器状态跳变。
- 起飞交棒窗口曾出现`target_velocity stale 150ms`，UART发零速心跳，`20:00:18.794→19.384`约0.59s后恢复；自降发生在约1.6s后，时间相关但直接因果未证实。
- 落地后核验`mode1/unlock0/cmdCID16/cmd1=4`、四路PWM0、高度2cm。结论：本次仍是核心`cmd1=6`，香橙派和F407均未发降令；下一候选是尚未记录的`rc_in.fail_safe`/RC通道状态或核心内部保护，电压/失联/传感器三项已被现有数据显示为无事件。

## 2026-10-25 20:00 CH6一键链复现，低位后已恢复安全态
- 新boot `a740b048…c67a`。遥控关闭时 `/fc_diag rc_lost=1`，打开后回到0；电压16.07–16.10V，三项诊断正常落盘。
- 遥控恢复但CH6高位时，`/mission_step`曾显示2/3，并出现`mode2/unlock1/PWM18%`。按当前F407发送顺序，`/mission_step`来自0xF1首字节（`mission_end_sub`，即最后一条0x66子命令）；该轮2/3的具体发送者尚未定位，不能把它当作F407内部一键状态。
- 用户将CH6拨回低位后，实时核对`/mission_step=0`、`/fc_status=[1,0,1,0,0,0,0]`（mode1/locked/cmdCID0/cmd1=0）、四路PWM=0；未发START、未执行任务或飞行。后续起飞前必须确认CH6低位。

## 2026-10-25 19:52 核心健康诊断已刷写并实测
- 阶段：电压、RC失联、传感器状态三项诊断已刷入F407并在香橙派实测输出；未发START、未解锁、未飞行。
- F407 `0x0E` 为6字节：`voltage_100(u16 LE) + rc_no_signal + imu_state + bar_sta + mag_sta`，5ms任务每10拍发送，即20Hz；`voltage_100/100=V`。字段全部只读，不参与控制、起飞、任务或降落逻辑。
- Keil `Ano_LX`：errors=0/warnings=0，Code=59064B、RO=2112B、RW=592B、ZI=9280B；HEX SHA `A0AE7EF8…D171C`，源SHA `F1B7B31B…E19`；刷前源码备份 `AnoDTRasp.c.bak-20261025-diag`，其SHA `9482724E…CC75`。
- 香橙派 `/fc_diag = [voltage_V, rc_lost, imu_state, bar_sta, mag_sta]`。源SHA：UART cpp `552C3436…88C3`、hpp `A270C2B1…10EF`、fcstat `1C71EE65…7479`；板上哈希一致，旧版备份位于 `/home/orangepi/diag/backups/*.pre_fc_diag_20261025`。
- 刷写：用户明确授权后，J-Link ARM-OB（STM32F407VG/SWD/1000kHz/VTref 3.30V）写HEX `O.K.`，回读61,364B `Verify successful`，正常复位。刷前完整1MiB备份 `flash_before_fc_diag_20261025.bin`，SHA `AACF7A86…B0ECD`；HEX `A0AE7EF8…D171C`，BIN `995F808E…5CCA9`。
- 实测：`/fc_diag` 20.05–20.07Hz；地面连续值 `volt≈16.12–16.15V, rc_lost=0, imu_state=0, bar_sta=0, mag_sta=0`。`/fc_status` 为 `mode1/unlock0`，四路PWM=0，高度1cm；CSV同列正常落盘。
- 安全边界：已确认无低电压、无RC失联证据；传感器与罗盘状态只记录原始0值，不推断其含义。用户已用万用表复核，确认与 `/fc_diag` 的约16.1V一致，电压通道校准通过。

## 2026-10-25 19:20 20cm扫描候选首测：竖直门过严，已回退
- 数据保存 `flight_data/20261025_scan22_no_laser_selfland/`。22cm/s扫描并未被A3/A2识别阶段严重阻塞（A3/A2均完成采纳），但全程未触发激光；原因是新`AIM_TARGET_DY=-0.10±0.15`与实际帧dy分布不匹配：A3/A2本轮dy基本在+0.10~+0.27，均被拒。
- 已在板上恢复mission `a2eb8e05…a6ef`（过渡稳定0.5s、上一版打点门），保留BVA `f9de8fec…7c21` 的22cm/s扫描候选。mission PID6613、arm PID4728、BVA PID3776 单实例。
- 同轮起飞受理42.51s后、A2后约19.5s核心再次自行`cmd1=6`，从143cm下降；UART仍只有起飞`0x66[1]`，无降落命令。核心自降问题独立存在，和扫描提速/竖直门无因果关系。

## 2026-10-25 19:16 扫描提速/竖直打点候选已部署（待飞）
- 用户要求起飞段完全不动，只加速直线扫描并让位置环更晚接管。BVA候选 SHA `f9de8fec…7c21` 仅改扫描分支：`BVA_SCAN_VFF 17→22cm/s`、新增独立`BVA_SCAN_VMAX_FF=22`、`BVA_FF_NEAR 10→6cm`、`BVA_SHORT_FF_NEAR 6→3cm`。
- 起飞/首段仍用原`vff=20`、`ff_near_nb=18`、`BVA_VMAX_FF=20`；扫描限幅独立，不会抬高起飞段速度。py_compile PASS。
- 打点竖直候选 SHA `7e4c3a78…7b0c`：`AIM_TARGET_DY=-0.10`、竖直窗口±0.15，crossing/window统一按相对目标比较。
- 过渡稳定hold已从5.0s降到0.5s。两份候选均已部署，旧版备份`body_vel_pre_scan22_20261025_1915.py`、`mission_pre_dy_center_20261025_1915.py`；mission PID3945、arm PID3967、BVA PID3776单实例，`/velocity_map`≈48Hz。FC mode3/unlock0/PWM0，等待机手切模式解锁；起飞链不变。

## 2026-10-25 18:46 完整A/B任务分析：稳定停留与打点偏差
- 数据保存 `flight_data/20261025_full_AB_yaw30/`，任务自动飞完、降落并锁定。航点11→12用8.10s，12→13掉头6.10s，13→14稳定点6.71s，14→15 B进-0另有5.39s；其中显式`STABILIZE_HOLD_S=5.0`是可见悬停主因。
- mission候选将`STABILIZE_HOLD_S`5.0→0.5，已部署并重启；预计B面起步少等4.5s。起飞链/arm/autostart未改。
- A面日志6个码均触发激光，但用户目测只命中5个；B面日志实际触发B6/B3/B2/B1四次，用户目测仅命中1个，因此问题是瞄准偏差不是未发送。
- B面触发偏移：B6(-0.07,+0.32)、B3(-0.07,-0.09)、B2(+0.07,+0.33)、B1(-0.03,+0.30)。当前`MARK_DY_TOL=0.35`允许竖直偏差至边缘；B3最接近校准中心，B6/B2/B1竖直偏约0.3。
- 暂不盲调标记阈值；需用户确认A面漏中具体码及B面唯一命中码，以确定是竖直/水平瞄准偏差还是单码图像漂移。部署后FC仍为mode2/locked、PWM0，下次飞前断电重启清核。
- 用户反馈漏点在A左下、B唯一命中在左上；按航线坐标对应A6和B3。日志中A6触发时dx=-0.04/dy=+0.32，B3为dx=-0.07/dy=-0.09；B6/B2/B1虽触发但dy均为+0.30~+0.33。待确认物理光斑是偏高还是偏低后再调瞄准偏置。
- 更正：工程坐标注释明确 A面站立观左侧=y大，故A左下=`A4`；B面左侧=y小，故B左上=`B1`。A4触发(dx=+0.03,dy=+0.29)未中，B1触发(dx=-0.03,dy=+0.30)命中；因此仅凭dy无法完整解释A4/B1差异。用户确认未中光斑整体偏高。
- 本地mission候选SHA `7e4c3a78…7b0c`：新增`AIM_TARGET_DY=-0.10`，竖直门限`0.35→0.15`，crossing/window两条路径统一比较相对目标竖直偏移；py_compile PASS。尚未部署，需换电后再决定是否上板试飞。

## 2026-10-25 18:35 30°/s版本起飞后自降（非旋转触发）
- 数据保存 `flight_data/20261025_takeoff_selfland_after_yaw30/`。UP日志仅有起飞`0x66[1,142]`；自降窗口`ms/mission_step`始终为1，从未出现降落子命令2。
- 起飞受理在18:33:09.27；18:33:11.90 BVA为`航向PD[scan]`，18:33:12.32切`leg`，航向误差约4.9°；未进入rotate。18:33:13.41核心自行报`cmd1=6`，从142cm下降并上锁。
- 因此本轮自降与30°/s自转代码无关，也不是mission发送降落，而是IMU核心在起飞后约4.1s自行进入降落。现有证据不能区分核心保护/内部状态与电池电压跌落；低速自转候选尚未获得有效实飞验证。
- 上一轮45°/s自降和本轮30°/s起飞自降均是核心`cmd1=6`。下一步优先确认是否同一块电池及电压；换满电电池并整机断电重启后再隔离验证。
- 用户确认是满电电池。复核轨迹：`cmd1=6` 前最新高度为142cm（目标142，段内最高142cm），没有超高度；58个连续轨迹样本 `tvd` 全为非NaN，切换前命令仍为 x4.4/y5.8/vz7.5，且F407无“速度断流”状态，只有检测到`cmd1=6`后mission才切active=3发零。高度超限和橙意派速度流断流均无证据。

## 2026-10-25 17:47 回退30°/s并加强自转位置保持（已部署）
- 45°/s组合在约5s自转后触发核心自降；测得该次自转期间水平漂移28.4cm，坐标约(175.9,-17.6)→(150.2,-5.5)cm。
- BVA SHA `15c494e5…3ced`：rotate max45→30、全局vyaw max45→30、自转XY保持 kp0.50→0.80、max3→8cm/s、dead1.0→0.5cm。mission 打点候选保持不变；arm/autostart起飞链未变。
- F407源码恢复`Set_m_speed()` yaw限幅30°/s，Keil errors=0/warnings=0；J-Link烧录HEX `a98884b1…6f00`，BIN回读61248B `Verify successful`。刷前45°版完整Flash备份`flash_before_yaw30_20261025.bin`。
- 当前 mission PID7916、arm PID7938、BVA PID7837 单实例，等待START；FC mode1/locked、PWM0，但IMU核心仍保留`cmdCID=16/cmd1=4`，下次飞前需整机断电重启。

## 2026-10-25 17:42 45°/s自转后核心自降（证据）
- 数据已保存 `flight_data/20261025_selfland_after_rotation/`。UP 日志只有起飞 `0x66[1,142]`；无 `0x66[2]`，mission 未到终点，因此不是香橙派任务发出的降落。
- 自转约从 t1328.2 开始，FC 回显 `yaw_dps=45`；航向从约1°到172°约5s，随后 t1333.3 飞控无新上层命令自行报 `cmd1=6`，开始从104cm下降并上锁。mission 只是随后检测到 `cmd1=6` 并中止。
- 这是首次把飞控外部偏航指令从5/30提升到45°/s，且自降紧跟在高速自转结束处；旧5°/s自转26s未自降。核心保护逻辑未知，不能证明45°/s是唯一根因；45°/s连续原地自转当前判为不安全，不应继续复飞该组合。
- 当前 F407 yaw上限45、BVA rotate max45 均在板上。再次飞行前需改为安全方案：回30°/s并把部分转角分摊到过渡段，或由用户确认其他处理。

## 2026-10-25 17:37 换电重启后候选就绪（待飞）
- 新 boot `131315de…3c4`；mission/BVA/arm/autostart 哈希与 17:34 候选一致。mission PID2980、arm PID3002、BVA PID1914 单实例，等待 START。
- FC 已清回 `mode_sta=2/unlock=0`、`cmdCID=0/cmd1=0`、PWM0；`/velocity_map`≈50Hz，`/mark_ok` 1发布/1订阅。
- 本轮待验证：180°自转是否≤5s、首码前是否连续不早停、A1–A6打点触发率和位置偏差。

## 2026-10-25 17:34 自转/首码/打点候选已刷写部署
- 用户授权烧写后，J-Link 写入 F407 `45°/s` 偏航限幅候选：`O.K.`，BIN 回读 61248B `Verify successful`，已复位。刷前完整备份 `flash_before_yaw45_20261025.bin`（1MiB）。
- 香橙派已部署 BVA `bbe0a05b…38bf` 与 mission `3f7a866c…d767`；备份为 `backups/body_vel_pre_speed_yaw45_20261025_1732.py`、`backups/mission_pre_speed_mark_20261025_1732.py`。
- 候选内容：自转环不再依赖 `no_brake`，最大45°/s；扫描恢复17cm/s；首段交回位置环28→18cm；`A进-0` hold1.0→0；过线帧最大间隔0.60→1.20s。起飞 arm/autostart 未改。
- flight-link active，新 BVA PID9128；mission PID9210、arm PID9246 单实例，等待 START；`/velocity_map`≈48Hz。FC 当前 `mode1/locked`、PWM0，但 IMU 核心仍记 `cmdCID=16/cmd1=4`；正式试飞前整机断电重启清除核心旧状态。

## 2026-10-25 17:27 A面后自转人工降落轮分析（候选未上板）
- 本轮数据 `flight_data/20261025_speed_opt_manual_land/`：A 面扫描完成，人工在自转中途降落。自转约26s仅转135°，平均约5.2°/s。
- 自转慢根因：BVA `no_brake` 在 A→B 过渡时仍为 false，快速 rotate 环未启用，实际使用 scan 航向环 `max=5°/s`。日志中 BVA 计划 `vyaw=10`，最终输出被压到5；飞控 0x41 回显也为5。
- 首码前停早根因：首段前馈 `ff_near_nb=28cm`，20cm/s巡航用时过短，惯性阶段约在距目标10–14cm处停住，再由低增益位置环收口。候选取18cm，并令 `A进-0` 的1s hold归零，连续进入A3。
- 打点：本轮只有A1/A6实际触发；A3的跨线候选间隔约1.04s，被 `MARK_CROSS_GAP_S=0.60` 拒绝；扫描20cm/s使有效二维码帧不足。候选取扫描速度回17cm/s，cross gap 1.20s。
- 本地候选：BVA SHA `bbe0a05b…38bf`（rotate max45、scan17、ff_near18、去掉rotate的no_brake门），mission SHA `3f7a866c…d767`（gap1.2、A进-0 hold0），py_compile PASS。尚未部署。
- 45°/s仍会被F407 `Set_m_speed()`的30°/s限幅截断；已将F407限幅候选改为45并Keil编译通过（errors=0/warnings=0），HEX SHA `bc3e389b…b741`，原 `ANO_LX.c` 备份 `ANO_LX.c.bak-20261025-yaw45`。尚未刷写。

## 2026-10-25 17:17 飞行段速度优化已恢复（待飞）
- 起飞链完全冻结：mission `44148bcf…5e63a`、arm_trigger `42277656…ee5b8d`/`0.8s`、autostart `c36e3c90…bb000` 均未改。
- 仅把 BVA 恢复到速度优化版 SHA `6446e8e5…ccb46`：首段/过渡前馈 `20cm/s`、扫描前馈 `20cm/s`、≤25cm 短段 `ff_near=6cm`、自转 PD `0.80/0.20` 最大 `15°/s`、自转时 XY 有界保持 `kp0.50/max3cm/s/dead1cm`，扫描航向 `kd0.28`。旧 BVA 已备份 `backups/body_vel_pre_speed_optimized_20261025_1711.py`。
- 通过终止 flight-link MainPID 触发 systemd 重启，新 BVA 已加载；随后因整机换电重启，autostart 单实例拉起 mission PID3031、arm PID3067。FC `mode1/locked`、`cmdCID=0/cmd1=0`、PWM0；`/velocity_map`≈49.5Hz，`/mark_ok` 1发布/1订阅。
- 未解锁、未起飞。下一轮只验证上述飞行段速度，不再改起飞链。

## 2026-10-25 17:12 起飞链冻结约束
- 用户明确要求：起飞链条不再修改。保持当前首次双面版本：autostart 默认 `0.8s`、arm_trigger 无预挂/无新时序改动、mission `44148…` 的 START→`0x66` 路径。
- 后续仅优化起飞完成后的航段速度与姿态控制，优先检查 BVA 的首段/过渡/扫描速度、减速和自转参数；不调整解锁保持、START 触发、mission 起飞前门控或 `0x66` 时序。

## 2026-10-25 17:10 起飞回归原因收敛（分析）
- 撤回“新版预挂 pos_hold 是回归原因”：旧 mission `44148` 本身在 `__init__` 调用 `_ensure_pos_hold()`，首次成功飞行解锁时 `/target_velocity` 也已是零速流。两版在地面都有该流，不能解释失败。
- mission `ba97` 相对 `44148` 只多 cmd1 稳定门；BVA `6446` 相对 `6d470` 只改已收到航线目标后的航向/速度/自转。均不参与解锁到 START 的地面阶段。
- 真正影响起飞窗口的代码回归是 autostart 把 arm_trigger 保持从 `0.8s→2.5s`，使 START 与 `0x66[1,142]` 整体晚 1.7s。首次版本 START→`0x66` 已需 3.894s，核心在等待有效起飞命令期间会自行进入 `cmd1=6` 降落/上锁；橙意派日志不存在任何 `0x66` 发送记录。
- 判断：不是新代码主动发降落，而是新代码把有效起飞命令推迟到核心“已解锁但未起飞”的短窗口之外；外部 `/target_velocity=0` 不能替代 `0x66` 起飞命令。核心闭源内部超时阈值未知，此结论基于版本差异和完整时序，不是核心源码证明。

## 2026-10-25 17:03 香橙派回退到首次双面飞行版本（待飞）
- 按用户要求回退：mission SHA `44148bcf…5e63a`、BVA SHA `6d470f08…f501`、arm_trigger SHA `42277656…ee5b8d`（解锁保持 0.8s、无预挂 pos_hold）、autostart SHA `c36e3c90…bb000`（保留 IMU/odom 启动修复）。
- 回退前当前版本已备份为 `/home/orangepi/diag/backups/*_pre_first_ab_rollback_20261025_1659.*`；四文件均通过 py_compile / bash -n。
- 因 sudo 无免密权限，通过终止 flight-link MainPID 触发 systemd `Restart=always` 重载；服务 active，新 BVA PID24662。任务 PID23878、arm_trigger PID23900 单实例，mission 等 `/gs/start`，arm 使用 `0.8 3600 start`。
- 链路验证：`/velocity_map` ≈50Hz，`/mark_ok` 1 发布/1 订阅。最近 FC `mode2/unlock0`、PWM0，但 `cmdCID=16/cmd1=4` 保留了刚才失败尝试的状态；正式飞前应先整机断电重启，恢复干净 `0/0`。

## 2026-10-25 16:49 32端待起飞请求修复已刷写并校验
- 用户明确授权后，通过 J-Link ARM-OB（STM32F407VG、SWD 1000kHz、VTref 3.3V）烧录 `ANO_LX.hex`；J-Link 报告 Flash 编程 65536B、`O.K.`，随后 `verifybin` 回读 61248B 全部 `Verify successful`，正常复位运行。
- 刷前完整 Flash 已备份为 `flash_before_20261025.bin`（1MiB，SHA `ed64f434…1333`）；待写 HEX SHA `a98884b1…6f00`、BIN SHA `c6d3d7fa…1609`。暂存副本 `firmware_ANO_LX.hex/.bin`。
- 刷后确认 F407 通信恢复：`mode1/locked`、`cmdCID=0`、`cmd1=0`、油门350、PWM0；mission/arm_trigger 仍在等待，未发 START、未解锁、未起飞。飞行效果和旧 pending 清除行为仍待实机验证。

## 2026-10-25 16:44 32端待起飞请求隐患修复（已编译，未刷写）
- 修改 Keil 工程 `C:\Users\金洲毅\Desktop\学习资料\国一32飞控代码\FcSrc\AnoDTRasp.c`，原文件 SHA `cc37337b…3f89`，备份 `AnoDTRasp.c.bak-20261025-1645`；新源码 SHA `9482724e…cc75`，工作区 UTF-8 对照副本 `fc407_takeoff_guard/AnoDTRasp.c`。
- 隐藏状态修复：收到 `0x66[2/3/4]` 立即取消待起飞；操作手由上锁沿 disarm 时取消；0x66 起飞只允许在 `unlock_sta=1` 且无待结束动作时入队；重试耗尽后清空全部 pending 并关闭 `takeoff_if/flown_flag`。删除失败后自动调用 `FC_Unlock()` 的路径，重新上锁只能由操作手执行。
- Keil `Ano_LX` 构建通过：errors=0、warnings=0，flash 61652B、RAM 9872B；最终 HEX `C:\Users\金洲毅\Desktop\学习资料\国一32飞控代码\ProjectSTM32F407\build\ANO_LX.hex` SHA `a98884b1…6f00`，AXF 同目录。构建日志 `.embeddedskills/build/ANO_LX_STM32F407-Ano_LX-build.log`。
- 当时验证范围仅 L2 静态构建；后续已按用户明确授权刷写，见上节。

## 2026-10-25 16:36 与首次双面飞行对比：核心在 START 前已自行升 PWM
- 首次双面飞行（`flight_data/20261025_full_AB_manual_land/`，15:19）：解锁前 `cmdCID=0/cmd1=0`；解锁后 PWM 保持 18%，START 后约 4 秒发 `0x66[1,142]`，0.2 秒内 `cmd1→5` 并正常上升。起飞段自动触发保持为 0.8s。
- 修复后 16:32 复测：解锁前 `cmdCID=16/cmd1=4`；16:32:53.66 解锁并保持 18%，16:32:57.47 在**尚未发送新 START/0x66**时 PWM 自行升到约 40%，16:33:00.36 才收到 START，16:33:00.51 已报 `cmd1=6`。因此 2.5s 保持和 pos_hold 不是本次自升 PWM 的根因。
- 当前任务 `ba97…` 相对首次双面 `44148…` 只增加 cmd1 稳定门；BVA 新差异只作用于已收到航线目标后的扫描/自转。真正明显差异是 F407 本次解锁前保留 `cmdCID=16/cmd1=4`，而首次飞行前为全零。
- 固件参考 `fc407_patch/AnoDTRasp.c:68-76` 的 `mission_tk_req` 只能由 `OneKey_Takeoff()` 成功清除，无超时，后续 lock/land 不清除；这是待验证的粘滞起飞请求机制。即使 pos_hold 持续发零速，16:32 仍自行升 PWM，支持但尚未证明该 pending 状态为根因。
- 下一步：用户确认是否真正切断 F407/IMU 核心电源；重新上电后先只读核对 `cmdCID/cmd1` 是否回到 `0/0`，未归零前不再解锁。

## 2026-10-25 16:32 解锁保持期零速流修复（已部署，待重新解锁）
- 现场仍怠速的直接原因：最新解锁窗口 `335.21s mode2/unlock1` 后约 `0.65s` 核心报 `cmd1=6`，到 `337.36s` 已自锁；2.5s 保持期间没有任何真实 `/target_velocity`。`/active_controller=3` 不能保流，因为 C++ 位置环没有 `/target_position` 时在 `pid_controller.cpp:455` 直接 return。
- arm_trigger 改为在待机时确保 `pos_hold` 常驻，并要求 `/target_velocity` 在 1.0s 内新鲜后才开始 2.5s 解锁保持；本地及板上 SHA `a5c894dc8e57884cbec46a155853ba051a95297392c7f0fdf474d65d9008e2a3`，板上备份 `/home/orangepi/diag/backups/arm_trigger_pre_poshold_20261025_1630.py`（SHA `42277656…ee5b8d`）。
- 只重启 arm_trigger；新 PID10881，pos_hold PID4411，mission PID3597、BVA PID1903 未动。实测 `/target_velocity=[0,0,0,0]`，arm 日志等待 CH5=2；最近 fcstat 仍 `mode1/locked/PWM0`。未发 START、未解锁。

## 2026-10-25 16:23 解锁后保持 2.5s（已部署）
- autostart 新增 `ARM_HOLD_S=2.5`，arm_trigger 仅在 mode2、unlock1、cmd1≠6、油门达标这些条件全部成立并持续2.5s后发送 `/gs/start`；可通过环境变量 `ARM_HOLD_S` 单独调整。
- autostart SHA `a882d479ac63240f1e2b9f57152a7870fea48b9f6d4b1313806a9a22b7969dea`，旧版备份 `autostart_pre_armhold25_20261025_162202.sh`。arm_trigger 新 PID6755 参数为 `2.5 3600 start`，autostart PID6544；mission 仍单实例 PID3597，FC mode1/locked/PWM0。

## 2026-10-25 16:18 cmd1 稳定门已部署并重启（待飞）
- 新 boot `e600327c-573c-4e76-aa14-83e9f9de08e4`；mission 已部署 `ba97ad09ae573db2fe9a151b0630c2d66bb41102219355eb2940e122f2783490`，旧版备份 `mission_pre_cmd1_quiet_20261025_161814.py`；BVA 保持 `6446e8e5…ccb46`。
- autostart 拉起 `imu_pub` PID3206、`fc_odom` PID3209、完整 mission PID3597、arm_trigger PID3755、autostart PID3182。`/velocity_map` 约50Hz，`/mark_ok` 1发布/1订阅；mission 等 START，arm 等 CH5=2，FC mode1/locked/PWM0。
- 起飞逻辑现要求：若看到 cmd1=6，必须在最后一次6之后连续2.0s为4才允许发0x66；不会再只等0.5s就发。

## 2026-10-25 16:14 起飞前 cmd1 冷却逻辑修正（本地候选，未部署）
- 本次异常时序：16:13:30.998 核心回报 `cmd1=6`，mission 于16:13:31.522 判定复位并在16:13:31.531 发送 `0x66[1,142]`，实际仅等待约0.52s；随后核心 `cmd1` 保持4、未进5，mission中止。PWM曾约42%，回落约18%，油门约512。
- 原因：原注释写“等2s”，代码却在 `cmd1!=6` 的第一帧直接 break，所以只避开瞬跳尾部，没有等核心状态机真正稳定。不是 BVA 速度参数导致；autostart/新任务让自动 START 更早命中该窗口。
- 修正：仅当看到 cmd1=6 时，以“最后一次6之后连续2.0s均为4”为放行条件，最多观察5s；超时或解锁掉线则不发0x66。本地 mission SHA `ba97ad09ae573db2fe9a151b0630c2d66bb41102219355eb2940e122f2783490`，py_compile + 82项检查PASS，尚未部署。
- 安全：`fc407_patch/AnoDTRasp.c` 的 `mission_tk_req` 在起飞失败后不会清、无超时；当前旧请求状态需整机断电重启清除。复测前先断电，再部署新 mission 并验证。

## 2026-10-25 16:13 怠速加速/起飞未受理检查（安全暂停）
- 现场 fcstat：解锁后油门约512，PWM 曾升至约42%并回落到约18%；`cmd1` 未进入5。UART 日志确认香橙派已发送 `0x66[1,142]`（`Sent 0x66 mission cmd`），但核心回报 `cmdCID=16, CMD1=4`，1.5s 内无 `cmd1=5`，mission 已中止。
- 源码风险：`fc407_patch/AnoDTRasp.c` 收到 0x66 起飞后置 `mission_tk_req=1`，仅 `OneKey_Takeoff()` 成功才清零；mission 中止或后续 land/lock 命令不会清该 pending，且无超时。存在核心稍后接受旧起飞请求的风险（待刷写版本实际验证）。
- 因此这轮不要继续用旧 F407 状态起飞；先整机断电重启清零 pending。若需修源码，应只在 `mission_tk_req` 上增加取消/超时并在地面验证，刷写需用户明确批准。

## 2026-10-25 16:13 再次上电，最新版与雷达链正常
- 新 boot `fc591f62-3d9c-4c8e-ba7c-696f23f23bfb`；mission/SLAM 修复后 BVA/autostart 哈希分别为 `44148bcf…5e63a` / `6446e8e5…ccb46` / `c36e3c90…bb000`。
- autostart 已启动 `imu_pub` PID2978、`fc_odom` PID2981、完整 mission PID3098、arm_trigger PID3135、autostart PID2954。`/velocity_map` 实测约50Hz；mission 等 `/gs/start`，arm 等 CH5=2；FC mode1/locked/PWM0，未发 START/解锁。

## 2026-10-25 16:02 最新版恢复（含 IMU/odom 启动修复）
- 保持 mission SHA `44148bcfc3ae59bc92d669429a849811fad0f7d1d57b6d946893f672b5b5e63a`，BVA 恢复最新候选 SHA `6446e8e5b7003a9d30343b365bbf0788389acfba5d8d22a487611d3b0e1ccb46`；回退版 `6d470f08…f501` 另存 `body_vel_pre_latest_restore_20261025_160221.py`。
- autostart 已恢复 IMU/odom 启动块，SHA `c36e3c90…bb000`；`imu_pub` PID14258、`fc_odom` PID14261 均运行。重载后 `/velocity_map` 约49Hz。
- 当前单实例：完整 mission PID11061、BVA PID16829、arm_trigger PID8976、autostart PID14234。mission 等 `/gs/start`，arm 等 CH5=2，FC mode1/locked/PWM0。
- 最新 BVA 包含扫描 `kd0.28`、扫描/过渡 `vff20`、短段 `ff_near6`、180°自转 max15°/s，以及自转期间锁进入位置的低速 XY 保持。

## 2026-10-25 15:59 雷达速度链修复（IMU/odom 启动块恢复）
- 根因确认：`/scan` 约14.6Hz正常，但 Cartographer 持续报 `Queue waiting for data: (0, imu)`，`/velocity_map` 与 `/velocity_map_filtered` 均无消息。`imu_pub.py`、`fc_odom.py` 均无进程。
- 原因：15:15 部署 full-route autostart 时漏掉了原脚本顶部的传感器桥启动块。已从 `backups/autostart_pre_fullroute_20261025_151530.sh` 恢复 `imu_pub.py` 和 `fc_odom.py --rate 20`，新 autostart SHA `c36e3c90…bb000`，旧退化版备份 `autostart_pre_imu_restore_20261025_155906.sh`。
- 恢复后实测：`/imu` ≈200.5Hz，`/velocity_map` ≈50Hz，`/velocity_map_filtered` ≈30Hz；Cartographer 不再等 IMU。rollback 任务仍单实例：mission PID11061、BVA PID8598、arm_trigger PID8976、autostart PID14234；mission/BVA 保持上一轮实飞 SHA `44148bcf…5e63a` / `6d470f08…f501`。
- 当前 FC `mode_sta=1/unlock=0`、PWM0；待用户再次拨 CH5=2 验证核心是否接受。

## 2026-10-25 15:55 回退到上一次实飞代码
- 按用户要求回退到上一轮已实飞组合：mission SHA `44148bcfc3ae59bc92d669429a849811fad0f7d1d57b6d946893f672b5b5e63a`，BVA SHA `6d470f085f2379e1218ac26750129bba1b7a30727f1ceac5c140f4fd71f7f501`。当前尝试中的 BVA `6446e8e5…` 已备份到 `/home/orangepi/diag/backups/body_vel_rotate_hold_20261025_155235.py`。
- BVA 从已验证备份 `body_vel_pre_rotate_hold_20261025_153644.py` 恢复，板上 py_compile 后重载 flight-link；清理了一次 autostart/手动同时拉起造成的重复 mission，最终单实例 mission PID11061、BVA PID8598、arm_trigger PID8976、autostart PID11363。
- 完整 12 货位 mission 等 `/gs/start`，arm 等 CH5=2；此前 FC 实际 mode1/locked、PWM0。回退不等于解锁故障已解释；若回退后仍无法进入模式2，继续按 FC mode_cmd/mode_sta 与橙香派控制话题隔离排查。

## 2026-10-25 15:43 断电重启后链路恢复（完整 A/B 待飞）
- 新 boot `bec4d1f7-7385-4f73-ba81-da86d616b13f`；mission SHA `44148bcf…5e63a`、BVA SHA `6446e8e5…ccb46` 均正确。autostart 默认空 subset，已拉起完整 12 货位 mission PID3225、arm_trigger PID3248、autostart PID2986、BVA PID1900。
- mission 等 `/gs/start`，arm 等 CH5=2；`/mark_ok` 1 发布/1 订阅，FC mode1/locked/PWM0。未发 START/解锁。

## 2026-10-25 15:38 自转位置锁定版已部署（待全任务复飞）
- 电池后上电 boot `45ad09ab-a8af-499a-af4a-eef4cdc9e42d`。BVA 候选 `6446e8e5b7003a9d30343b365bbf0788389acfba5d8d22a487611d3b0e1ccb46` 已部署并 py_compile；旧版备份 `/home/orangepi/diag/backups/body_vel_pre_rotate_hold_20261025_153644.py`。
- 本次控制候选：扫描 `vff20`、过渡 `vff20`、短段 `ff_near6`、扫描航向 `kd0.28`；自转 `max15°/s`，并在进入自转时锁当前 XY，以 `kp0.50/max3cm/s/deadband1cm` 做平滑回漂修正，不再追原过渡点。
- mission 保持完整 A/B 连续旋转版 SHA `44148bcf…5e63a`；autostart 默认空 subset。BVA PID4790、完整 mission PID5060、arm_trigger PID5216、autostart PID4954；mission 等 `/gs/start`，arm 等 CH5=2，`/mark_ok` 1 发布/1 订阅，FC mode1/locked/PWM0。
- 本地 py_compile + 81 项检查 PASS。尚未发送 START/解锁；飞后按固定流程分析航向、A/B 打点、过渡速度、自转角度收敛和自转期间 XY 漂移。

## 2026-10-25 15:22 完整 A/B 飞行分析（BVA 候选仅本地，未部署）
- 本轮完整任务日志显示：A1–A6 都完成打点；到达 `过渡-到达B面侧` 后开始 180° 自转，但人工降落前仅从约 1.3° 转到 132.2°（26.74s，平均 4.90°/s），未进入 B 面。主 trace 与全部辅助日志已保存到 `flight_data/20261025_full_AB_manual_land/`，分析脚本同目录。
- 航向指标：A 上排 RMS 2.40°、峰峰 9.1°、主频 0.39Hz；A 下排 RMS 2.25°、峰峰 10.7°、主频 0.44Hz。横向 x 上排 RMS 5.69/峰值12.94cm、下排 RMS 2.63/峰值5.53cm。下一档仅将扫描 `kd 0.25→0.28`，Kp=0.50/max5 不变。
- 首码时序：START→A进-0 18.89s；A进-0→A3 3.10s；A进-0→首次 A3 激光 2.93s。首码段短，原扫描 `ff_near=10cm` 几乎全交慢位置环；候选对 ≤25cm 短段使用 `ff_near=6cm`。
- 过渡速度：A出-1→过渡-离开板 88cm/5.70s≈15.4cm/s；过渡-离开板→到达B面侧127cm/8.20s≈15.5cm/s，均被 `vff=15` 限住。候选将首段/过路和扫描前馈统一提至20cm/s。
- 自转慢的主因是航向环上限：C++ 输入和 BVA 输出都在约5°/s。候选新增仅在 `no_brake + |err|≥30°` 启用的 rotate PD（kp0.80/kd0.20/max15°/s），误差≤3°退出，全局 vyaw 上限临时10→15°/s。
- 自转期间 XY 不追原过渡点，而是在进入 rotate 模式时锁住当前位置，再做有界 P 保持：`kp=0.50/max=3cm/s/deadband=1cm` 并经过加加速度平滑；避免旧目标在身后导致大反向、避免航向与位置同时满权抢控制。航向差≤3°后解除锁定。
- BVA 本地候选 SHA `6446e8e5b7003a9d30343b365bbf0788389acfba5d8d22a487611d3b0e1ccb46`；mission 保持 `44148bcf…5e63a`。本地 `py_compile` + 81 项检查 PASS；电池充电中，尚未部署/重启/实飞。

## 2026-10-25 15:22 完整 A/B 手动落地数据已保存
- 用户手动降落，落地核对 FC mode1/locked、PWM0。本轮完整任务原始数据已复制到 `flight_data/20261025_full_AB_manual_land/`：主 trace `rec_20261025_141704-1.log`（4802136 bytes，SHA `7c5d6818…099e0`）、`mission.log`、`wpn.log`、`qr.log`、`bva.log`、`fcstat_pwm.csv`、`route_mission.yaml`、mission CSV 与 autostart/arm 日志。
- 本轮因为人工降落没有走 mission 自动结束/自动降落流程；后续分析需按实际 waypoint/QR 记录判断完成到 A/B 哪一段，不把“手动收尾”写成任务自动完成。
- 换电后可安全断电；下轮上电先复核 boot、BVA/完整 mission 单实例和 FC locked/PWM0，再按固定流程分析航向 RMS/峰峰/主频并小步调参。

## 2026-10-25 15:18 完整 A/B 双面任务（已部署，待飞）
- 完整任务已从 A1–A6 切换到整条 12 货位；autostart 默认 `SUBSET=`，断电重启也默认完整任务。板上 `--check` 干跑顺序：起飞→A进-0→A3→A2→A1→A出-0→A进-1→A4→A5→A6→A出-1→过渡-离开板→过渡-到达B面侧→过渡-掉头→过渡-稳定→B进-1→B4→B5→B6→B出-1→B进-0→B3→B2→B1→B出-0→land；12 货位 + 12 过渡点。
- 过渡自转改为单个 `ψ0+180°` 连续目标，不再拆 45/90/135/180 四段；掉头航向容差仍 3°，随后“稳定”点保留 5s 确认，再进 B 面。B 面复用 A 面同一 `face_pass`、瞄准窗/过线打点、pending 和连续扫描逻辑。
- mission SHA `44148bcfc3ae59bc92d669429a849811fad0f7d1d57b6d946893f672b5b5e63a`，旧版备份 `mission_pre_continuous_rotation_20261025_151743.py`（旧 `abf6e917…4061a`）；BVA 保持 SHA `6d470f08…f501`。autostart SHA `6288ba58…92eb`，旧版备份 `autostart_pre_fullroute_20261025_151530.sh`。本地 `py_compile` + 71 项检查 PASS。
- 当前 boot `23452956-e04d-44fa-a1dd-a93c113ecf42`；BVA PID1914、完整 mission PID8406、arm_trigger PID4364、autostart PID8557。mission 等 `/gs/start`，arm 等 CH5=2，FC mode1/locked/PWM0；未发 START/解锁。
- 飞后固定先分析航向 RMS/峰峰/主频与横向误差，再只调一档航向参数；同时核对 A/B 打点、过渡旋转和降落。

## 2026-10-25 15:05 连续扫描恢复（已部署，待飞）
- 后续每轮实飞结束固定流程：先取主 trace 计算航向 RMS/峰峰/主频与各段横向误差，再只调一档航向参数；其他已确认问题同步修，未验证项不写成已解决。
- 最新一轮六码均已打中，但 `CELL_HOLD_S=0.5` 与 BVA 每航点 0.6s 到点停叠加，形成可见驻留；A3→A2 约 11.4s、A2→A1 约 8.1s。现 `CELL_HOLD_S=0`，并删除 BVA `/route_reached_index` 到点停；打不到由 pending 后台继续等，不阻塞飞下一格。
- 首码前短段原 `ff_near=14cm`，在约 15cm 入点→A3 上几乎全程位置环；现扫描 `ff_near=10cm`，前馈保留更多，缩短首个二维码前的收口时间。
- mission SHA `abf6e917d2c9db63be1af7d5b1c38e9d856ceb88deaa80f7fce0bf50e034061a`，备份 `mission_pre_continuous_scan_20261025_150526.py`（旧 `892e38db…24823b`）；BVA SHA `6d470f085f2379e1218ac26750129bba1b7a30727f1ceac5c140f4fd71f7f501`，备份 `body_vel_pre_continuous_scan_20261025_150526.py`（旧 `1ca32fe5…e0b6`）。本地 `py_compile` + 70 项检查 PASS。
- 2026-10-25 15:09 换电重启 boot `23452956-e04d-44fa-a1dd-a93c113ecf42`；autostart 拉起 BVA PID1914、mission PID3278、arm_trigger PID3300；mission 待 START，arm 等 CH5=2，`/mark_ok` 1 发布/1 订阅，FC mode1/locked/PWM0；未发 START/解锁。

## 2026-10-25 14:57 上排瞄准窗兜底 + 每航点到点停（已部署，待飞）
- 上一轮实飞只有 A4/A5/A6 打中：上排 A3/A2/A1 的 QR `dx` 最好仅到 `+0.03～+0.11`，始终没有跨过 `AIM_TARGET_DX=-0.06`，因此纯 crossing 判据整排漏打；A2/A1 随后在 8s TTL 到期。新增 `MARK_WINDOW_TOL=0.22` 窄瞄准窗兜底，仍要求同货位同码、新鲜 `/mark_ok`、speed ≤20cm/s；过线+额外偏移逻辑保留。
- 直线不稳证据：同一 y 向直线段内 x 在 52.9～65.0cm 间摆动（目标约 57cm），说明位置环/横向纠偏在滞后位姿上过冲，并非纯前馈指令本身反向。扫描横向纠偏 `kp 0.30→0.20`、`max 4→3cm/s`；每个空中航点到达后 XY 停 0.6s，Z/航向继续控制，避免过点后反向追旧目标。首段速度仍 15cm/s、提前 28cm 收口。
- 航向继续上一档小步：扫描 `kp=0.50`、`kd=0.25`、`max=5°/s`。
- mission SHA `892e38db06ff207af7b7abee3e723b436ad55b6e190cef9ad1e357d86c24823b`，备份 `mission_pre_window_fallback_20261025_145647.py`（旧 `bcb52eb1…ac4178`）；BVA SHA `1ca32fe5dd3587fbcd48de245d1e6a92831718584bd68df6ec2189f3cdd3e0b6`，备份 `body_vel_pre_window_fallback_20261025_145647.py`（旧 `720d7253…a93b3`）。本地 `py_compile` + 70 项检查 PASS；证据 `flight_data/20261025_A_tuning3/`。
- 已在地面锁定/PWM0 下重启 flight-link 与任务链：BVA PID8867、mission PID9796、arm_trigger PID9240；mission 待 `/gs/start`，arm 等 CH5=2，`/mark_ok` 1 发布/1 订阅，FC mode1/locked/PWM0。未发 START/解锁。

## 2026-10-25 14:50 过线后偏置打点 + 首段低速到点停（已部署，待飞）
- 上一轮实飞：航向继续改善；A2/A4/A5/A6 打中，A3/A1 因旧 5s pending TTL 先过期而漏打（A1 从采纳到过线约 6.1s）。过线仍要求同货位同码、横向中心 ±0.12、纵向 ±0.35、speed ≤20cm/s；现过线后沿运动方向再前进 `MARK_CROSS_EXTRA_DX=0.02`（约 0.5–2cm，取决于尺度），等待窗口 0.80s，pending TTL 5→8s。
- 首段 90cm：删除 near-target 回拉锁存，前馈速度 20→15cm/s，提前 28cm 交回位置环；新增 `/route_reached_index` 单次到点停 1.0s，停 XY、Z/航向继续正常控制，然后再响应下一目标。
- 扫描航向继续小步阻尼：`kp=0.50`，`kd 0.22→0.25`，`max=5°/s` 不变。
- mission SHA `bcb52eb1527a124ba74d31e8fef054e9f81044639a47e6cb01b998ec85ac4178`，备份 `/home/orangepi/diag/backups/mission_pre_aftercross_stop_20261025_144657.py`（旧 SHA `cdb20357…c88841`）；BVA SHA `720d7253a42f0ed43e83186f5a0426e59749e74bd013327c1d0b6797093a93b3`，备份 `/home/orangepi/diag/backups/body_vel_pre_aftercross_stop_20261025_144657.py`（旧 SHA `ec839ac6…b5ac9a`）。
- 本地 `py_compile` + 68 项检查 PASS；原始证据 `flight_data/20261025_A_tuning2/`。2026-10-25 14:50 换电重启 boot `f954f3a2-496a-413c-907a-e701f2250835`，autostart 拉起 BVA PID1904、mission PID3269、arm_trigger PID3291；mission 待 `/gs/start`，arm 等 CH5=2，`/mark_ok` 1 发布/1 订阅，FC mode1/locked/PWM0；尚未 START/解锁。

## 2026-10-25 14:20 过线短暂重试 + 航向阻尼 + 近点防回拉（已部署并启动）
- 2026-10-25 14:33 重新上电 boot `230de8c2-306b-4a88-a47b-74727b880bde`；autostart 已拉起 BVA PID1903、mission PID2832（A1–A6）和 arm_trigger PID3036；`BVA_FF=1`、无旧 yaw/近点环境覆盖，`/mark_ok` 1 发布/1 订阅。mission 等 `/gs/start`，arm_trigger 等 CH5=2；FC mode1/locked/PWM0，本步未发 START/解锁。
- 本轮 A1/A5 均为几何合格后 `_mark_stable()` 瞬时拒绝：A1 在 7592.730 过线时 gyro raw=-121（-6.6°/s）、航向误差约 +3.2°；A5 在 7611.223 前 0.1–0.3s 的 gyro raw 为 174–304（9.5–16.5°/s），过线帧降至 109（5.9°/s）。`/target_velocity_nav` 与 `/mark_ok` 均为约 50Hz，freshness 非首要嫌疑；A1 因 `/mark_ok` 只有 Bool，无法从存档再拆瞬时 gyro/航向哪一项。不扩 gyro/航向阈值，新增过线候选 0.30s 重试，仍须同货位同码、横带 ±0.12、纵向 ±0.35、speed ≤20cm/s，保护重复触发。
- mission 候选 SHA `cdb2035733f46d184baa8252de1b06fd1cd19760d84ffc03cc20694034c88841`；板上已替换并通过 py_compile。旧版备份 `/home/orangepi/diag/backups/mission_pre_mark_retry_20261025_141259.py`（SHA `4b5c4169…1a8d751`）。
- 本轮连续位姿：上排扫描航向 RMS 3.19°、峰峰 13.6°、主频约 0.32Hz；下排 RMS 3.04°、峰峰 13.3°、主频约 0.38Hz。扫描 yaw 由 `kp0.60/kd0.16/max5` 改为 `kp0.50/kd0.22/max5`，只做降比例、提阻尼，不改速度/横偏。
- 90cm 首段（起飞点→A进-0）：实飞沿原航向越过目标约 10cm，随后因位置环恢复而折返；首段前馈交回距离 `20→28cm`，巡航速度仍 20cm/s。近点回拉另改为一旦过点且开始离目标运动就锁存到目标切换；A1/Aout0 附近沿新航向最小速度约 -4.6/-5.2cm/s。
- BVA 候选 SHA `ec839ac6420ddd002f612ceefc13af1a68791306a514820b45b25b4fa4b5ac9a`；板上已替换并通过 py_compile。原始版备份 `/home/orangepi/diag/backups/body_vel_pre_yaw_near_latch_20261025_142010.py`（SHA `98b581b8…142c8f`），仅加 yaw/近点锁存但未加 90cm 提前交回的中间版备份 `/home/orangepi/diag/backups/body_vel_pre_firstleg_handoff_20261025_142459.py`（SHA `90efc140…37cbf3`）。
- 本地 `py_compile` + 70 项检查 PASS；原始证据 `flight_data/20261025_A_mark_retry_tuning/`。上电后已由 autostart 启动新代码，尚未 START/解锁。

## 双高度方案定稿：最低高度放行（2026-10-24 晚）
- 2026-10-25 13:48 打点改为“码经过激光中线”：同一货位/同一码的连续 QR 帧，`dx` 从 `AIM_TARGET_DX=-0.06` 一侧跨到另一侧才触发；横向两帧均须在 `±0.12`，纵向仍为 `±0.35`，并要求新鲜 `/mark_ok` 且速度 `<=20cm/s`。不再在首次绑定或宽瞄准窗边缘直接打。QR 发布冷却从 2s 降到 0.12s；板载 `detect_every=2 + confirm_frames=2` 下约 5Hz 提供过线帧。
- mission SHA `4b5c416921606918724f815fa5ddd0abcccc0d6dd605471d257fb3dea1a8d751`，本地 `py_compile` + 50 项检查 PASS；已部署到 `/home/orangepi/diag/mission_node.py`，旧版 `dc2340e3…f2b789` 备份在 `/home/orangepi/diag/backups/mission_pre_mark_cross_20261025_1348.py`。完整 A 面 mission 单实例 PID6439 待 START，arm_trigger PID6632 等 CH6，FC mode1/locked/PWM0。尚未实飞验证过线时机。
- 2026-10-25 上/下排打点偏侧分析：最新一轮上排触发 dx=-0.40/-0.34/-0.38，下排为+0.05/+0.19/+0.10。上下排航线方向相反，打点都发生在进入瞄准窗的前缘，所以落点呈相反横向偏差；这不是按高度给光轴做固定补偿的理由，也不能用分排 `AIM_TARGET_DX` 解决。
- 2026-10-25 13:35 换电重启 boot `352f2b14-fe16-454b-a2db-078dfbfbf81a`；完整A面 mission + arm_trigger 已由自动栈拉起，PID手动 `1.0/0.2`；BVA SHA `98b581b8eb35fc7a129cccfdb651881d74f93a5fb8db43b8533e07063c142c8f`（近点防回退8cm、mark门12°/s/4°），observer `traces/history_hover_20261025_133549.jsonl`；FC mode1/locked/PWM0。`/mark_ok` 在尚无PID画面输入时未被发现，飞行接管出现消息后再核。
- 2026-10-25 位置回退：轨迹显示航点按8cm判到点后，C++位置环仍向旧目标反向修正，形成可见后退。BVA新增近点反冲保护：`d<=8cm`且速度背离目标时直接清零XY。A2/A4打点被姿态门拦截的原因分别为航向3.61°和3.26°；mark门小幅放宽到 `|gyro|<=12°/s`、`|航向误差|<=4°`。
- BVA SHA `98b581b8eb35fc7a129cccfdb651881d74f93a5fb8db43b8533e07063c142c8f`；已重启，完整A面mission待START、PID `1.0/0.2`、arm_trigger等待CH6，FC locked/PWM0。
- 2026-10-25 扫描航向第五轮：`kp0.60/max5` 相比 `max4`，A5→A6 RMS 降至3.68°但 A4→A5 峰值升至9.88°；判定为阻尼不足。下一档只提高扫描 `kd 0.12→0.16`，Kp/max保持0.60/±5。
- BVA SHA `eeb6b3b47b62966b6f08a3ef9693936e0328b44a340c6239e5b2eec54e0c0fcb`；已重启，完整A面mission待START、PID `1.0/0.2`、arm_trigger等待CH6，FC locked/PWM0。
- 2026-10-25 10:46 换电重启 boot `d44f6f27-1118-4860-a25a-748d075a1408`；完整A面 mission + arm_trigger 已由自动栈拉起并保留，PID手动 `1.0/0.2`；BVA SHA `1e086ed329acc4facc92de0565a34d3f463d30ec3eb9950e1d51347a5adba69f`（扫描 yaw max±5），observer `traces/history_hover_20261025_104619.jsonl`；`/mark_ok` 1发布1订阅，FC mode1/locked/PWM0。
- 2026-10-25 扫描航向第四轮：`kp0.60/kd0.12/max4` 下多数段继续改善，A5→A6 仍出现约11.4°瞬态，控制指令在减速段贴±4°/s。下一档只把扫描 yaw 上限提高到 `±5°/s`，Kp/Kd/速度/横偏不动。
- BVA SHA `1e086ed329acc4facc92de0565a34d3f463d30ec3eb9950e1d51347a5adba69f`；BVA已重启，完整A面mission单实例待START、PID `1.0/0.2`、arm_trigger等待CH6，FC locked/PWM0。
- 2026-10-25 打点判定：A1/A4 实际打点时 FC gyro 峰值约19/14.5°/s，属于摆动中打点。BVA 新增 `/mark_ok`，条件为 active2、离地、`|gyro_z|<=10°/s`、`|航向误差|<=3°`；mission 要求视觉瞄准窗 + 新鲜 mark_ok + 速度`<=20cm/s` 才触发，0.15s脉冲。
- 停止/瞄准不再阻塞路线；待打点保留5s，期间同货号进入瞄准且姿态稳定才打。BVA SHA `c9ed47768dae2e4cae5f847e90993ab93ddde56dcef5f66d78be95cffb1e5cba`、mission SHA `dc2340e3a577a4a5bcfe345a2adfdbc93a122200d8d4d33eae0c95b20f52b789`。
- 当前完整A面mission单实例待START、PID `1.0/0.2`、`/mark_ok` 1发布1订阅、arm_trigger等待CH6，FC locked/PWM0。
- 2026-10-25 10:21 换电重启 boot `527d1752-8808-4d5c-9262-0f523276a01e`；完整A面 mission + arm_trigger 已恢复，PID手动 `1.0/0.2`；BVA SHA `1a411251a11d102ed477112f993d6b59a11364b3cbb674e9f6c8ad7c16974c89`（扫描 yaw `0.60/0.12/±4`），observer `traces/history_hover_20261025_102123.jsonl`；FC mode1/locked/PWM0。
- 2026-10-25 扫描航向第三轮：`kp0.50/kd0.12/max4` 相比 `0.70/0.15/max3`，多数扫描段航向误差 RMS 由约3–5.6°降至约2.2–4.9°，用户确认摆动改善。下一档仅将扫描 `kp 0.50→0.60`，kd0.12/max4、速度17、横偏0.30/max4保持。
- BVA SHA `1a411251a11d102ed477112f993d6b59a11364b3cbb674e9f6c8ad7c16974c89`，已部署重启；完整A面mission单实例待START、PID `1.0/0.2`、arm_trigger等待CH6，FC locked/PWM0。
- 2026-10-25 用户澄清要放宽的是下限：起飞放行窗改为 `100–180cm`、稳定0.3s；上限仍180。mission SHA `fea3849ea3e7b72470bc1bc7ac844a088ebf01bb65922adc987dfe29582f8cbf`，已部署并重启完整A面任务。
- 2026-10-25 10:03 完整 A 面再次六码全中；但扫描航向环 `kp0.70/kd0.15/max3` 指令饱和达 29–53%，实际航向段 RMS 约 3–5.6°，用户确认摆幅更大、回正更慢。响应相关显示 yaw 指令符号正确，但车体响应滞后约 0.4–0.7s。
- 扫描航向环改为 `kp0.50/kd0.12/max4`，降低饱和并给足纠正速度；扫描速度17、横偏0.30/max4、首段参数保持不变。BVA SHA `15dc20e7150cdfd7ec2f26b642125505099a6ba23834fcfd062f97d4edbe8768`，已部署并重启。
- 当前完整A面mission单实例待START，PID `1.0/0.2`，arm_trigger等待CH6，FC locked/PWM0。
- 2026-10-25 09:45 完整 A 面：A1–A6 六码全部打中。位姿分析首段 90.3cm 线路最大越过 3.5cm；扫描段横向 RMS 1.4–3.5cm，航向误差峰值约 5–7°。
- 当前扫描参数已独立：首段/过路 `vff20, ff_near20, cross0.20/max3, yaw0.50/0.10/3`；扫描 `vff17, ff_near14, cross0.30/max4, yaw0.70/0.15/3`。BVA SHA `b4f77738cef6c2ffb2e5e1cb62640b3da10f03fbd9e83cec1b62d8deb1d92e54`。
- 2026-10-25 10:02 新 boot `1fff1e58-7201-442b-a0d6-ff54151e0afe`；完整 A 面 mission 单实例待START，PID `1.0/0.2`，BVA新参数运行中，observer `traces/history_hover_20261025_100224.jsonl`，CH6等待；飞控 mode1/locked/PWM0。
- 2026-10-25 首轮完整 A：起飞与高度正常，但第一段 `no_brake` 期间航向未控，TF 机头从约0°漂到36–68°；随后 nav→body 把短距离修正旋转成反向运动。`A进-0→A3` 仅15cm，旧 `ff_near=8cm` 仍混入前馈导致过冲。
- 修复：航向PD改为所有 `active=2` 且离地阶段生效（不再因 `no_brake` 清零）；短段 `ff_near 8→14cm` 提前交回位置环。BVA SHA `19eb3f06a23a3dc0b8baa9ba4d7bf60ed4b1f08d03dcaa2331053aefaa9fd96a`；已部署，完整A面mission与arm_trigger待命。
- 2026-10-25 09:38：再次断电后 boot `65bf262e-c166-4161-8c4a-ba44b04ddc7a`；自动守护已拉起完整 A 面 `A1–A6` mission + arm_trigger，随后守护本体已停。PID 手动恢复 `1.0/0.2`。observer 写 `traces/history_hover_20261025_093820.jsonl`；当前飞控 mode1/locked、PWM0，CH6 等待。
- `±8cm` 和 `>=125cm` 两版均因核心起飞平台低于门限而悬停：最新一轮稳定在 `123–124cm`，所以125cm仍差1–2cm。说明核心物理起飞偏置会波动。
- 最终门槛改为 `110–180cm`、连续0.3s即放行；放行后 C++ Z 仍以142cm为目标继续补高。
- mission SHA `ea8deeca8433a8b6f442f5f0a2a0afa4df4084bd081744d17d2454b6337c0489`、BVA SHA `17232fa2f931b84ff05836f995ed4ba772adadcbedddbe8ae2020708ecb9c4ce`，py_compile PASS；mission 单实例待START。2026-10-24 收工，用户明日再测；arm_trigger 已停止，飞控 locked、PWM0。
- ±8cm 门首轮实飞：`0x66[1,142]` 已受理，但激光高度只平滑升到约 `127–129cm` 并平住；`134–150cm` 门 20s 未放行，`/target_velocity.z` 全程为 0，飞机原地悬停后人工降落。当前门过紧，且核心本轮没有把物理高度带到142。
- 证据：`flight_data/20261024_history_hover/third_attempt/`；`/fc_height_diag` 仍无样本，无法区分核心 `fus/add`。residual mission/waypoint 已停止，飞控 locked、PWM0。
- 激光打点/扫描放宽：`laser_driver` 在 domain6 且 `/laser/trigger` 有订阅；mission 按 `center_tol=0.60`、`SCAN_ACCEPT_SPEED=30cm/s` 采纳，视觉进入 `AIM_TOL=0.25` 瞄准窗即打点，脉冲 0.15s。
- 修正：上一轮悬停不是激光门，而是 BVA 在扫描 PD 切换时调用不存在的 `YawDamper.reset()` 崩溃，`/target_velocity` 断开后 UART 发零速保活。已补 `reset()`。
- 打点改成“顺手打”：识别后不等待停稳，视觉进入 `AIM_TOL=0.25` 瞄准窗即发 `0.15s` 短脉冲；离开货位仍未进窗则跳过。
- 板上因并发 CLI 曾短暂显示 `/laser/trigger` 订阅为 0；用完整 topic endpoint 复核后 `laser_driver` 订阅存在，GPIO40 默认高电平熄灭。不要据此重复重启驱动。
- 扫描段优化：XY 已是逐段锁定方向、恒速前馈 + ≤3cm/s 横向纠偏；航向此前不是强锁定。最新 A 面日志中目标航向恒为 0.5°，实测机头摆到 +12.6°…−7.5°，旧 BVA 阻尼长期饱和在 ±2°/s。
- 候选新增扫描专用航向 PD：`-0.5*误差 -0.10*陀螺Z`，±3°/s，仅在 `active=2 + 锁定直线 + no_brake=false` 时启用；其他阶段保留原控制。已部署到 `/home/orangepi/diag/body_vel_adapter.py` 并以 `BVA_SCAN_YAW=1` 启动，纯类符号/限幅测试 PASS；尚未实飞。
- 扫描航向证据：`flight_data/20261024_history_hover/scan_yaw/`（BVA CSV、waypoint 日志、analysis.json）。
- 用户确认异常轮是电池绑带脱落并挂住/遮挡激光，导致高度误判；不是双高度环互相抢高度。双环不抢高度已验证，继续当前 CH5=2 + 原生起飞 + C++ Z PID 方案。
- 第一步已将起飞放行门从 `142±12cm` 收窄为 `142±8cm`（134–150cm），持续 0.3s；板载 mission SHA `808c8bc96e57271d44f11c4e21b126b508fd70c9c65823fbda38aa47bd73e246`，py_compile PASS。
- 当前 mission 单实例待 START，范围 A3→A2→A1；未挂 arm_trigger，飞控 mode2/locked、PWM0。下一步由用户决定是否复飞验证 ±8cm 门。

## 10/14 双高度复现：两次起飞对比（2026-10-24 晚）
- 候选 `stage1_20261021/flight_effect_candidate/history_hover_20261014/`；CH5=2、原生 `0x66[1,142]`、BVA 激光 Z 关闭、C++ Z PID 1.0/0/0.2。范围 A3→A2→A1。
- 正常轮：`0x66` 后激光 1→112→128→142 cm，约 3.87s 通过 `142±12cm/0.3s`，随后 C++ Z 接管；稳定段高度接近 142。
- 异常轮：mission 未交棒，`/target_velocity.z` 全程严格为 0；激光从 3 出现 376/333/253 等非物理跳变，随后长期停在 5–12 cm，而 PWM 仍约 60%。20s 高度门超时，未进航线。香橙派 Z 不是这次持续爬升的来源。
- `/fc_height_diag` 的发布者存在，但两轮观察文件均 0 条；核心 `fus/add` 无法用于判定。`/fc_target_vel` 当前不存在，内部目标 Z 也无记录。
- 证据：`flight_data/20261024_history_hover/compare_takeoffs/`（原 JSONL、两份 mission 日志、analysis.json、analyze_compare.py）。
- 当前已停止 mission/waypoint，飞控落地锁定、PWM 0。下一步先补齐 F407→核心高度输入或 `fus/add` 的同步证据，不先调高度门或加固定补偿。

## PC 程控启动 + 激光 Z：CH6 受控降落已烧录（2026-10-24）
- 更正：不能把一键起飞少升直接归因于“激光与气压融合抢高度”。真实F407事务在接收到核心`cmd1=5`回显后立即置`takeoff_if=1`，未等待实际高度；此后外部Z已获准进入`rt_tar.vel_z`，存在两路Z指令过早衔接的风险。但已分析的10/22起飞轮与10/24高压轮中，爬升期`target_velocity.Z`均为0，尚不能证明该机制实际导致少升。后续必须按每场`cmd1=5`回显、首个非零外部Z、当时激光/融合高度三点核对。
- 新 boot `557860f7-9d2e-4f3d-bca5-fe1b41b069c1`；CH5 当前回到低档，`/fc_status=[1,0,0,0,0,0,0]`（模式1、锁定）。测试前须重新把 CH5 拨回程控档并核对。mission PID 2882、BVA PID 1912 仍是部署前启动的旧进程；未发 START/解锁/控制指令。
- 本场最后一次飞行证据在 `flight_data/20261024_fast_final_high/`：`0x66[1,156]` 在激光 136cm 放行，稳定段激光中位 155cm（10–90% 147–158，峰值 161）；交棒后 `/target_velocity.z` 1143 帧全 0。根因是核心融合定高与物理激光的映射不稳定，且 BVA 在上排目标附近主动清零 Z。
- 核对了 `fc407_patch/AnoDTRasp.c`、`ANO_LX.c` 与匿名协议：0x66 受理后 F407 置 `takeoff_if=1`，0x31 的 Z 会进入 `rt_tar.vel_z`；模式 3 定义为程控，但尚未证明一键起飞结束后核心会释放自身融合高度目标。模式 3 仍可能与核心定高互顶，不能直接宣称纯激光闭环已安全。
- 已备份前一轮候选到 `/home/orangepi/diag/backups/laser_z_program_20261024_pre_program_arm_557860f7`；线上源现为本轮 mission SHA`4b958fa…bbe6`、BVA SHA`3aa32be…7658`，板载py_compile PASS。未重启，PID 2882/1912仍运行旧代码。
- F407现含`0x66[5]`程控arm、硬件互锁和CH6受控降落。程序arm后模式3锁存，不再随CH5误切；CH6高位上升沿清外部速度并请求`OneKey_Land()`，不调用`FC_Lock`，落地解锁后才恢复模式选择。当前HEX SHA`e8aad80…279d`，Keil构建0 error/0 warning，J-Link重烧`verifybin`校验61760字节PASS。备份：原始版`rollback_chip_before_program.bin` SHA`6236287…5f82`，仅程控启动版`rollback_chip_program_start_v1.bin` SHA`c277e0e…a253`，安全互锁版`rollback_chip_safety_interlock.bin` SHA`e22c11a…66fa`。
- mission默认`takeoff_mode=program`：地面START发`0x66[5]`，5s内须解锁；随后停pos_hold→`/laser_z_enable`→active2，从地面用激光Z飞到142±4cm并判稳0.5s。BVA在低于40cm时只允许正Z离地，XY/偏航保持零；高度误差上限改为200cm以覆盖地面到142cm。
- 验证：`py_compile` PASS；纯状态机/源码/F407静态检查38项PASS，旧航线回归18项PASS；板载ROS域80/localhost-only合成运行13项PASS（含地面正Z离地、飞控模式掉档、断流锁存）。未发START/0x66/0x31；F407构建不代表串口或物理行为。
- 当前飞行控制仍mode1/锁定/PWM0；mission测试进程已停止，被动observer仍在记录。重烧前不要切mode3、不要START。重烧并复核后，先做CH5 3→1地面互锁测试，再做系留/低高度正Z响应；操作手必须准备在模式3异常时立即切CH5到1/2接管并手动降落。
- 地面arm测试：`0x66[5]`使`unlock_sta=1`、PWM约18%；CH5切1为`[1,0,1,1,...]`、切2为`[2,0,2,1,...]`，均`unlock_sta=0`、PWM0，故CH5不能作为空中接管。CH6受控降落路径已加入并烧录，但尚未发CH6做过地面降令验证或低高度下降验证；mode3正式飞行仍禁止。

## 直线航段推进修正（2026-10-24，已部署待重启）
- 依据`flight_data/20261024_core156_route/`：150cm直段耗时12.2s、横向RMS5.4cm；A3→A2与A2→A1的50cm分别9.7/10.6s，横向RMS2.5/4.8cm。BVA运行环境确认`BVA_FF=1`。
- 候选`flight_effect_candidate/straight_route_20261024/`：时间预算改用实际限速15cm/s并计入加速；直线保持到近段18cm；新增kp0.20/max3cm/s横向纠偏；接近目标用kbrake*d平滑减速。mission仅修正误导性“直线逻辑关闭”日志，航点与156/142设置不变。
- 已备份旧mission/BVA到`/home/orangepi/diag/backups/straight_route_20261024_120915`并部署源文件；新SHA mission`5eb630…944f0`、BVA`bc7fef…09a39`。当前PID7544/1896仍运行旧代码，下次断电重启生效。
- 验证：py_compile和9项离线几何/源码检查PASS（候选`check_result.json`）；未做ROS运行时或实飞。首飞目标：90cm首段约7–8s、50cm段约5–6s、横向RMS<4cm/峰值<8cm；过冲或振荡则回退。
- 已重启生效：新boot `5a551c0c-3b51-49e4-8ad9-0ee9a4d34c02`；mission SHA`5eb630…944f0`、BVA SHA`bc7fef…09a39`，BVA PID1915，环境`BVA_FF=1`。mission PID3307待START，observer PID3280写`/home/orangepi/traces/height_A_straight_20261024_121329.jsonl`；飞控锁定/PWM0，未发START。

## 0x66目标156复测：可进航线，但首段与扫描推进慢（2026-10-24 11:56）
- 证据`flight_data/20261024_core156_route/`。核心收到`0x66[1,156]`，激光到134cm后由v2闸门放行；mission于START后约18.85s令waypoint_nav开始，首个90cm过渡段又耗时约12.2s。
- 首段实际进入BVA直线分支（运行进程`BVA_FF=1`）：前约4s目标速度15cm/s并移动约65cm；随后`_cor_left`按`distance/vff(20)`在4.5s耗尽，剩余约20–25cm交回滞后位置环，速度降到约2–6cm/s，耗时约8s。首段横向RMS约5.4cm、最大8.8cm。
- mission在A进-0明确关闭“直线逻辑”，扫描段恢复闭环；A3→A2的50cm用9.7s（约5.2cm/s，横向RMS2.5cm），A2→A1的50cm用10.6s（约4.7cm/s，横向RMS4.8cm、最大9.6cm），并出现接近航点后回拉。
- 根因不是BVA_FF未开启，而是：时间预算使用vff20而输出硬限幅15；计时结束后没有平顺减速/横向纠偏交接；扫描段本来就不是整段直线前馈，且SLAM位置滞后会让末段减速和回摆。
- 当前飞控已上锁/PWM0，但mission/waypoint进程仍残留，下一次飞行前必须清理。保留核心目标156的本次实验；未改控制代码。

## 高速直线扫描与打点门修复（2026-10-24，已部署待重启）
- 依据`flight_data/20261024_straight_route/`：上一版虽然BVA_FF=1且横向RMS多段2–4cm，但50cm段仍约9–10s，低速主要耗在18cm内位置环收口；A1/A4/A5/A6均出现视觉“对准判据=等”后仍由on_reached直接打点。
- 候选`flight_effect_candidate/fast_straight_scan_20261024/`：巡航/限速20cm/s、加速度20cm/s²、kbrake0.8、近段阈值8cm；保留横向纠偏。`SCAN_ACCEPT_SPEED=25`允许巡航识别，激光仍要求停稳≤10cm/s且`_aim_ok`新鲜。
- 删除`on_reached`自动打点旁路；航点索引前进时若pending仍未视觉居中，明确记录并丢弃，宁漏勿偏。每格hold保持0.5s。
- 已备份上一版mission/BVA到`/home/orangepi/diag/backups/fast_straight_20261024_122821`并部署新源：mission SHA`441137…53be0a`、BVA SHA`be70e1…ee4457`。当前进程仍为旧代码，下次断电重启生效。
- 验证：py_compile、13项离线几何/源码检查PASS（候选check_result.json）；未做ROS运行时或实飞。目标：90cm首段约5–8s，50cm段约4–6s，只有视觉居中的码才打点。
- 追加进入航线加速（同日12:31）：waypoint_nav在爬升阶段预加载且保持active=3不运动；门通过后直接stop poshold→active3→active2；poshold预热2.0→0.8s。交棒带宽8→10cm（132–152cm，仍排除128cm）。mission SHA`2af2cfb0…c5ac0`，BVA SHA`be70e1…ee4457`；备份`/home/orangepi/diag/backups/fast_entry_20261024_123131`。18项离线检查PASS，仍待重启与实飞。
- 已换电重启：boot`2b1601f4-66f1-4ed2-bdc8-0ef7a74d7e19`加载 mission`2af2cf…c5ac0`、BVA`be70e1…ee4457`，BVA_FF=1。mission PID3319待START，observer PID3274写`/home/orangepi/traces/height_A_fast_20261024_123737.jsonl`；飞控锁定/PWM0，未发START。

## 直线前馈首轮复测：仍慢、停不住且打点绕过对准门（2026-10-24 12:13）
- 证据`flight_data/20261024_straight_route/`。本boot已运行新mission/BVA（`5eb630…944f0`/`bc7fef…09a39`），`BVA_FF=1`，156cm核心目标保留；mission扫描完成并在A出-1之后进入过路模式，随后操作手手动降落。
- 起飞：START→waypoint_nav开始约23.95s；其中命令后到激光134cm带内判稳通过约16.6s，带内实际判稳耗时7.6s。之后首段90cm约12.6s。
- 扫描推进仍慢：A3→A2为50cm/9.09s、A2→A1为50cm/9.10s、A4→A5为8.9s、A5→A6为9.8s，中位实测速度约4.3–5.5cm/s。`BVA_FF=1`确认生效，因此不是没走直线分支；慢点集中在距目标18cm内交回C++位置环后的收口。
- 横向：A3→A2 RMS2.8/max4.2cm，A2→A1 RMS2.36/max4.14cm，A5→A6 RMS4.54/max7.51cm；后段偏差增大。BVA日志目标航向约0.6°，10s窗口实际航向误差摆到约-7.6°..+11.5°，悬停越久相机横向偏移越明显。
- 打偏是确定代码缺陷：`mission_node.py:638-645 on_reached`在航点到达时无条件触发已有pending货位的激光，绕过`_aim_ok()`。本场A1/A4/A5/A6均有“对准判据=等”随后仍打点；激光触发时相机偏移达0.3–0.48。货号数字识别和货位绑定本身均正确（A1..A6）。
- 下一步优先级：先删除on_reached的无条件打点，只允许新鲜`_aim_ok`+停稳后打；再将BVA近段阈值18→8cm并增大制动斜率/推进速度，分开验证推进速度；航向慢摆需独立调参，不能与打偏混为一个修复。

## 一键起飞156抵消实验（2026-10-24，已部署待飞）
- 依据：三次`0x66[1,142]`试飞中，核心在融合增量约141–146cm时悬停，激光只增加119–127cm。用户明确要求把送入核心的目标改为156。
- 候选`stage1_20261021/flight_effect_candidate/core_takeoff_156_20261024/mission_node.py` SHA`6c3bc573…ee33e`。仅当首航点高度为142时发送`0x66[1,156]`；激光交棒目标、±8cm闸门和航线高度仍为142。
- 已备份原mission到`/home/orangepi/diag/backups/core_takeoff_156_20261024_115528/`并部署。mission PID7544待START；observer PID7502写`/home/orangepi/traces/height_A_core156_20261024_115539.jsonl`。
- 当前飞控锁定、PWM0；未发START/未解锁。`TAKEOFFHIGH=100`默认参数未改，UART/心跳未改。
- 待验证：核收到156是否使物理激光接近142；若机体过冲、未到142或交棒后下降，停止156补偿，回到激光Z闭环方案。

## 冷启动复测：核心按融合增量到高，物理仍约124cm（2026-10-24 11:38）
- 新boot `6db009a6-ad86-404e-8efa-8fcc68952861`，新版UART/±8cm闸门。证据`flight_data/20261024_A_cold128/`；mission单次`0x66[1,142]`，+0.292s核心回`cmd1=5`。
- 上电初期融合高度约10cm，起飞前已漂到约47cm；命令后8–12s激光平台约120–125cm，同期融合185–189cm，融合相对起飞增量约138–142cm。核心达到命令增量时物理激光仅增加约121–123cm。
- `TAKEOFFHIGH=100`不是本次覆盖值：协议说明0x66高度字段为0才用默认高度，F407对非零142原样写成CMD[2:3]。实测融合增量跟随142而非100，进一步支持非零载荷被核心采用。
- 本轮`/fc_status`无cmd1=6；约+12s目标油门从538开始下降，随后落地，符合操作手进动。温度/气压或融合尺度不是简单冷启动残留，因为冷启动后地面漂移及融合增量/激光增量≈1.14仍在。
- 结论：核心把命令当“相对起飞时刻融合高度的爬升量”，但其融合垂直增量相对激光多约14%；因此命令142时物理停点约124cm。核心闭源，无法证明内部增益来源。下一步若要求物理142cm，应由香橙派激光闭环接管垂直控制，且先定义核心高度环退出/仲裁；仅改TAKEOFFHIGH、闸门或补固定偏移不可靠。

## v2+心跳复测：闸门正确拒绝128cm平台（2026-10-24 11:26）
- 证据 `flight_data/20261024_A_plus8/`。同一 boot `d2f7d952…fe0d` 运行新mission/UART；mission只发一次`0x66[1,142]`，+0.156s核心回`cmd1=5`。
- 起飞后8–18s激光平台中位128cm（121–131），从未达到v2下限134cm；融合/诊断高度同期中位177–178cm，与激光差约49–50cm。±8cm闸门始终未放行，完全正确。
- `/fc_status`全程只出现cmd1=0/4/5，没有cmd1=6；`/target_velocity`在观测窗口持续、Z=0。+19.48s目标油门首次低于500，随后高度下降并于+26.97s解锁，符合操作手回拉油门，不是mission发降落。
- 结论：v2闸门与速度保活未造成128cm平台；剩余问题位于核心一键起飞/定高的停止逻辑及高度源映射。交棒窗口的连续性本次没有被触发，仍未验证。
- 当前已锁定、PWM0。未改代码/参数。下一步不能继续收紧闸门；需明确核心高度控制或让香橙派接管垂直环，且必须先解决两套Z控制权冲突。

## 交棒收紧与速度心跳（2026-10-24，已部署并重启生效）
- 实机运行源匹配 `stage1_20261021/flight_effect_candidate/first_leg_20261023/`：mission SHA `4ab554…5e58f`、UART CPP `8e07c7…794ec`、HPP `e0cb0a…b3fe8`。候选在 `flight_effect_candidate/handoff_heartbeat_20261024/`，基准文件保留未改。
- 修改1：`TAKEOFF_HEIGHT_TOL_CM 12→8cm`，交棒允许134–150cm（目标142），保持0.3s、新鲜度1s；上一轮最高133cm，新门不会再交棒。
- 修改2：`uart_to_stm32` 增加最终速度流保活：`/target_velocity`超过150ms无新消息时，直接以20Hz向串口发送`0x31 [0,0,0,0]`，新命令恢复即停止心跳；定时器50ms，析构取消。
- 验证：mission `py_compile`通过；15项AST/边界/源码检查PASS（`check_result.json`）；真实ROS Humble环境隔离和机载正式`colcon build`均通过（`build.log`/`build_actual.log`）。
- 已按用户要求部署但不手动重启；换电后boot `d2f7d952-2b51-4e0d-be8a-a31faa24fe0d`已加载新binary SHA `78a88d2f…0713bc`（UART PID1876）。备份为`/home/orangepi/diag/backups/handoff_heartbeat_20261024_111922`。
- 启动验证：`/target_velocity`无ROS发布消息时，UART日志出现零速心跳并持续发送`0x31 x=0 y=0 z=0 yaw=0`；`FC link echo last frame LEN=8`确认飞控接收。飞控锁定、PWM全0。
- 新版本待飞就绪：mission PID3325单实例，A1–A6，日志明确`起飞高度闸门v2：±8cm/0.3s`；被动observer PID3853写`/home/orangepi/traces/height_A_20261024_112542.jsonl`。未发START/未解锁。
- 尚未执行新版本飞行，因此±8cm交棒行为和心跳在交棒窗口内的连续帧率仍待首段实飞验证。下一步由用户按原流程起飞，先确认任务日志显示闸门v2，再在交棒时核对0x31无静默。

## A面重飞：起飞正常，交棒时速度流停止后核心自降（2026-10-24 11:04）
- 新证据 `flight_data/20261024_A_retry/`。mission 只发一次 `0x66[1,142]`，`+0.296s` 核心回 `cmd1=5`；激光从2cm升至最高133cm，t+5.6s左右进入130–133cm悬停，t+6.2s到t+9.8s满足现有±12cm判据。
- `/target_velocity`在t+9.817s停止，t+10.13s才置`/active_controller=3`；t+10.451s核心报`cmd1=6`并开始下降，t+15.2s前后上锁。全程Z目标为0，mission未发送`0x66[2]`，油门输入约520且下降开始前无人工回拉。
- 强时序证据指向交棒代码`stop pos_hold → set active=3 → start waypoint_nav`造成0x31流静默；现有“active=3会继续发0”的代码注释在实际运行链中未成立。还不能读取核心源码，无法证明其内部超时精确阈值。
- 目标=142时当前判据允许130放行；本轮最高133，未到达142。该放行只影响交棒时机，不是本次下降命令；但会让“核心已经完成142”与上层认知不一致，后续仍需单独修正。
- 当前已锁、PWM0。未改代码/参数。下一有界动作：设计单一速度发布仲裁或零速心跳，并用接收时间记录做离线验证，先消除交棒断流再复飞。

## A面复测未起飞：解锁后高油门地面起转（2026-10-24 10:51）
- 新 boot `f41f973a-b817-407c-9796-d9edc2618dfa`；唯一 mission PID3269，范围A1–A6。START 后核心板在 `+3.42s` 报 `cmd1=6`，mission 未发任何 `0x66`，随后因 `unlock=0` 拒绝起飞；当前已上锁、PWM全0。
- 证据：`flight_data/20261024_idle_spin/rec_20261024_104934.log`、`height_A_20261024_105027.jsonl`、mission/pos_hold日志。地面段 `/height=2cm`；解锁前遥控目标油门已约572–577（中位约576），解锁后PWM从18%升到约45%，但无持续偏航，Pi的`/target_velocity`全0且mission已置`active_controller=3`。
- 用户确认整机有原地打转趋势并主动回拉油门。观察器记录的偏航目标为0、`/target_velocity`为0；主trace中18%怠速段四路基本相等，随后目标油门仍约576时PWM升至约40–45%，转向趋势出现在地面升推阶段。现有证据支持“解锁后油门高于中位500，飞控在地面产生推力/力矩”，不支持香橙派注入偏航；仍不能区分核心偏航环与桨/电机机械力矩。不可把该轮当作自动起飞或高度复测。
- 机载被动observer PID3869仍在写上述JSONL；未改固件/参数、未发起飞或降落命令。下次复飞前先确认遥控油门中位并重新上锁/启动单一mission实例。

## 回查雷达速度失效（2026-10-23，本轮）
- 用户提出历史起飞/半途下降曾归因雷达速度失效；本次主trace在起飞回落窗口89条10Hz样本中，原始/过滤水平速度仍变化，各轴峰约6.8cm/s，无>=60cm/s尖峰。不能据此排除亚200ms断流或F407接收/有效性问题。
- 当前真实UART source仍缓存velocity_valid并在TF回调重发，未核对源年龄；串口持续发送/上位机通用速度正常不能证明源新鲜。上次改动只移除A2阻塞，不解决测量源时效。
- 已准备本地被动velocity_height_observer.py（flight_data/20261023_A_low_hover/），记录raw/filter接收间隔、scan源戳、融合/激光、气压与控制状态，语法通过；尚未部署。SSH192.168.137.186连接超时，待网络恢复后启动，禁止把本地候选当已就绪。

## A面复测：先升后掉并低空悬停（2026-10-23 21:53轮）
- 用户反馈起飞半途回落、低空悬停后手动降落。证据`flight_data/20261023_A_low_hover/`及analysis.json：仅一条mission_cmd=[1,142]，回落期间target_velocity.Z全部0，无上层降落命令，未进入90cm段。
- 同步双高度关键样本：起飞前融合30–31/激光3–4；命令后1.953秒融合159/激光98；4.546秒融合155/激光29。激光下降69而融合仅降4，且用户确认机体真实回落；这是高度估计不随实际下降的直接线索，不能只调准入阈值或给142加固定补偿。核心是否按融合值定高仍属推断，内部target_vel无消息。
- 本地真实F407源码Set_m_speed_now到0x33把外部测量Z标0x8000无效，不把SLAM Z=0当有效垂直速度；不据此将香橙派“目标Z0”与“测量Z0”混淆。
- 发现mission3216和3250同时等待，单主控锁拒绝第二份；已停重复3250/3248，复核仅3216/3214保留。共享日志已有混写，精确时序使用唯一observer3165 JSONL。未改飞行控制/发起飞，当前锁定PWM0。
- 下一项：核对IMU核心高度源/融合状态，现有香橙派协议不能读真实核心参数；已询问用户匿名上位机是否可连接。保持落地，不再重复相同A面试飞。

## 第二次换电：A面任务待机（2026-10-23，本轮）
- boot c20c8906-4073-403b-aff1-7b1684f531e6；主链active/running，UART1885实际exe SHAe6b2c4…a581d，mission源码4ab554…5e58f保留。
- 按用户明确要求恢复整个A面A1–A6任务3216待START；fcstat3163唯一实例，双高度observer3165记录/home/orangepi/traces/height_A_c20c8906.jsonl（30分钟）；主trace1120记录rec_20261023_211704-1.log。定位有效、地面h1/unlock0/PWM0。未发起飞指令。
- 飞后先取本boot双高度与主trace，不以文件名推时序（上电时钟曾回退）。

## 换电后起飞不足：用户手动降落，已复核（2026-10-23 21:46）
- 用户澄清本轮为手动降落，不是飞机自降；任务日志“自降或未解锁”是手动降落后的中止判定。配置仍A1–A6整面，本轮未进90cm段；之前称只测首段未正确限制任务范围，后续配置需明确。
- 本轮证据`flight_data/20261023_post_battery_takeoff/`。主trace独立分析：离地后到首次明显收油门约16.1秒，高度约100→112→119→121，期间ROS目标Z全0；全程最高123含手动降落过渡。最高电机输出71.5%不能证明动力充足，也没有饱和证据。未证实固定120截断/硬上限，不据此加补偿。
- fcstat出现3253和本轮重复3471混写同一CSV，存在畸形t，不能用它精确判定cmd1切换对应高度；先前称“约100cm从5到4”撤回。已只停本轮重复3471，保留3253；主trace不受该CSV混写影响。
- 补齐被动observer6737，文件`/home/orangepi/traces/height_events_postbattery_20261023_214517.jsonl`，1800秒有效至约22:15。39秒地面采样height/fc_target/fc_vel持续，fc_height_diag194条，内部fc_target_vel仍无消息。融合396–408cm、附加激光2–3cm，说明数值参考不一致；不能据地面差值确定起飞高度基准或直接作补偿。
- 缺口：本轮空中没有同步融合高度，无法反推核心在120cm时是否认为已到142。下一次需保留同一观察器记录起飞前基线/上升/平台，比较融合与激光增量；不重复无记录试飞，不自动发START。UART异步/接管改动保留，飞行效果待首段有效复测。

## 换电恢复已核对（2026-10-23，本轮）
- 新boot13d78583-2821-4cc6-9606-0584ab676b85；flight-link active/running，UART1884实际/proc/exe SHAe6b2c4…a581d、mission SHA4ab554…5e58f，修复保留。
- 恢复被动fcstat记录及原A1–A6等待任务3518；主trace1129写/home/orangepi/traces/rec_20261023_211704.log。定位初期nan现已恢复map→dr_base，激光扫描约14.56Hz，最新位置(-0.86,-0.26)cm；h2/unlock0/PWM0。未发START/解锁/飞行。
- 下一步用户沿原方式复飞首段，降落后取上述本次boot日志。

## 90cm停走与接管等待修复已部署（2026-10-23 21:31）
- 当前任务：悬停先暂停，只复测悬停→90cm首段的等待和走停。最新实飞证据在`stage1_20261021/flight_effect_candidate/target_vel_diag/`，不是只看flight_data旧目录。
- 已定位源代码/日志证据：UART每收到active=2执行3次应答并sleep100ms，总阻塞约300ms；交接连发10次及巡航每2秒补发会反复堵住速度回调，超过F407源码的200ms指令看门狗。首段记录同时有ROS指令非零、F407目标间歇归零；这是停走的强原因证据，物理改善待复飞。
- 已部署：UART用100ms异步timer完成三次应答，重复信号在应答期间合并；mission启动航点前清空旧wpn日志，看到新waypoints=标记立即交接，未见标记则沿用原2秒等待。BVA/悬停/高度参数保留。早前“no_brake无限巡航”临时候选已撤回为板上基线，不能部署该方案。
- 验证：ROS完整UART单包构建通过；真实提取回调在domain80隔离运行，sent6/pulses134/max_gap21ms PASS，无串口连接。证据`first_leg_20261023/build.log`、`check.log`、`flight_before.log`、`deployment.json`。
- 地面h1/unlock0/PWM0后备份到`/home/orangepi/diag/backups/first_leg_20261023_212925`并标准重启flight-link；active/running，UART13811实际/proc/exe SHAe6b2c4…a581d匹配。BVA仍86f6be…510f1。任务15208待START，A1–A6范围，未发飞行指令。
- 下一有界动作：用户沿原方式飞首段并降落；主记录器1126仍写`/home/orangepi/traces/rec_20261023_201704.log`。飞后取当前mission/wpn及主trace，对比零目标次数和高度放行→首段启动耗时。

## 内部目标速度诊断候选进度（2026-10-23）
- 用户已授权准备本地诊断候选及离线验证，不含实机部署/重启。已实现 target_vel_diag/audit.py 离线原始字节分析器；7组检查通过，证据 stage1_20261021/flight_effect_candidate/target_vel_diag/check_result.json。
- 已制作本地candidate/serial_rx_capture.h和serial_comm接收镜像钩子（默认关闭、8MiB封顶）；源码差异检查通过。已用板上既有g++11.4在自动清理的临时目录隔离编译真实serial_comm.cpp，采集类7项真实文件测试通过（含写失败/上限/恢复），证据target_vel_diag/capture_build_result.json。未部署/连接串口；ROS整包链接及运行时序未验证。现有三轮日志无/fc_target_vel消息，rec的fcTarV仅XY且全nan；没有实飞原始串口字节，不能判断120cm平台的内部目标Z。
- 本轮实机只读复核boot a43bfd45-db1f-4f39-b01d-19448f811fe3，UART1889实际SHA b14888…0167。8秒被动订阅height/fc_vel各397条，内部目标0条，发布器存在；当时锁定/height1。新boot使历史PID失效。serial_comm源码SHA70b7b904…32628b1，无原始采集/帧计数开关。
- 当前候选准备和离线验证已完成；后续如需地面采集，必须单独明确部署/重启授权及静态窗口，先完成ROS整包链接与集成检查。保留142/103和控制设置，不自动部署或飞行。

## 新对话继续入口（2026-10-23）
- 用户要求新对话继续。当前只关注起飞不足142cm，保留142/103和现有高度选源/几何设置；用户确认遥控已落实，停止遥控诊断支线。
- 已部署receipt_gate候选，见下节。只修判断，物理少升未解决。失败轮融合约69→213（+144），激光1→121（+120），仅是线索；成功轮没有同期融合数据，激光132放行后继续139–152。不可认定核心使用相对融合高度，也不可套旧+16cm补偿。
- 本地飞控手册507–509说明自动激光+气压计智能定高，未说明一键起飞高度基准。协议p22参数表仅示例，TAKEOFFHIGH82、TAKEOFFSPEED83，不是实机值。参数表_ocr.txt是旧OCR错误输出，应看协议p22_参数表.png。
- 已确认香橙派→F407未转发E1，ROS未处理E2；F407 ANO_DT_LX.c中E1本地响应固定par_val=0，不是核心配置，不能查询后误作实值。不新增参数透传/遥控诊断固件。
- 下一有界动作：只读核对失败/成功日志的核心目标垂直速度（/fc_target_vel或记录器fcTarV字段，先确认字段含义与来源），判断120cm平台时核心是否仍要求上升。不能把外部/target_velocity.Z=0当核心内部目标。若无Z记录，明确证据缺口，避免重复查协议/遥控或未经依据补高。
- 固件刷写、复位、解锁和飞行均不在本次诊断授权中。现有设备PID/锁定状态为历史，下次受影响操作前重新核实。

## 起飞受理/离地判断已部署（2026-10-23 17:11）
- 实机基线809312…相符；新鲜fcstat高度3cm、unlock0及PWM全0，旧任务6575待机。备份/home/orangepi/diag/backups/receipt_gate_20261023_171102后部署ba44c4039030aca5d9a132c57b5c563daa6232f034f9512b0c4f20225208fe43，仅替换/重启等待任务，新timeout8887，新增启动日志确认就绪。
- 证据stage1_20261021/flight_effect_candidate/takeoff_receipt_gate/deployment.json。142/103、Z、模式及重试策略未变；未重启主链/改MCU/发送START或飞行。软件检查40项通过不代表实飞；旧命令持续回显局限及物理少升仍未解决。
- 下一步继续只读核对核心仲裁依据；不以本次判断修正已部署为起飞高度修复成功或自动试飞条件。

## 起飞受理与离地判断候选（2026-10-23，本地未部署）
- 已实现stage1_20261021/flight_effect_candidate/takeoff_receipt_gate/mission_node.py，SHA ba44c4039030aca5d9a132c57b5c563daa6232f034f9512b0c4f20225208fe43。发送前记录回显序号/时间，只接受其后的新鲜cmd1=5；离地25cm判断要求新鲜激光、解锁且未abort。142/103、Z、模式及重试策略保留。
- check_receipt.py实际AST回调/helper及调用点15项通过；原高度闸门25项回归通过，语法通过。未验证ROS时序/物理控制。新收到的cmd1=5仍可能是核心旧命令的持续回显，协议无事务ID，不能证明本次受理；不解决物理少升。
- 未连接/部署/重启/飞行。当前设备仍为809312…高度闸门版（本轮未重新核验）。下一步复核候选差异及回显局限后，再决定部署；控制仲裁尚未解决，不能自动加入补高。

> **2026-10-21 用户最终澄清：基于当前代码架构和实飞现象，优化飞行效果，完成单板A/B两面任务。** 当前入口为《优化方案-香橙派D题-2026-10-21.md》飞行效果版。
> 重点顺序：BVA直线方向/减速收口/平滑 → 停点与偏航 → 换排 → 扫码补打；一次局部改动、一次针对性动作对比，不走全系统验证或架构重建。
> 当前前景范围按用户最新要求限定为起飞→悬停接管→飞完首段90cm。BVA航向、35cm退出巡航及正常加速/转向平滑已部署；平滑效果待复测，不以UART收尾或完整源身份/控制交棒为前置。
> 归零坐标修正已部署，21:27首次核对197组同TF时间戳上游/输出航向差为0°，非零角度及再次归零行为已离线检查；换电后代码哈希保留，实飞整体改善未验收。
> 保留原硬件、Cartographer安装、UART协议和路线；已有候选与验证证据保留，不自动接入，不删除或回退，不重编Cartographer。
> 下文旧“阶段①剩余/下一bounded task”作历史索引，不自动继续。当前仅完成下述BVA与归零航向局部修正及部署重启，后续物理动作按对应授权执行。

## 起飞判定局部修正已部署，待机就绪（2026-10-23 17:00）
- 用户“下一步”承接先修到位/判稳/超时逻辑。保持142/103目标、原±12cm/0.3s接受标准与全部垂直输出不变；两阶段分别30/25s超时必须返回失败，取消127/112及最终±15兜底；日志明确带内判定，不再声称精确达到142。新鲜高度门槛1.0s为新策略参数，非实测设备超时；同一旧样本不能累计0.3s，断流/越带重置计时，落地/abort中止；回调断流>1s后重接允许重同步0–500cm样本，保留新鲜连续样本100cm跳变剔除。
- 候选stage1_20261021/flight_effect_candidate/takeoff_height_gate/mission_node.py SHA80931220ba0e8dca4891779eaf96551c35ea8fc042f8b7ab920190f4f2459cee，check_gate.py对真实候选AST提取helper/等待方法/回调执行25项正常、门槛、超时、缓存、断流/恢复、上锁/abort检查通过。已修正本地写入CRCRLF导致的行续接语法问题，最终实际文件语法通过。check_result.json、gate_only.patch及deployment.json为证据；不代表实飞验证。
- 实机基线SHA531050…d617相符，确认任务3028仅待START、unlock0/PWM0后备份/home/orangepi/diag/backups/takeoff_height_gate_20261023_170043；只停止旧等待任务，原参数/环境启动新timeout6574，启动日志确认高度闸门v1及待START。未重启主链、改MCU/BVA/Z控制或发START/解锁/飞行。仅修判据不解决物理少升；130cm仍可满足既有±12cm标准，不能称保证142。
- 下一步单独核对核心一键起飞与外部Z速度仲裁/补高交接，用户要求现高度设置不变；不把本项当作起飞高度已修复。

## 用户保留现有高度，仅查起飞软件逻辑（2026-10-23）
- 用户明确142/103现设置可打点，暂不修改高度，不应用11cm偏移；先排除硬件讨论，仅核对软件逻辑。只读拉取实机mission/BVA/pos_hold至flight_data/20261023_takeoff_logic_review/，findings.md记具体行号与证据。
- 142命令未发现120截断；起飞pos_hold.Z恒0、上排BVA也强制Z0，上层未按激光误差补足。爬升门槛目标-15（127）与超时目标-30（112）不一致；判稳±12持续实际0.3s而日志称2s，25s超时后最终误差<=15仍可放行；mission高度无新鲜度，拒绝相邻>100cm跳变可能保持旧值。后两项为明确可靠性缺口，但未证实导致物理停在120；Z0与核心起飞仲裁也仍未确认。
- 本轮未修改运行代码、部署、重启、改高度/参数或发飞行指令。下一项可准备统一到位/判稳/超时失败/测高新鲜度的局部候选；激光补高是独立控制接管变更，不能误称仅改判据就能保证到142。

## 换电恢复及用户明确高度优先级（2026-10-23 16:49）
- 用户澄清：融合高度按飞控到地面的理解作次级参考，激光/核心回传测距相对其照射表面（当前可为桌面）；不同参考面不能直接相减判定融合异常。用户估计机体中心142cm时飞控融合约148cm，属于位置关系估量，不自动作为激光补偿常量。控制以过滤后的激光为主，差异大时不让融合替换激光；当前运行mission/BVA/pos_hold订阅/height（激光add经三点中值），融合仅诊断，已符合来源优先级。先前仅凭138/51差值称“融合偏高”结论撤回；同参考面静置变化需另作判断。
- 用户换电后重新上电，boot980299d8-21f3-4168-ad85-058ebfeef0f0，主1075/UART1873，/proc实际SHA仍b14888…0167；F3方向0/270保持。已有trace1089至rec_20261023_161704.log（按进程路径取，不按文件名推时间）、fcstat2996；本轮没有重复启动fcstat。仅恢复被动observer3210至range_input_events_20261023_164857.jsonl，约17:18到期。
- 最新锁定静置：F3输入1cm、F2[-27,1,0]、/height1cm，unlock0/PWM0；不据不同参考面对融合值定异常。证据flight_data/20261023_battery_height_check/activation.json、ground_result.json。未刷写/复位/重启主链/发任务或飞行。下一步若需把142cm机体中心目标换算到激光目标，仍缺机体中心到激光测量平面的实际垂直间距，不套用飞控或摄像头的偏移。

## 方向0无动力抬高对照通过回传链（2026-10-23 16:38历史，参考面解释以上节为准）
- 用户“已太高”按上下文读取抬高段；捕获16:36:08.968–16:36:28.615约19.65s，178条F3（方向0/270，输入20–53cm、中位51），同期98条F2（add22–53cm、中位51，fus105–142cm、中位137.5，raw_state0），/height19–54cm、中位51。随后原始及add/height回落4–5cm；最新融合107cm，锁定/PWM0。未新发动作/改配置。
- 对照方向1前轮抬高53cm时add/height全0，恢复0后抬高约51cm能正常回传，支持本机保留方向0。该证据确认回传/任务测高通路恢复，不证明核心融合采用激光或142cm起飞恢复。融合偏差/漂移为待查问题，不能把状态0直接解释为激光无数据；实际核心固件版本仍未读回。
- 证据flight_data/20261023_height_ground_check/restore_direction0/lift_result.json、lift_events.jsonl；前轮方向1证据同父目录lift_analysis.json/lift_events.jsonl。下一步检查核心融合高度源/参数/参考基准，保持方向0、不叠加补高/试飞；当前observer4400约16:52到期。

## 已授权恢复方向0，核心add/height恢复近地测距（2026-10-23 16:34）
- 用户“授权”并接回J-Link。实时unlock0/PWM0后，使用工程J-Link/SWD1000kHz先停核备份完整1MiB；61308个加载字节确认片内为方向1版b75a6b…f4c5ec，再刷回0175b0…b68d4。回读0不一致、前64KiB仅0x08001530的01→00，随后复位运行。保留ANGLE270/F2/F3/ROS中值过滤；无主服务重启/参数调整/START/解锁/飞行。
- 同次香橙派boot（核心未断电）恢复0后的地面诊断：F3方向0/270、距离4–5cm；F2最新[63,5,0]，近8s fus62–65、add4–5、raw_state0，/height5。此前方向1输入4–5cm而add/height全0，且抬高53cm仍0；恢复0后add立即有新近测距值。因此本机应保留方向0，撤销先前按旧通用协议改1的适配判断。状态0不直接等同“没有测距数据”，也仍不证明核心融合已采用激光。原142cm少升原因和融合偏移仍未解决。
- 证据`flight_data/20261023_height_ground_check/restore_direction0/`含backup.log、flash_before_restore0.bin、readback_before.json、flash.log、flash_after_restore0.bin、readback_after.json、run.log、ground_result.json及核验脚本。ROS仍1881/b14888…0167，observer4400至range_input_events_20261023_162213.jsonl约16:52到期。下一步拔烧录器后锁定无动力再次抬高30–50cm、静置10秒，核对方向0时add是否跟随F3；不试飞。

## 发现官方另一套方向定义及恢复0授权历史（2026-10-23，已于上节恢复）
- 用户随后明确“授权”：恢复方向0/保留角度270及诊断、地面无动力验证已获授权，不再重复询问。16:31同boot、unlock0/PWM0、/height0，observer4400仍记录。Windows两次PresentOnly枚举均未发现J-Link/VID1366，故未连接调试器/刷写/复位；等待用户接回工程J-Link。恢复目标0175b0…b68d4哈希已核对，备份/刷写/运行脚本准备于flight_data/20261023_height_ground_check/restore_direction0；接好后应先备份并确认当前片内匹配方向1版b75a6b…f4c5ec，再刷回、回读、地面读取，不含解锁/飞行。
- 厂家官方《匿名光流V3.4》0x34协议定义direction0向下/1向上，模块ANGLE恒0；与本地20200813通用协议0水平/1垂直的定义不同。官方源https://www.anotc.com/wiki/匿名产品资料/匿名光流v3.4（本轮官方域搜索取得全文索引，直接open失败）；不能自动等同本机核心版本，但撤回“direction0一定错误、1就是正确修复”的判断。此前未确认版本就修改，依据不足。
- 现有香橙派/F407通道未回传凌霄IMU实际固件号；SysConfig.h的SOFT_VER17/HW_VER1为F407工程宏，不作为核心身份。未发送未知查询/抢占串口/改参数/刷写。详细证据flight_data/20261023_height_ground_check/protocol_compatibility_findings.md。
- 已复核可恢复的0175b0…b68d4诊断HEX，与当前b75a6b…f4c5ec仅同一方向字节0/1差异。建议只恢复0，ANGLE270不动，保留F2/F3和ROS过滤；同次上电再无动力抬高，核对add是否恢复。跨日期旧0与今1对比不足以最终定因；恢复0属于新的固件更改，待用户明确授权（仅地面对照，不含试飞）。

## 无动力抬高已记录：F3约53cm而核心add始终0（2026-10-23 16:24）
- 用户“已抬高”，被动记录捕获16:23:40.810–16:23:59.160约18.35秒原始测距20–54cm区间，165条F3、中位53cm（方向1/270）；同期92条F2，fus13–52cm/中位50，add全部0、raw_state全部0，/height全部0。随后原始测距回到4–5cm。电机一直0/锁定，未发动作或修改。
- 证据`flight_data/20261023_height_ground_check/lift_result.json`、lift_analysis.json、lift_events.jsonl（F3/F2/height/status原始切片）。抬高区间已超过仅近地3–5cm范围，不能用近距离误差充分解释核心add全0；方向1的核心兼容性必须重查。融合随抬高变化不等于证实采用激光，也可能含气压/惯导响应，状态0含义仍未知。
- 先前按通用协议认定direction0一定错误、direction1正确的判断过早：只能确认与该协议定义不符，尚未确认实际核心固件支持该定义。昨日方向0有add随高度，今日方向1重上电后add为0；跨重启/日期存在混杂，不直接认定方向是唯一原因。下一项应核对核心版本/方向规则，必要时单变量0/1无动力地面对照（恢复方向是新的刷写动作，需授权，不自动执行）；不试飞、不加补高。

## 重新上电仅恢复诊断（2026-10-23 16:22）
- 用户报告已上电，恢复先前暂停的高度检查。boot `1021f471-00b4-41cd-b150-5cd58a78dabc`，flight-link主1094/UART1881；/proc实际加载ROS二进制仍b14888…0167。实机F3方向1/角度270确认方向修正保留；未读芯片，不把昨晚回读哈希当作今日片内全量复核。
- 已有trace2751至`/home/orangepi/traces/rec_20261023_161425.log`、fcstat2900至/tmp/fcstat_pwm.csv、mission2932，本轮未另起这些进程/发START。确认无range observer后仅恢复被动observer4400至`/home/orangepi/traces/range_input_events_20261023_162213.jsonl`，约16:52到期；源脚本哈希fb67a4…fa48与候选相符。
- 地面最新：F3输入原始距离5cm、约53.01Hz；F2回传[fus=-33,add=0,raw_state=0]，/height0；近10s fus[-35,-33]，add/state均0，observer首条fus-38约25s后-33。unlock0、四路PWM0。不要将F3原始距离与F2 add混称同一数值；方向修正后近地输入与核心回传出现5/0差异，原因/有效状态待核对，不能认定已采用或拒绝。
- 本地证据`flight_data/20261023_height_ground_check/activation.json`、ground_result.json。当前只被动读取/恢复记录，未刷写/复位/重启主链/飞行。下一步请用户锁定状态手动抬高约30–50cm静置10秒，对照F3、F2及融合的响应；不先起飞/改目标/加补高。

## 用户已断电，今日任务结束历史（2026-10-22，现已恢复只读检查）
- 用户明确断电并结束今日任务；停止设备工作，不连接/启动/刷写/飞行。下文PID、服务和记录新鲜度均为断电前历史，下次上电只复核受重启影响的运行身份与记录器。
- 阶段结论：已确认激光任务高度约120–122cm而指令142cm；运行测距方向曾为0/270，与协议向下1/270不符，现已授权单字节修正并回读/实机诊断确认1/270。但修正后只做近地3–4cm静态读取，融合仍漂移、raw_state0含义未知，未做修正后无动力多高度或实飞，不能宣称起飞142cm问题已解决或方向配置是唯一根因。
- 下次优先无动力不同离地高度对照，检查激光与融合的响应，再决定是否实飞；不叠加提高起飞目标/外部补高。当前片内应为direction_candidate.hex b75a6b…f4c5ec（断电前已回读核验），ROS为range输入诊断版b14888…0167；桌面原工程仍未同步诊断及方向修正。

## 测距方向修正已授权刷入，实机提交1/270，融合仍漂移（2026-10-22 22:25，断电前历史）
- 用户明确“刷入”，范围为direction0→1及地面验证。刷前复核J-Link、实时unlock0/PWM0、高度4cm，使用工程J-Link/SWD1000kHz先停核备份完整1MiB Flash；所有61308加载字节与现行0175b0…b68d4诊断版一致后才刷direction_candidate.hex（b75a6b…f4c5ec）。回读全加载字节0不一致，前64KiB实际仅0x08001530的00→01，随后复位运行。未发START/解锁/飞行，ROS服务/参数未改。
- 实测最后8s70条新方向诊断，方向1、角度270、距离3–4cm，累计计数/运行时间差推得提交约51.13Hz；/height3，双高度最新[328,3,0]，近8秒fus318–328、add3、raw_state0。方向修正生效，但融合仍漂移，不能宣称核心已采纳激光或起飞高度已解决；近距离可能在传感器盲区也未证实。保持单一修改，不叠加补高/目标高度调整。
- 证据range_direction_fix/backup_20261022.log、flash_before_direction_20261022.bin、before_identity.json、flash_20261022.log、flash_after_direction_20261022.bin、flash_readback_result.json、run_20261022.log、pre_flash_live_result.json、post_flash_live_result.json。observer18692原range_input_events_20261022_221402.jsonl继续记录至约22:44；fcstat3109。下一步先拔烧录器，再安排锁定无动力的不同测距高度对照，不直接以此认为可飞；桌面原工程仍未改，重刷原工程会覆盖修正/诊断。

## 测距方向单字节修正候选准备历史（2026-10-22，已于上节刷入）
- 用户“下一步”承接“准备仅修正方向字段候选”；本轮按该范围仅本地准备，未连接/复位/刷写设备。`stage1_20261021/flight_effect_candidate/range_direction_fix/LX_FC_EXT_Sensor.c`只将direction=0改为1，angle270、距离、F2/F3诊断及ROS激光中值过滤保留；桌面原工程未改。
- 工程Keil ARMCC编译该翻译单元+原对象隔离链接通过；原源重新编译链接HEX与已刷0175b0…b68d4完全一致。修正候选61308个加载字节仅0x08001530的00→01，反汇编确认MOVS r0,#1随后写direction字段，非其他代码变化。证据build_result.json、binary_diff_result.json、direction_disassembly.txt、direction_only.patch及README。
- 可刷产物direction_candidate.hex SHA256 `b75a6bb629fd2a4cd8017be8a9e0ae180940c2e9f9f0ae55144413d97bf4c5ec`；回退为range_input_diag/range_input_candidate.hex。候选未证明核心采纳或空中改善。下一步刷入会改变核心收到的传感器安装方向，超出前轮只观察诊断的实际刷写范围，需用户确认该修改刷写及地面验证；不包括解锁/飞行。确认后复核片内基线/落地状态，刷写回读，再观察方向1/270及双高度，不预设raw_state必须变1。

## 0x34诊断已刷入，实测提交方向0/270、约50.56Hz（2026-10-22 22:16）
- 用户接好J-Link，按既有刷写授权先复核unlock0/PWM0，再用工程J-Link/STM32F407VG/SWD1000kHz备份完整1MiB Flash。刷写range_input_candidate.hex（0175b0…b68d4），回读覆盖所有61308个HEX数据字节，0不一致；刷前片内与旧双高度诊断HEX617888…4db9fd一致，随后复位运行。未发START/解锁/飞行指令，方向/控制配置仍未改。
- 先备份ROS原接收端至/home/orangepi/diag/backups/range_input_diag_20261022_221259，部署已隔离验证binary b14888a0d7c97427351da4b7bd23599121561923e926153af93f7847a1aa0167及对应.cpp/.hpp；标准重启一次成功。运行UART18273已按/proc实际加载哈希确认；被动observer18692输出`/home/orangepi/traces/range_input_events_20261022_221402.jsonl`，约22:44到期。原fcstat3109、trace1122保留。
- 实测近10秒89条诊断，字段方向均0、角度均270、距离均5cm，无0xFFFFFFFF；累计提交计数差494/运行时间差9770ms=50.56295Hz。/height5、双高度[269,5,0]均新鲜，unlock0/PWM0、目标速度0。证据range_input_diag/post_flash_live_result.json。确认运行时确有测距提交且方向与协议向下1/270不符；只证明提交接口被调用，不证明核心收到/采纳或少升原因。ALT_STA=0仍未定义。
- 同目录flash_20261022.log、flash_before_range_20261022.bin、flash_after_range_20261022.bin、flash_readback_result.json、run_20261022.log、ros_deployment.json、ros_activation.json和manifest记录部署身份。下一有界修改应单独准备direction=1候选并地面核对；当前尚未实施方向修正，不直接飞行。

## 0x34真实提交内容诊断候选准备历史（2026-10-22 22:08，已于上节部署）
- 用户确认机底激光经光流模块接F407 UART4。代码路径为AnoOF_GetOneByte→of_alt_cm/alt_update_cnt→General_Distance_Data_Handle→0x34→UartSendLXIMU；已刷诊断AXF反汇编确认direction=0/angle=270，协议向下定义1/270。历史代码也为0，不能据此宣称本轮少升根因；现有链路没镜像0x34，运行期提交/核心采纳尚未实测。凌霄手册3.5.3称激光+气压定高自动判断，无需手动开启；ALT_STA数值0定义仍缺，0x34距离无效标志为0xFFFFFFFF，不混同。
- 新本地目录`stage1_20261021/flight_effect_candidate/range_input_diag/`：在原发送调用后将实际0x34载荷7字节+累计32位提交计数+运行毫秒镜像为0xF3（15字节，最多10Hz、无新测距提交即无诊断）；不改方向/原帧/控制。ROS候选/fc_range_input_diag Int64MultiArray=[direction,angle_deg,distance_cm,tx_count,uptime_ms]，保留uint32无效标记，observer增加该话题；保留原双高度及激光中值过滤。
- 工程Keil ARMCC翻译单元编译+原对象隔离链接通过，HEX `0175b07b640b20c4d8fa44cf18e3e19aab13a7dad5358f2f25384e62d24b68d4`；旧对象重链接HEX与当前已刷双高度诊断HEX617888…4db9fd一致。原固件发送函数抽取后实际执行检查通过（原UART字节/次数不变、限频计数、异常形状、时钟回绕）。板上/home/orangepi/range_input_diag_20261022仅隔离构建，真实ROS节点只连私有PTY/domain76/localhost，字段、无效标记、异常帧、停报恢复、原双高度/过滤均通过；测试进程已结束。
- 证据该目录README.md、firmware_build_result.json、baseline_identity.json、check_result.json、pty_result.json、manifest.json及两个patch。运行服务仍8217/UART8338，实际加载SHA仍ac4697…cca03；未部署、未连真实串口/调试器、未刷写/复位/发飞行指令。诊断镜像只能证明F407调用提交，不能证明核心接收/采纳。下一步需把已拔下J-Link重新接好，之后才能安排诊断刷写与地面读取；本轮范围仅候选准备。

## 双高度起飞实测：激光约120cm、融合增量约144cm（2026-10-22 21:43轮，已落地）
- 21:49进一步只读核对：协议文本仅给一键起飞0–500cm，没有绝对/相对基准、融合/测距选择或状态0含义；不能宣称核心已确认使用融合增量。成功轮20261021_braking_trial在h132放行，之后升到142–152cm；同轮recorder t2226–2243所有有限z记录为0，故排除“已观察到航点Z补高”的解释（先前仅待验证假设）。旧成功轮没有双高度，无法追溯其融合偏差。
- 实机BVA源码SHA86f6be…510f1，已直接读取受影响分支：首个target_position.z作为z_home，abs(target.z-z_home)<8cm时强制out.data[2]=0。配合起飞pos_hold.Z=0，表明上排142cm阶段未对激光高度误差做外部闭环补足；换排才放行/放大Z。这是已确认控制缺口，不等于已定位闭源核心少升原因。当前只读，未改代码/发动作。建议下一修改候选为明确核心起飞→激光补高的单一垂直控制接管，并达到实高后才放行XY，先核对接口正常/失败/恢复约束，不能简单放宽门槛或固定抬高142。
- 用户确认已降落。保存本轮原始ROS事件切片（21:42:49–21:43:30）、mission.log、实机route_mission.yaml与analysis.json至`flight_data/20261022_dual_height_trial/`。唯一mission_cmd=[1,142]，t0=1792676574.5441942；命令类别cmd1→5。原航线起飞/首段/上排142cm，下排103cm。
- 实测起飞前3s fus中位69cm/add1cm；起飞12–15s fus213/add121；18–21s fus212/add120；21–24s fus213/add122、add短时最高129。相对起飞前融合增量约143–144cm，而激光增量约119–121cm。所有分段target_velocity.Z范围0；这支持核心按融合增量完成起飞的假设，但没有内部目标或所用高度回读，不能认定根因。落地27–30s fus91/add5，静置漂移背景保留。
- 实机任务判据：爬升循环在h>=142-15=127时可退出；30s超时后甚至仅h>=142-30=112可继续打印“爬升到位”；后续稳定条件±12cm持续实际0.3s（日志文字仍称2s），最终检查±15cm。此次21:43:17打印h129，21:43:21因unlock0中止，未挂航线。判据宽松不等于产生核心高度偏差；未改控制/阈值/固件或再飞。
- 下一步先核对核心一键起飞高度基准与现有外部速度接口，避免直接提高142、放宽容差或把漂移融合值作为任务高度。现场保持落地。

## 激光孤立跳变过滤已部署（2026-10-22 21:38）
- 21:41就绪复核：mission3141日志仅等/gs/start；observer3458持续更新（约21:56到期）。发现21:26:33已存在fcstat3109，本轮21:26:49又新增3459；仅停止本轮重复3459，保留3109/同一CSV，后续不要沿用3459。前述重叠写入窗口的fcstat CSV需警惕双实例写入，observer JSONL不受此问题影响。桌面OneKey_Takeoff仍原样写入142低高字节，协议只给0–500cm未说明核心所用高度；下一次若获现场触发，将利用新增双高度记录比较起飞与悬停，不预设定高依据。
- 用户确认仍落地，3074是激光偶发跳变并允许滤除。仅香橙派UART接收端/height的4:1抽取后加入三点中值；约50Hz时延迟约20ms，启动首三样本后发布；相邻接收间隔>200ms重置窗口并重新等三样本，不定时重发旧高度。200ms是候选策略参数、非实测设备超时。原/fc_height_diag保持原始值；未改F407/核心内部高度控制、目标高度、坐标补偿或其他参数。
- 源基线与实机逐一核对，独立/home/orangepi/laser_height_filter_20261022完整构建通过。真实helper检查正常/正负尖峰/阶跃/斜坡/断流恢复/连续异常边界通过；实际ROS节点只连私有PTY、domain76，3074孤立尖峰拒绝、真实阶跃通过、停报无旧值重发、断流重新预热、原诊断保留通过。连续异常可通过，不能当作传感器有效性检测；未验证空中效果。
- 地面unlock0/PWM0后备份/home/orangepi/diag/backups/laser_height_filter_20261022_213649；部署二进制和对应源，标准重启仅一次。重启客户端30s超时但服务随后自行active/running（等待脚本实际最多30s，早先服务注释90s过时）；未重复重启。最终主8217/UART8338，/proc实际加载SHA `ac46972d851c040f0e78be330c451fb1446e7f8fce1267905ebf3fd7e09cca03`；近10s /height497条且1–2cm，双高度仍新鲜，unlock0/PWM0。未发送START/解锁/飞行；原任务3141及observer3458保留。
- 本地候选/证据`stage1_20261021/flight_effect_candidate/laser_height_filter/`含filter_only.patch、check_filter.cpp、check_pty_filter.py、pty_result.json、deployment.json、activation.json及新增filter头。下一步关注静置过滤实效及原起飞高度来源，不用此ROS过滤解释或宣称修复核心定高问题。

## 地面起点重新上电，双高度记录恢复（2026-10-22 21:27）
- 21:30用户确认始终静置，持续记录发现fus未收敛：observer首条2cm，约240秒后54cm，最后20秒40→54cm；add最后20秒1–2cm，unlock0/PWM0。此前“可能启动零点误差、等待收敛”仅假设，现有静置结果不支持已收敛。近60秒301条中1条add3074cm，前后相邻约0.2s均2cm；仅ROS采样孤立异常，不能认定源端只持续0.2s或已定位原因。raw_state全0含义仍未确认。证据height_diag/static_drift_20261022.json；先分析融合漂移及激光异常保护，不据此调起飞目标/安排重复试飞。
- 用户确认激光为主要测高来源；桌面约126cm高，之前fus139/add1不能单独判断异常。机体中心在飞控下方3–4cm、扫码摄像头在中心下方6–7cm（用户估量，未标定）。不改变/height语义或引入固定高度补偿。
- 实机boot `8b19c0c1-3903-4f3b-a44d-9fdba92c4949`，flight-link1116/UART1887，运行二进制SHA109fc010…97f16；双高度近10秒50条，fus范围-5至-3cm、add1–2cm、raw_state0，/height2cm；unlock0、四路PWM0。证据height_diag/power_on_ground_20261022.json。融合高度有效性/状态定义/核心起飞高度来源仍未定，地面数据不证明空中性能。
- 重启后/tmp observer和fcstat已丢失；已将被动observer保存至/home/orangepi/diag并恢复PID3458，输出`/home/orangepi/traces/height_diag_events_20261022_212649.jsonl`，约21:56到期；fcstat3459至/tmp/fcstat_pwm.csv。主trace1122至rec_20261022_211704.log；最后检查mission3141存在（本轮未启动它），未发START/解锁/起飞/复位。下一步有界实飞前确认现场准备及原任务状态。

## 诊断固件已授权刷入并回读一致，双高度实机链路已通（2026-10-22 21:14）
- 用户明确授权助手通过已连接烧录口刷写。使用工程配套`D:/Keil51/Keil5 C51/ARM/Segger/JLink.exe`，STM32F407VG/SWD/1000kHz；刷前实时高度1cm、unlock0、PWM0。先保存完整1MiB Flash，再写既有diagnostic_candidate.hex（617888…4db9fd），回读其60544个HEX数据字节全部一致，随后复位运行；未发START/解锁/飞行指令。
- 已测：刷前片内所有原HEX数据字节与original_rollback.hex一致，也解释了此前诊断消息为0；桌面原工程源及HEX仍为原版，候选改动未写回桌面工程，直接从原工程再次下载将覆盖诊断版。
- 已测：21:14香橙派/fc_height_diag累计84条、近10秒49条约5.007Hz，最新`[fus=139,add=1,raw_state=0]`；原/height=1cm，unlock0/PWM0、目标速度全0。地面两高度明显不同，但raw_state含义、fus参考基准及核心起飞所用高度仍未确认，不能据此认定空中差20cm或调整控制。
- 证据均在`stage1_20261021/flight_effect_candidate/height_diag/`：flash_20261022.log、flash_before_20261022.bin、flash_after_20261022.bin、flash_readback_result.json、run_20261022.log、post_flash_live_result.json。被动记录仍为PID9983、height_diag_events_20261022_205807.jsonl，约21:28到期；服务/任务未重启。下一步先据地面双高度与状态定义决定有界测量，不自动起飞。

## 双高度ROS接收端已部署，固件由用户自行刷写（2026-10-22 20:58，以下为历史）
- 用户授权刷入诊断固件及部署接收端，随后明确“我自己烧录”：助手停止固件刷写流程，未连调试器/复位/刷写；Windows未检测到工程配置J-Link。提供height_diag/diagnostic_candidate.hex（617888…4db9fd）及original_rollback.hex（原08187c…23bc53），等待用户确认烧录完成，不能当作片内已更新。
- 地面h3/unlock0/PWM0后，备份ROS原二进制和源至`/home/orangepi/diag/backups/height_diag_20261022_205651`；部署隔离已测候选binary109fc010…97f16至原build目标，install符号链接不变，同步.cpp/.hpp/codec源。标准重启主链成功，主PID9442/UART9562；读取/proc/9562/exe哈希确认实际加载候选。证据height_diag/ros_deployment.json、ros_activation.json。未改飞行参数/原height语义，未发START。
- 被动记录器切换为/tmp/takeoff_observer_height_diag.py PID9983，输出`/home/orangepi/traces/height_diag_events_20261022_205807.jsonl`（30分钟有效），包含新诊断话题；旧observer已停，原主trace/fcstat保留。烧录前没有/fc_height_diag消息属预期，不代表链路已验；下一步用户自行烧录并上电后先验证地面双高度和状态，再安排物理复测，不直接飞。

## 双高度隔离构建/协议测试完成（2026-10-22）
- MCU：Keil ARMCC修改翻译单元编译并与原对象隔离链接，生成height_diag/diagnostic_candidate.hex（SHA256 `61788897e15aee15d1f8be9d2662569a274e91d6893785ecfb91cda2114db9fd`）及AXF；非全量干净重编译。原对象基线重链接HEX与原08187c28…23bc53逐字节一致，见firmware_artifact_identity.json。未刷写/读芯片。
- ROS：实机当前四个相关源文件哈希与10/21快照逐一相同，独立目录`/home/orangepi/height_diag_20261022`完整构建通过，候选二进制SHA109fc010…97f16。首次安装漏打包launch，补齐原launch后增量构建完成；未改业务代码绕过安装。
- 协议运行：真实ROS候选只连接/dev/pts/0、domain76、localhost1，检查FD未开/dev/ttyS3；signed极值、原始state、短/长帧与校验错误拒绝、停报不重复缓存、恢复、原/height独立通过。原始新增固件转发块抽帧/异常长度/恢复实际执行通过。C++解码器断言已用g++执行；详见height_diag/isolated_runtime_result.json及check_result.json。测试子进程已结束。
- 主服务仍PID1116 active/running，原UART源码15bfd4…8720不变；诊断候选未部署、未连真实飞控串口、未发任务/飞行指令。下一步需要用户明确固件写入授权后，才按配置Keil后端处理刷写和对应ROS部署；写入前还需确认连接/落地状态，不能仅凭测试通过视作物理安全证明。

## 双高度诊断本地候选已准备（2026-10-22）
- 范围`stage1_20261021/flight_effect_candidate/height_diag/README.md`：仅诊断，不改/height、起飞高度、Z/XY或任务逻辑。基于用户确认桌面工程ANO_DT_LX.c，每10条新核心0x05转发一次原始9字节为0xF2；无核心更新即不发。ROS快照候选新增Int32MultiArray /fc_height_diag=[fus,add,state]，严格9字节；被动记录器同步扩展。raw_state不擅自解释。
- 已检查：配置Keil ARMCC及该工程IncludePath/Define编译实际修改固件翻译单元通过；C++解码器边界测试载体编译通过（断言尚未运行）；实际记录回调离线运行验证正负32位极值、原始状态和顺序通过。manifest.json、firmware_compile_result.json、check_result.json记录范围。
- 未完成：完整固件链接/HEX、完整ROS构建、真实协议正常/短帧/停报恢复验证及双高度实测。未连接调试器、刷写、部署、修改原工程或实机。下一有界动作在隔离目录完成ROS构建/协议测试；部署前复核当前实机源码身份，固件写入需单独授权。不再重复同条件飞行。

## 满电起飞仍停约1.2m（2026-10-22 20:34轮）
- 用户确认最后刷写使用上述桌面工程（用户提供身份依据，非芯片读回）。进一步核对真实工程：只有AnoDTRasp调用OneKey_Takeoff，无另一条固定120cm起飞路径；User_Task仅解锁并等0x66。新线索：核心0x05原始9字节含s32 fus/s32 add/u8 state，但AnoDTRasp只调用AnoPTv7SendALT(fc_alt.st_data.add)，转为2字节给香橙派。任务/现场约1.2m不能证明核心内部融合高度同为1.2m；协议称fus为融合对地高度，add为附加测距。尚不知核心一键起飞使用哪一高度，也没有本轮fus记录，不能认定二者差20cm。下一有界修改应为仅诊断的双高度+测距状态透传，保留现有/height语义；先本地准备，刷写需用户另外授权，不直接换/height或动高度门槛。
- 后续代码核对见该目录height_chain_findings.md：142低高字节编码/参考解码/OneKey_Takeoff传参未见120截断。关键更正：参考固件把rt_tar直接转发为/fc_target，故其Z=0不是核心内部目标回读，不能据此排除外部流与一键起飞交互；参考takeoff_if在cmd1=5后立即放行速度。实际刷入固件身份与核心规则未确认，成功轮也有Z=0，交互冲突仅假设。成功轮fc_target.thr中位579、失败571，参考代码映射为遥控输入，不是实际电机油门；不据小差异定因。下一步先核对现有固件构建身份与核心接口资料，不盲停速度流、不重复同条件试飞。
- 再核对用户此前指定的真实桌面F407工程：build/ANO_LX.hex SHA08187c28…23bc53、axf6225a2f8…374682、AnoDTRasp.c cc37337b…373f89均与10/21已记录哈希一致。该工程ANO_LX.c也在takeoff_if后将m_speed_z写rt_tar，XY另受flown_flag限制；不再用10/18归档工程替代该源。仍缺最后刷入记录/芯片内容身份及闭源核心版本，现有文件只能证明本地产物未变，不能证明片内版本。未连接调试器/读取芯片/刷写。
- 用户确认已降落且目测约1.2m。数据保存`flight_data/20261022_full_battery_takeoff_trial/`含mission/hold/fcstat/yaw、rec、事件日志；不再把电量不足当作充分解释。
- 同步事件：20:34:45.269仅1条mission_cmd=[1,142]；板上journal 45.507确认串口节点Sent 0x66 height=142，45.792任务确认cmd1→5。起飞至失败窗口height最高118，起飞后5/10/15秒末1秒高度中位103/113/116；用户随后降落，最终h1不是悬停高度。target_velocity.Z共656样本及fc_target[6]共1831样本在起飞前5秒至失败窗口均0；没有发现ROS层下压Z指令，fc_target[6]不是内部高度目标。
- 本轮重复START均被忽略，未进入航点，不能验证反向提速平滑。源码mission高度142、串口按16位低高字节编码；本地参考OneKey_Takeoff原样编码height_cm，未见120限幅，但这不证明板内已刷固件/IMU内部实际接收目标。未改参数/重启/发命令。
- 下一有界动作核对飞控端实际接收的起飞高度和起飞模式与外部速度流的交互；现有cmd1=5只证明起飞命令类别受理，不能证明142目标已被核心采用。没有该内部观测前不盲加Z补偿或降低门槛。

## 满电换电恢复历史（2026-10-22 20:33，北京时间）
- 用户说明刚才起飞失败为电池不足/动力不足，现换满电电池；作为现场判断记录，未测电压，不将其写成独立遥测证明。保留高度门槛和控制参数。
- 新boot `b0475d8c-948a-4ac4-ae47-bb27080783f7`；主链active/running PID1116/BVA1902，BVA86f6…510f1、mission5310…d617、dead_reckon de8e…0c73哈希正确。任务PID2864就绪等原触发；fcstat2832更新0.35s，高度3/unlock0/PWM0。
- 当前主trace PID1120输出`/home/orangepi/traces/rec_20261022_201705.log`，文件名为同步前时间；被动observer已恢复PID3213，输出`/home/orangepi/traces/takeoff_events_20261022_203336.jsonl`，30分钟有效约至21:03。未重启链路/修改参数/发送START；下一步按原流程复测首段起步反向提速平滑。

## 换电上电恢复历史（2026-10-22 20:29，北京时间）
- boot ID `ba9793f5-50fc-447c-a1ff-b7d5590372de`，主链路active/running主PID1111/BVA1904；BVA86f6be…510f1、dead_reckon de8e…0c73、mission531050…d617保持正确。启动日志平滑10.0，无ERROR/进程退出，无旧目标z_home初始化。
- 已有唯一主trace PID1120输出`/home/orangepi/traces/rec_20261022_201704.log`（文件名为同步前时间，按进程路径取）；fcstat PID2944、任务PID2976已自行就绪，未重复启动。仅恢复临时被动observer PID3158，输出`/home/orangepi/traces/takeoff_events_20261022_202855.jsonl`，有效约至20:59；启动证据post_battery_restore_20261022.json。
- 地面高度1/unlock0/PWM0，fcstat更新<0.8秒，事件height/fc_target/fc_vel持续更新、stderr空；当前未收到target_velocity，不当作零指令。未改参数、未发START或起飞。下一步按原流程复测首段起步平滑，用户降落后收取以上本次启动路径。

## 反向提速豁免修正已部署（2026-10-22 20:23，北京时间）
- 用户“下一步”授权后，仅收紧smooth_xy负点积直接放行条件：新速度幅值不高于上一拍时才允许反向纠偏立即执行；反向提速也走10cm/s²向量变化率限制。降速、零指令、35/18cm收口、Z/yaw、任务互斥不变，偏航候选未部署。实际源码body_vel_adapter_reverse_accel.py、差异reverse_accel_only.patch。
- check_reverse_accel.py真实on_cmd验证本轮向量复现、90拍反向加速连续限制、减速/等幅反向/零立即执行、35/18收口、低高/无航向/恢复、AST仅smooth_xy变化通过；reverse_accel_check_result.json。静态检查不是实飞效果证明。
- 新鲜地面遥测h5/unlock0/PWM0后备份部署；BVA SHA256 `86f6be707f525ec966c7e792fdaf2332b69661bb4c2c9086bab70fd47ac510f1`，备份`/home/orangepi/diag/body_vel_adapter.py.bak-reverse-accel-20261022T202238`。停旧waypoint PID6538后标准重启flight-link（未发START），20:23:39最终active/running、主8757/BVA8905、哈希正确、startup无错误、无旧目标初始化z_home。证据reverse_accel_deployment.json、reverse_accel_ready.json；首次检查处于start-pre，未重复重启，等待原操作完成后检查通过。
- 下一有界动作：原自动流程再测首段起步，用户手动降落后比较0.83→15阶跃是否消失；不同时叠加偏航。被动事件记录仍至20:41，飞后按当前进程路径取主trace。

## 平滑复测反馈与遗漏（2026-10-22 20:13轮）
- 用户反馈悬停/90cm改善不大。数据已保存`flight_data/20261022_smoothing_trial/`含rec、任务、wpn、hold、fcstat、事件JSONL、route；first_leg_analysis.json含源哈希。本轮20:13:03.154起飞，08.545判稳，12.360 takeoff到达，23.360 A进-0到达(d4.4cm)，首段约10.7s（按takeoff hold0.3s后估算起点）；重复START已被忽略，无锁失败。
- 已测起飞保持63样本偏航指令-9.545～+9.499°/s、RMS6.674（上一轮57样本RMS4.434）。固定历史PD下将阻尼分量限幅2，回算RMS4.122、峰值约6.545，但非闭环预测；候选已基于新start_guard重制mission_start_guard_damp_limit.py，未部署，原旧候选不能覆盖新互斥修复。
- 关键发现：20:13:12.379机体系指令约(-0.681,0.468)，12.480约(8.731,12.197)，速度0.826→15cm/s，首段起步仍阶跃。smooth_xy对任意负点积直接放行，允许从小幅反向追起点直接跳到正向巡航；提取实际方法用该向量复现原样15输出（记录为机体系，未直接采到地图平滑输入，分支关联尚有观测限制）。较大后续变化集中减速，原减速豁免仍在。不能宣称起步平滑已有效。
- 下一优先修平滑反向豁免的范围：保留降速/零指令即时响应，反向大幅加速也需受限；先不部署偏航候选，避免同时改两项。当前板上仍原平滑41f6…及mission互斥5310…，本轮未改运行参数/重启/发送START。事件记录本轮起飞至判稳target_velocity.Z与fc_target[6]均0，实测高度升到133；上一轮低高度具体原因仍不能由本轮成功唯一归因竞态。

## 起飞同步被动记录已就绪（2026-10-22 20:11，北京时间）
- 新增本地`flight_effect_candidate/takeoff_observer.py`，仅ROS订阅，无发布器/服务或控制指令；语法检查通过。临时板上`/tmp/takeoff_observer_20261022.py` PID6060，domain6，持续1800秒；输出`/home/orangepi/traces/takeoff_events_20261022_201140.jsonl`，错误日志`/tmp/takeoff_observer_20261022.log`。断电后需重启，不随主服务自启。
- 记录每条消息的墙钟ns及单调接收时间：height、target_velocity、target_velocity_nav、fc_target、fc_target_vel、fc_vel、fc_status、active_controller、mission_cmd；不是UART实际交付或飞控内部高度目标。验证11秒数据：height/fc_target/fc_vel各542条、fc_status1080条、target_velocity221条，文件更新年龄0.05s，stderr空。无新消息的话题不能视作0；当前目标Z=0，地面高度3/unlock0，任务仍就绪。
- 没有改飞行参数或重启任务；下一步在记录器30分钟有效期内沿原流程复测，重点观察能否越过约1.2m继续爬升；若仍只悬停，由用户按原方式降落后对齐事件日志与任务日志。不再加聊天启动口令。

## 重复START竞态已修正部署（2026-10-22 20:04，北京时间）
- 用户授权“修正”。本地mission_start_guard.py基于实机原版9cddeabe…2bb1，仅修改启动生命周期：on_start先非阻塞占用线程锁并置running再启动线程；重复触发不创建线程；线程启动失败释放；run_traverse外层finally在原任务/文件锁清理结束后恢复状态并释放；抢文件锁失败关闭本次FD。未改爬升门槛、控制参数、路线或偏航候选。
- check_start_guard.py提取实际触发/包装方法，100次重复、运行中重复、正常完成再启动、线程启动失败/任务异常恢复、AST范围通过。证据start_guard_check_result.json、start_guard_only.patch；离线调度测试不是实飞证明。
- 地面新鲜遥测h=2/unlock=0/PWM0后备份部署，mission SHA256 `53105085bef43ec05d8466deabab61f842aad1d1c0118075d63b7d7c3570d617`；备份`/home/orangepi/diag/mission_node.py.bak-start-guard-20261022T200401`。仅按原脚本重启任务节点PID4508，20:04:05就绪等原触发，未重启主链路、未发送START。证据start_guard_deployment.json。
- 尚未解决：本轮爬升判定未通过的高度原因，不能保证此竞态修复就恢复航点。下一有界动作先离线对齐已保存高度曲线/任务采样，再决定复测，不降低门槛绕过。
- 后续离线核对：fcstat记录高度主要停在约121–125cm，130只出现连续2行(约0.1s)。任务同源`/height`的flight_log整份记录最大125cm，未记录>=127；两记录器相对t零点不同，不能按相同t直接对齐。127cm为爬升等待退出门槛，后续判稳又要求距142不超过12cm；仅降低第一门槛还会卡下一阶段。已证实任务可见高度未稳定满足门槛；实际高度/飞控爬升偏差与输入差异仍未判定，需要现场高度描述或下一轮同步观测，暂不调整门槛。
- 用户目测约1.2m，与遥测相符。进一步对比10/21 21:54成功轮：相同mission源、同为单帧0x66[1,142]且确认受理；pos_hold代码输出[vx,vy,0,vyaw]，本轮未进入导航/BVA，XY平滑无已知执行路径影响本轮爬升。按各自trace首次unlock对齐，+10秒高度中位本轮106/成功132cm，+20秒121/144cm（首次unlock不是起飞命令时刻，不混用）。本轮trace的z字段起飞时多为NaN，不能用其证明线上Z指令为0；实飞固件内部高度目标和停止爬升原因仍未知。下一步应补足起飞期间实际target_velocity与FC目标的同步记录，再复测，不盲调高度门槛或Z补偿。

## 本轮未进入航点（2026-10-22 20:00–20:01，北京时间）
- 用户反馈触发后只悬停。mission：20:00:05.809发起飞142cm、06.584确认受理；37.398报爬升未到位(h=47)中止，wpn.log不存在，未进入BVA首段，不能评价XY平滑。最新地面遥测unlock=0/PWM0。
- 数据已保存`flight_data/20261022_smoothing_takeoff_trial/`（当前rec、mission、pos_hold、fcstat、yaw/BVA CSV、route），air_summary.json为解锁样本统计，最高130cm。代码等待height>=127，每0.2s检查；超时后height<112才中止。fcstat最高130仅2行(96.22/96.27)，不能据采样认定任务观察到该值；实际高度不足与高度输入问题待区分，不能只凭最终47推断全程低空。
- 重复START在20:00:01.913/.915到达，先启动两线程；第二线程抢锁失败却把共享running置False。源on_start在线程内才设running，存在确定的重复触发竞态；锁报错不能据此认定有外部hover_test。本轮未改代码或重启，后续优先独立修正触发互斥，再核对起飞高度曲线，不降低爬升门槛绕过问题。

## 起点重新上电（2026-10-22 19:59，北京时间）
- 用户断电摆到起点后再次上电。只读核对boot ID `0d0f93c5-23e1-4a47-b419-a6ca692cbbd1`；flight-link active/running，主PID1079、BVA1887，三个文件哈希与下节一致。任务PID2884已就绪等原触发，无需重新启动。
- 唯一flight_log PID2702输出`/home/orangepi/traces/rec_20261022_195836.log`，唯一fcstat PID2852输出`/tmp/fcstat_pwm.csv`；检查时更新年龄分别0.052/0.166s。最近20秒高度均1cm、unlock=0/PWM0，未见该窗口异常高度或进程退出。此为地面准备核对，不是实飞验证；本轮未改代码/启动进程/发送START或起飞。
- 下一步按原自动流程复测起飞至首段90cm，用户手动降落后收集上述当前trace，不沿用19:48旧路径；只比较已部署XY平滑效果。

## 上电恢复历史（2026-10-22 19:50，北京时间）
- 已核对boot ID a1f9038c-60f3-41df-b016-1ca63229b7d3；flight-link active/running、主PID1122、BVA1917。BVA SHA41f6c970…a6f7b、dead_reckon de8e528…0c73、mission原版9cddeabe…2bb1均保留；启动平滑10.0，偏航候选未部署。
- 已确认被动flight_log PID2857至`/home/orangepi/traces/rec_20261022_194854.log`、fcstat PID3141；最初进程筛选误漏python -u导致启动重复，已仅停止本轮新增3324/3325，保留先存实例，两文件更新正常。通过原脚本恢复A1–A6任务PID3893，仅就绪等原触发，未发送START/起飞/复位。route_mission.yaml由任务实时生成，开机初始不存在正常。
- 初检高度有5783cm异常且位置环丢弃；随后最后20秒高度2–4cm，fcstat unlock=0/PWM0；原因未明，不能当已修复。19:50:27两记录文件更新年龄<0.04s，证据flight_effect_candidate/post_power_on_20261022.json（processes筛选漏掉python -u，记录器PID另查确认）。下一步沿原流程复测起飞至首段90cm，用户手动降落后分析平滑效果；不叠加偏航修改。

## 断电历史与本地候选（2026-10-21，22:07状态之后）
- 用户确认香橙派已断电。本轮只做本地准备，未连接、部署或重启；下文PID和记录器路径均为断电前历史，上电后仅重核受影响身份与记录器。
- 已实现但未部署：`stage1_20261021/flight_effect_candidate/mission_hold_damp_limit.py`仅在原任务POSHOLD_ENV增加POSHOLD_DAMP_VMAX=2.0，将偏航阻尼叠加限幅由默认5降为2°/s；保留阻尼增益2.2、偏航PD、XY和接管时序。候选SHA256 `02d1a19a35bc9ba88cc3c63aaed168f4ecafbceeaff97dbd1d3f1c16d9984263`；差异见同目录hold_damp_limit_only.patch。
- 离线已验证：prepare_hold_damp_limit.py执行真实YawDamper类，使用21:54轮yaw_damp.csv的57个空中样本、固定历史陀螺和PD输入回放。原输出复现误差<0.01，语法、单参数范围、复位及强输入限幅检查通过；总偏航指令范围-7.244～+7.903变为-4.244～+4.903°/s，RMS 4.434→2.854。证据hold_damp_limit_replay.json。仅阻尼分量限幅2，总输出仍可超过2；闭环效果未测，减弱阻尼也可能增加漂移。
- 下一有界动作：上电先核对并实飞已部署BVA平滑版（SHA41f6c970…a6f7b，dead_reckon仍de8e528…0c73）；平滑尚未实飞验收。之后再决定单独部署此偏航候选，部署前核对实机mission源与本地基线一致；不同时叠加两项，不改接管等待。

## 断电前状态：首段抖动优化已部署，待复测（2026-10-21 22:07，北京时间）
- 用户最新反馈：90cm段机身一顿一顿、悬停仍不稳，要求先集中优化起飞至首段结束。已保存`flight_data/20261021_braking_trial/`，first_leg_analysis.json含时间线、测量范围、源日志哈希；仅该阶段分析，不展开扫码/换排。
- 已测：21:54:08.514起飞，21:54:16.557开始首段（约8.04s），21:54:28.356到A进-0（首段11.799s，日志到达误差1.5cm）。记录重建的地图XY指令在起步0.1s内跳约16.05cm/s、距目标44.56cm时转向跳6.44cm/s；这支持指令突变耦合抖动的推断，非唯一物理根因证明。悬停段pos_hold偏航指令-7.244～+7.903°/s，最后hold CSV到导航激活约2.917s；这两项留待下一独立调整。
- 已实施单项：`stage1_20261021/flight_effect_candidate/body_vel_adapter_smooth.py`在地图XY最终正常指令增加BVA_XY_ACCEL=10cm/s²（单调时钟，初步参数）；减速、反向纠偏、零指令立即执行，低高度/无航向等原保护优先并清平滑缓存。35/18cm收口、PID、起飞等待/偏航参数、Z链及路线未改。check_smoothing.py受影响5组离线通过，smoothing_only.patch、smoothing_check_result.json为证据；这是指令变化率，不是物理加速度/实速上限。
- 已部署/恢复：BVA SHA256现为`41f6c970e7ba03c7d0702151c48df613e56be393fd94abc8e21a0f088d4a6f7b`，之前35cm版备份`/home/orangepi/diag/body_vel_adapter.py.bak-smoothing-20261021T140334Z-170d51`；dead_reckon保持de8e528...0c73。首次启动发现旧waypoint PID4067仍发103cm目标，误置新BVA z_home=103；落地锁定后仅停该旧航点进程，再标准重启链路。最终22:07:16 active/running、主PID7918、BVA8066、平滑10.0日志、正确双文件哈希、启动journal无错误，且未被旧目标初始化z_home。证据smoothing_deployment_result.json、smoothing_activation_result.json（首次）、smoothing_cleanup_result.json、smoothing_ready_result.json（最终）；未发START/起飞/复位消息。后续重载前注意已结束任务的旧waypoint残留，不让103旧目标占起飞基准。
- 下一有界动作：沿用原自动流程，用户手动降落后比较首段起步/转收口顿挫是否减轻，再单独考虑pos_hold偏航或接管等待。当前记录进程路径仍`/home/orangepi/traces/rec_20261021_211704.log`（文件名是同步前时间）；取数据按进程路径，不猜mtime。不要重复旧检查，不自动叠加PID/偏航改动。

## 换电恢复历史（2026-10-21 21:52，北京时间）
- 用户重新上电后，已只重核受重启影响的版本、主链路与记录器。新boot ID `8f401930-072a-4ada-92ff-5aa418589465`；flight-link active/running、主PID1103。以原start_mission_subset.sh恢复A1–A6任务节点PID3412，日志仅“就绪，等/gs/start”；未发送START/起飞/复位指令。原自动起飞触发流程保留，不加聊天口令。
- 刚完成的单点改动已部署：BVA在剩余距离≤ff_far（当前35cm）时将本段_cor_left置0，立即进入原35/18cm分档，回摆不恢复巡航；仅新目标重置。没有加平滑、改计时速度、调PID或动Z/yaw/路线。35cm边界可能仍输出15cm/s，进入分档不等于立即刹停；18cm内交回原PID。
- 换电后实查BVA SHA256仍为`22e948c05a168b4c31d1fb74a6aae417f4eed56546edef3e1d4f5f25c00b2678`、PID1897，源码为本地`stage1_20261021/flight_effect_candidate/body_vel_adapter_brake.py`；dead_reckon SHA256仍为`de8e52801bd328c37114317b9000ddefa9d28b40fed7f14937767eb7e2750c73`、PID1863。板上前一BVA备份`/home/orangepi/diag/body_vel_adapter.py.bak-braking-20261021T134625Z-4bbab2`。当前记录器PID1106输出`/home/orangepi/traces/rec_20261021_211704.log`，fcstat唯一PID2967输出`/tmp/fcstat_pwm.csv`，21:52:44均新鲜，h=3、unlock=0、电机0。
- 已验证：check_braking.py复用原回调外壳，仅新受影响分支5组通过（边界、回摆锁存/新目标恢复、近区起步、原禁用/计时结束分支、低高度/无航向/恢复、AST范围）；braking_only.patch、braking_check_result.json为证据。断电前21:46部署/重启和21:47启动核对见braking_deployment_result.json、braking_activation_result.json，旧主PID17558/BVA17707仅历史。换电后证据post_power_on.json、post_power_on_restore.json、post_power_on_ready.json。首检板上时钟为21:17，随后NTP同步到21:51；初检UTC时间不能作真实时序，当前trace文件名也含同步前时间，后续按记录器进程的输出路径取日志，不能按所有文件mtime猜最新。未重复旧TF/控制/UART离线检查，35cm改动未实飞验收。
- 历史速度对比已核对：10/7完成12格、10/19完成A面同样出现目标15以上的观测峰值；首段末约2s指令约14.6/14.4/14.8，SLAM位移均速约15.9/16.4/23.2（对应10/7、10/19、本轮）。飞控上报速度偏低并非本轮新增，不能据1.92s窗口认定固定比例故障；本轮位置/航向重复1.68s，定位/记录滞后原因未明。最新顺序是先验35cm退出巡航的效果，平滑另轮；复测沿用原自动起飞，不增加回复启动口令。起点摆好再上电，用户中途手动降落后分析相关日志。

最新复测（2026-10-21 21:28–21:29，北京时间）：用户反馈“悬停较久且不稳，随后加速飞向航点并飞过头”。本轮只保存/分析日志，未改参数或重启。证据`flight_data/20261021_origin_fix_trial/flight_analysis.json`及同目录rec/mission/wpn/route/fcstat/pos_hold/BVA/阻尼CSV、stage_journal.json。
已量化：21:28:55发起飞；21:29:00.923判稳（5.918s）；21:29:04.027导航激活；21:29:07.737完成takeoff航点后才进首段（起飞起算12.732s，判稳后额外6.814s）。pos_hold最后CSV到导航激活约3.107s，随后回起点容差内又3.710s；日志支持接管启动/重收起点造成等待，不能据CSV断言UART断流。首段指令突至约15cm/s，滤波SLAM速度观测峰值27.66cm/s（非独立真实地速）；XY/yaw最长重复1.68s，定位停滞/记录延迟/真实静止尚不能区分。
过冲边界（21:28该轮版本）：现场用户报告飞过头，但所选首段高空窗口日志距A进-0最近仍25.39cm、未报到达；不能把该差值写成实测过冲，也不能认定位置已跟上现场。当时BVA仍按计时巡航、未在巡航内执行35/18cm减速分档，正常XY无变化率限制。随后已按顶部最新状态部署35cm退出巡航修正；平滑未实施，先不提高kp、不同时压缩起飞等待，实飞效果待测。

前轮实飞反馈（2026-10-21 21:15，北京时间）：用户按原自动起飞流程执行，报告“悬停后开始90cm偏/方向反了”，随后确认已落地；助手未发START/起飞指令。日志事实：21:15:11发起飞到142cm，pos_hold介入，选取飞行窗口遥测最高113cm，21:15:43因h=2判爬升未到位而中止，未交棒航线；`/tmp/wpn.log`不存在、BVA CSV为空与此一致。本轮不能作为新BVA直线效果验收。原A面六格入口保留；取消独立90cm测试入口，未创建/部署该程序。
本轮证据：`flight_data/20261021_heading_trial/`内有rec日志、mission.log、route_mission.yaml、fcstat_pwm.csv、pos_hold.log、yaw_damp.csv及实机pos_hold/dead_reckon源；`takeoff_analysis.json`含范围/计算/文件哈希。归零前dr航向约-121.2°，任务归零后变为0°；21:21被动TF抽样原map→base约-147.63°、map→dr_base约-26.36°，差约-121.27°（非共同时间位姿测量），见frame_yaw_check.json。

最新有界问题（代码已修、实飞未验）：实机dead_reckon passthrough=true，修改前文件SHA256 `030512ec4e0f76c76edff7d85ed73523e657cf40f9ade2b31966f4213e9f13fd`；原publish_passthrough第366–369行只平移XY，却扣除off_yaw，导致非零初始航向时位置轴与航向不一致。这是代码事实；它可将本工况纠偏转约121°并使误差方向投影反号，是本轮飞反的强关联解释，不是唯一物理根因证明。诊断时heading_fuse未见运行进程。
用户“修正”授权后，21:25备份并部署`stage1_20261021/flight_effect_candidate/dead_reckon_heading.py`：仅去掉`yaw -= self.off_yaw`，归零保留地图航向，与未旋转的XY一致；新SHA256 `de8e52801bd328c37114317b9000ddefa9d28b40fed7f14937767eb7e2750c73`。板上备份`/home/orangepi/diag/dead_reckon.py.bak-origin-heading-20261021T132507Z-4870a7`，见origin_deployment_result.json；部署前新鲜遥测h=1cm、unlock=0、电机输出0。带--no-block的命令被sudo免密规则拒绝、未执行；改用原标准命令重启成功，见origin_restart_result.json。未改权限、Cartographer安装、PID、UART或任务路线，未发布任务/起飞/复位消息。
21:27:48被动验证通过：flight-link active/running、主PID12392、dead_reckon PID12496、BVA PID12540均在原服务控制组，两个源文件哈希符合预期，启动journal无错误；197组同TF header stamp的map→odom→base_link与map→dr_base航向差最大0°，当前约0.145°。证据origin_activation_result.json，复核脚本verify_origin_runtime.py；此检查不是SLAM源时效或实飞验证。非零角度/再次归零等既有离线检查不重跑：origin_heading_check_result.json、origin_heading_only.patch、prepare_origin_heading_fix.py（实际方法、门控关闭，0/±90/180/约-121°，重复源/缺TF/恢复、跳变拒绝均通过）。下一步沿用原自动起飞任务，由用户中途手动降落后分析起飞保持；不增加“回复启动任务”口令，不自动发起飞，不调PID/换排/扫码。

## 最新局部修改（2026-10-21，BVA方向转换）
- 阶段：用户确认落地锁定后，2026-10-21 21:01（北京时间）备份并替换板上原路径BVA；用户随后明确授权重启，21:04服务重新启动，21:04:59核对active/running、主PID5237、唯一BVA PID5381（旧1904已替换）、新文件哈希匹配、BVA初始化正常，本次启动journal未见错误。未编译ROS或发送起飞/任务/物理测试指令，单板A/B两面12货位仍未验收。
- 基线：`stage1_20261021/snapshot/body_vel_adapter.py`，SHA256 `c49cf9c89909dcbd31af127f2c4666f079beeae7174ef913e1c8835bfc1bcf8d`；20:59板上`/home/orangepi/diag/body_vel_adapter.py`磁盘哈希仍一致，PID1904命令参数指向该路径（不证明内存加载内容）；证据`stage1_20261021/flight_effect_candidate/live_bva_check.json`。`board_scripts/body_vel_adapter.py`哈希不同，未改用它作基线。
- 已实现：`stage1_20261021/flight_effect_candidate/body_vel_adapter.py`保留nb_dir地图方向锁定，删除_cor_psi冻结及覆盖，改用本拍pose航向；航向缺失/NaN/Inf时XY置零。仅on_cmd逻辑变化；巡航计时、减速阈值、PID、Z补偿、偏航参数未改。
- 已检查：`check_heading.py`执行原回调，替换ROS外壳；语法及5组离线检查通过，覆盖转头、无效航向与恢复、低高度门、原PID/混合收口、目标切换。结果与双文件哈希：`stage1_20261021/flight_effect_candidate/heading_check_result.json`；差异：同目录`heading_only.patch`。未重复旧UART/控制/mission检查。推断旧航向冻结可能导致转头后偏离，但实际直线、停点与航向改善未测；原融合航向接收年龄<0.5s/TF回退规则保留，TF源时效未新增验证。
- 部署证据与下一动作：`stage1_20261021/flight_effect_candidate/deployment_result.json`记新哈希`21378549431cd68cda0434feb22f9bb9326d40223f381a6ef1149a7c1de572c0`及旧文件备份`/home/orangepi/diag/body_vel_adapter.py.bak-heading-20261021T130144Z-30bff2`。`restart_result.json`记录重启命令等待45s超时；未重复重启，`restart_status.json`确认新服务已active且无待处理job；最终`activation_result.json`记录PASS、新进程/哈希/初始化日志及空startup_errors。BVA仍在flight-link.service控制组，ROS domain6、BVA_FF=1；Cartographer等原链路随服务正常重启，未改其安装/源码。下一步遵循顶部最新原任务/用户手动降落安排，暂不加入减速/平滑或隔离控制架构。

# 当前交接（2026-10-21，省 token 接续）

## 阶段与边界
- 阶段①「版本与链路」：候选软件已修改并部分隔离验证，整体未验收；阶段②～④未开始。
- 目标仍是单货架 A/B 两面12货位，之后再评估原题24码。
- 用户已授权继续隔离联调；为节省 token 停止本对话执行，转到新对话接续。
- 未替换运行服务、烧录、复位、解锁、操作真实电机/激光或飞行。诊断/联调不是这些操作的授权。
- 用户要求单代理、每轮一个有界问题、只读相关片段、日志仅错误/结果、通过的检查不重复。
- 67个已发现 skill 已在用户 config.toml 禁用；省 token 的 AGENTS.md 保留。不要重启子任务或启用 skill。

## 连接与身份
- 工作区：`C:\Users\金洲毅\Documents\ChatGPT\香橙派内容分析`。
- SSH：`orangepi@192.168.137.186`，密钥 `%USERPROFILE%\.ssh\id_codex_orange`，BatchMode=yes、ConnectTimeout=10。
- 最后只读核对：2026-10-21 15:54，北京时间；SSH可达，flight-link.service active，无 stage1 候选测试进程。
- boot ID：`341689bb-d85c-44fc-918e-422b221e792d`。发生过非助手触发重启，接续仅重核受影响身份。
- 真实运行：ROS Humble/domain6；UART `/dev/ttyS3`、921600；入口 `~/diag/flight_link_run.sh`。
- 原运行UART SHA256：`cb630a0ba0c8c8cb0c5a034ff24fac52ad09013cabfbf427dfc34431eb81f82b`。
- Cartographer实际源码：`~/cartographer_ws/src/cartographer_ros/cartographer_ros/src/node.cpp` 与 include/cartographer_ros/node.h，不是node.cc。
- 对应构建与安装二进制的.text/.rodata/.dynsym/.rela.dyn/buildID一致，整体哈希不同；证据 `stage1_20261021/freshness/binary_sections.json`。不要推断差异原因已证实。
- F407工程：`桌面/学习资料/国一32飞控代码/ProjectSTM32F407`；本地HEX/AXF哈希已记，芯片内实际固件和凌霄闭源核心版本未核实。

## 已实现、实测及证据（全部为候选）
- 原UART缓存问题复现：速度断源、TF持续，过期后400ms仍41帧旧0x32。`stage1_20261021/baseline_result.json`。
- 初始接收年龄补丁15/15 PTY通过；已由带源身份候选进一步覆盖，不必重复旧测试。
- `freshness/`：新 VelocityObservation 含SLAM源戳、epoch、sequence、valid、frame；Cartographer每个新SLAM pose仅发一次，滤波保留身份，UART默认要求元数据。源/TF年龄≤200ms，输出≤50Hz，拒绝非有限/超界，源epoch变更锁存。
- Cartographer、消息包、UART已在板上独立树编译。录制传感器回放8/8：42观测、41有效，源戳匹配42个SLAM cloud；停scan后obs保持42，旧速度/TF仍310→368。`freshness/cartographer_replay_result.json`。
- 带元数据UART真实二进制+PTY 23/23，包含旋转、断源、旧/未来/重复身份、epoch锁存，未出0x31/0x66。`freshness/metadata_uart_result.json`。
- 观测策略/实际滤波方法18/18：`freshness/observation_unit_result.txt`；滤波最终修正scale=min(1,max/magnitude)。
- `control_candidate/`：arbiter唯一最终出口，hold/nav请求带session/epoch/seq/源戳；显式prepare/select/ACK，故障锁存；BVA用当前有效TF航向、禁止无航向透传；PID只发带身份nav_input，旧目标接口默认禁用。
- 控制逻辑31/31：`control_candidate/test_result.txt`。PID隔离编译成功，真实ROS合成13/13：`pid_build.log`、`pid_synthetic_result.json`；重复TF不刷新，旧代次拒绝，新鲜输入不清故障。
- `mission_candidate/`：QR帧身份/时间语义、货位绑定、去重打点、本地≤0.5s截止；默认laser_calibration_verified=false、laser_envelope_verified=false，不可伪设true。
- 最新mission安全与控制联接共23项均已有通过证据（2026-10-21）：首次22通过、1测试夹具缺phase字段报错，见 `mission_candidate/offline_test_result.txt`；补齐真实初始phase=STOPPED后仅重跑失败项1/1通过，见 `mission_candidate/control_recheck_result.txt`。候选运行逻辑未改，未重复通过项。
- 新联接已写入 `mission_control.py`、`mission_runtime.py`、生成的`mission_node.py`、`test_mission_control.py`：显式task_id/session和目标；锁定prepare→人工解锁→hold/nav确认→目标推进→disarm；旧/gs/start禁用，mission_execution_enabled默认false。
- 两个子任务已中断，不要依赖它们继续补文件。mission README已更新新接口、默认禁用、验证范围；按prepare_candidate→build_integrated重生成3个候选均逐字节一致，哈希见 `mission_candidate/generation_check_result.txt`。

## 本次接续项已完成
- 最新任务联接离线验证、实际失败修复、生成一致性核对及README更新均完成；无服务部署或硬件操作。
- 隔离聚合 launch 边界核对、domain80 启动、READY 与 stale 锁存已完成；当前没有运行中的隔离进程。后续按阶段①剩余工作另选一个有界项。
- 已完成该边界核对：`control_candidate/offline_control_observation.launch.py` 只启动 observation_filter、observation_fresh_gate、control_arbiter、pos_hold、body_vel_adapter、PID；强制 `ROS_DOMAIN_ID=80` 与 `ROS_LOCALHOST_ONLY=1`，最终出口 remap 为 `/offline/target_velocity`，明确不含 mission、UART、laser 或合成命令。因此它不能证明最新 mission 的 prepare/select/status/目标交棒。

## 后续剩余（本次接续项之外）
- domain80 mission+control 合成联调已实测 prepare→READY→start→select hold/nav；stale 在 nav 交棒时触发 abort 并锁存 FAULT。最终速度 remap `/offline/target_velocity`，无 UART/GPIO/真实飞控连接；这不是物理处置证明。
- 本机无 ROS；验证在已授权板上独立目录 `~/stage1_20261021/mission_control_candidate` 执行，未覆盖真实运行目录。
- 已获授权并完成隔离同步与两次启动尝试：候选位于板上 `~/stage1_20261021/mission_control_candidate`，未覆盖真实目录。第一次因遗漏 `observation_contract.py` 失败；补齐后第二次在 source 两个 overlay、domain80、localhost-only 下 15 秒启动成功，PID、mission、arbiter、hold、BVA、filter/gate 均进入启动阶段，未注入任务/解锁/起飞/观测/TF。超时 SIGINT 收尾时出现多个 rclpy context invalid/二次 shutdown 错误，属于强制停止路径，尚未证明运行期交棒或断源逻辑；需后续用受控退出与离线事件注入继续验证。
- 受控退出复核（2026-10-21）：同一隔离组合改用 `timeout 10s` 默认 TERM 启动；7 个进程均正常拉起，PID 参数与 hold/BVA 初始化日志出现，未见前次 SIGINT 的 context 错误输出。仍未注入任务/ACK/观测/TF，因此仅确认启动链路，不代表交棒或断源通过。
- 默认安全门注入（2026-10-21）：隔离组合运行期间发布 `/mission/task` `{"op":"start"}`；首次因远端 shell 引号损坏而收到空 JSON并拒绝，第二次修正后日志明确为 `任务请求拒绝: mission_execution_enabled is false`。未产生 prepare/select/nav 控制命令；验证了默认禁用门，尚未验证有效 task/session 的 ACK 交棒。
- 无效 prepare 注入（2026-10-21）：发布合法 JSON 但缺少 `issued_stamp_ns`、FC 锁定证据及 goals；mission 日志明确为 `任务请求拒绝: task intent expired or future`，未产生控制命令。下一步需构造带当前时间戳的 prepare，验证随后因 FC/目标条件拒绝，仍不触发交棒。
- 当前时间戳 prepare 注入（2026-10-21）：临时 ROS 发布器等待组合启动后，用 ROS 时钟生成新鲜 `issued_stamp_ns` 发布 `t7/s7` prepare（goals为空）。mission 明确拒绝：`prepare requires stopped mission and fresh locked FC`；未产生控制命令。已完成任务网关在新鲜意图下的 FC 锁定前置门验证。
- observation stale 注入（2026-10-21）：隔离组合运行期间发布 `/observation/fresh=false`，未见速度输出、FAULT 或异常日志；由于没有先建立有效 mission session，不能据此证明运行中断源锁存，仅确认空闲图对 stale 输入不自行启动控制。
- ROS 图核对（2026-10-21）：隔离运行时 `mission_node` 正常存在；`/mission/task` 为 `std_msgs/String`、1 个订阅（mission）；`/fc_status` 为 `UInt8MultiArray`、mission/control/hold 共3个订阅；`/control/status` 为 `String`、arbiter发布且mission/控制器订阅。注入失败来自临时 publisher 未形成可见端点，非 mission 缺失或 topic 类型不匹配。
- READY 握手验证（2026-10-21）：持久在线 publisher 确认 `/mission/task` publisher/subscriber=1/1；先发布 FC `[0,0]`、新鲜 `prepare(t11,s11)` 与 matching IDLE ACK，再读取 `/mission/status`，得到 `phase=READY, task_id=t11, session=s11, epoch=0, fault=null`。未解锁、未起飞、未发布 start，控制交棒尚未继续。
- 节省步骤规则：不重复已通过的 mission 23项离线测试、控制/PID模拟测试、生成一致性和静态语法检查；后续只验证新的运行时边界：消息连通、READY、断源锁存、受控退出。每项只保留一次结果和日志。
- launch 新增 `mission_execution_enabled` 参数，默认false；仅隔离测试明确传true。早期数次注入失败由端点未匹配、锁定/观测心跳过期及 nav ACK 延时造成；最终事件驱动注入已通过，具体证据如下。
- 事件驱动注入最终通过（2026-10-21，domain80/localhost）：持续发布 locked FC 与 fresh observation，完成 `prepare(evt1)`，发 synthetic unlock/start；接收 `select hold(epoch=1)` 后回 ACTIVE ACK，mission随即发 `select nav(epoch=2)`；检测到该交棒后立即注入 `/observation/fresh=false`。mission 发出 `abort`，并持续保持 `phase=FAULT, fault='velocity observation stale'`。未连接UART/GPIO/飞控，最终速度仍 remap `/offline/target_velocity`。这是合成消息的软件联接验证，不代表物理处置。
- 收尾复核（2026-10-21）：单独板上恢复尝试的 prepare 进入 READY，但 synthetic start 因 `identity/telemetry/ACK not ready` 被拒；运行时恢复断言未单独复测。节省步骤：既有离线用例 `test_observation_lost_latches_no_fresh_auto_resume_new_locked_task_only` 已通过（23项首轮中该项ok），明确先FAULT、fresh恢复仍FAULT、仅新锁定任务prepare获ACK后清除本地锁存；因此无需重跑。隔离 launch 用 TERM 退出，未见额外节点错误日志。
- 当前进程核查（2026-10-21）：无隔离 launch、候选节点或注入器残留进程；本次只读核查未检查或改变 domain6 状态。
- TF/源时间回放对照（2026-10-21）：扩充后的 domain78 localhost 录制回放共42条 observation/42条 SLAM cloud，scan 停止后 observation 仍42、无新增样本；与此同时 TF header stamp 前进1.0s（`map→odom`、`odom→base_link`）。证据 `stage1_20261021/freshness/cartographer_replay_tf_time_result.json`。确认 extrapolated TF stamp 不能代表新观测源时间；PID 按 TF stamp 生成 nav_input，而 gate 仅传 freshness Bool，尚未把 observation source identity 与 TF/pose identity 绑定。下一 bounded task：评估用新的带身份 pose/TF observation 消费路径，不能靠改时间戳伪造共同时间。
- 坐标/身份路径审计（2026-10-21）：PID 实际 pose frame 为 `dr_base`（`pid_integration.yaml`），现有 Cartographer 输出为 map→odom→base_link；dead_reckon 对变换做原点偏移/门控后发布 map→dr_base，沿用输入 TF stamp，但不携带 SLAM observation epoch/sequence。故单独增加 PID pose topic 会绕过 dr_base 语义；完整改动边界是 Cartographer 同一 SLAM pose 产生带身份 pose 与 velocity、dead_reckon 保留该身份/时间并执行现有偏置门控、PID/BVA 校验同一 pose identity。改动前需先确定消息 schema 和失效/重置行为，再做隔离联调。
- 共享位姿消息 `freshness/src/flight_link_msgs/msg/PoseObservation.msg` 已在板上隔离安装 `~/stage1_20261021/freshness/install_pose_candidate` 编译并成功 Python 导入（2026-10-21）；独立于现有 `install/` 与运行服务。
- Cartographer 候选源已新增 `/pose_map_observation` 发布：与 velocity observation 共用同一 local-SLAM source stamp/epoch/sequence；pose 用源样本的未外推 tracking pose，frame 为 map→published_frame。源文件已同步至 freshness 隔离源码树，但遵循用户指出的历史限制“Cartographer 不能重编译”，本次候选构建已停止且未完成；不可宣称通过编译或回放。部分候选 build/log 文件留在 `freshness/build_pose_cartographer`、`install_pose_cartographer`、`log_pose_cartographer`，不属于运行安装；不要重跑 Cartographer 编译。dead_reckon/PID/BVA 尚未接线。
- dead_reckon 候选 `freshness/dead_reckon_candidate.py` 已新增 opt-in `PoseObservation` 输入/输出，保留 source stamp/epoch/sequence，通过原点偏移与跳变/融合门控后输出 `map→dr_base`；epoch 变化锁存，`/dead_reckon/reset` 清锁存并递增 pose_epoch。默认仍走原 TF passthrough 路径。Python AST 语法检查通过，文件已同步至板上 `~/stage1_20261021/freshness/dead_reckon_candidate.py`；未运行、未接入 launch、未构建或验证 ROS 行为。
- 现有 `control_candidate` PID/BVA 已增加可选 PoseObservation 通路：PID校验 map→dr_base、源时间、epoch/sequence，nav_input带回身份；BVA只在身份与缓存位姿完全相同时生成请求，仲裁请求继续携带身份。旧TF模式默认保留。Python AST检查通过；PID单包已在板上新目录 `freshness/install_pid_pose_candidate` 编译成功（2026-10-21），未运行节点。BVA与请求封装已同步至板上 `mission_control_candidate/control_candidate/`。尚未接入 launch；原因是 Cartographer 对应候选未构建且用户历史约束禁止重编 Cartographer。
- 板上隔离树 `~/stage1_20261021/freshness/`：`install/`有消息包/Cartographer/UART，`install_pid/`有PID；先source `/opt/ros/humble/setup.bash` 再对应overlay。Python候选按需同步，不可覆盖真实运行目录。
- 已用domain77 PTY、78 Cartographer回放、79 PID；不要在真实domain6发布合成数据。停发/锁存不证明飞控物理处置。
- 完整SE(2)/TF共同时间、原点复位/融合源年龄未完成；dead_reckon未盲改，TF戳不等于新雷达观测。
- FC/高度/gyro部分旧接口无源戳，仅验证回调接收年龄；下位机内部源年龄、失定位降落/看门狗、物理停止未证实。
- 相机曝光/缓冲延迟、激光外参/光斑包络、硬件故障熄灭、实际固件身份和无桨/物理验收仍缺证据；当前12/12或24/24未验收。

## 按需资料索引
- 总方案：`优化方案-香橙派D题-2026-10-21.md`，只在需要时读相关章节。
- 原始运行快照/身份：`stage1_20261021/snapshot/`、`identity.json`、`live_graph.txt`；初项报告 `stage1_20261021/执行报告.md`。
- `board_diag/`、`board_pullback/`等含旧版本，不可直接覆盖板上。不要重读整个历史对话/全部旧交接。
