# 407 第二批改动：把 IMU 的 0x0A/0x0B/0x20/0x21 原样转发给香橙派，并修正 0x20 的越界读取。
#
#   0x20 PWM 控制量   LEN=8~16, U16×N, 0.01% 油门（4 轴只发前 4 路 → 必须按 LEN 读）
#   0x21 姿态控制量   LEN=8,    s16×4: CTRL_ROL/PIT/THR/YAW（±5000；THR 0-10000）
#   0x0A 目标姿态     LEN=6,    s16×3: TAR_ROL/PIT/YAW（×0.01 度）
#   0x0B 目标速度     LEN=6,    s16×3: TAR_SPEED_XYZ（cm/s）
#
# 另外：把上一批的 AnoPTv7SendPWM() 调用注释掉（改用原样转发 0x20，ID 与协议一致）
#
# 用法: pwsh -NoProfile -File .\tools\fc407_forward_frames.ps1
$ErrorActionPreference = 'Stop'

$srcC  = 'C:\Users\金洲毅\Desktop\学习资料\国一32飞控代码\FcSrc\AnoDTRasp.c'
$srcH  = 'C:\Users\金洲毅\Desktop\学习资料\国一32飞控代码\FcSrc\AnoDTRasp.h'
$srcP  = 'C:\Users\金洲毅\Desktop\学习资料\国一32飞控代码\FcSrc\ANO_DT_LX.c'
$gbk   = [System.Text.Encoding]::GetEncoding(936)
$stamp = 'bak-20261010-fwd'

function Read-Gbk($p)  { [System.IO.File]::ReadAllText($p, $gbk) }
function Write-Gbk($p, $t) { [System.IO.File]::WriteAllText($p, $t, $gbk) }
function Backup($p) {
    $b = "$p.$stamp"
    if (-not (Test-Path -LiteralPath $b)) { Copy-Item -LiteralPath $p -Destination $b; "备份 -> $b" }
}
function To-Lines($t) { $nl = if ($t.Contains("`r`n")) { "`r`n" } else { "`n" }
    return @{ nl = $nl; arr = (New-Object System.Collections.ArrayList (,[string[]]($t -split [regex]::Escape($nl)))) } }

# ---------------- 1) AnoDTRasp.c ----------------
Backup $srcC
$t = Read-Gbk $srcC
$o = To-Lines $t; $arr = $o.arr; $nl = $o.nl

# 1a) 注释掉 AnoPTv7SendPWM() 调用
for ($i = 0; $i -lt $arr.Count; $i++) {
    if ($arr[$i] -match '^\s*AnoPTv7SendPWM\(\);\s*//') {
        $arr[$i] = "`t`t" + '// AnoPTv7SendPWM();   // 已改用原样转发 0x20（ID 与协议一致），保留函数备用'
        "1a) 已注释 AnoPTv7SendPWM() 调用"
        break
    }
}

# 1b) 追加 AnoPTv7ForwardFrame 实现
if ($t -match 'AnoPTv7ForwardFrame') {
    "1b) AnoPTv7ForwardFrame 已存在，跳过"
} else {
    $fi = -1
    for ($i = 0; $i -lt $arr.Count; $i++) {
        if ($arr[$i] -match '^void AnoPTv7SendPWM\(') { $fi = $i; break }
    }
    if ($fi -lt 0) { throw '找不到 AnoPTv7SendPWM 定义' }
    $ci = -1
    for ($i = $fi + 1; $i -lt $arr.Count; $i++) {
        if ($arr[$i] -match '^\s*\}\s*$') { $ci = $i; break }   # 注意：行首可能有 tab
    }
    if ($ci -lt 0) { throw '找不到 AnoPTv7SendPWM 结尾' }

    $fn = New-Object System.Collections.ArrayList
    [void]$fn.Add('')
    [void]$fn.Add('// 2026-10-10 新增：通用转发 —— 把 IMU 发来某一帧的"载荷"原样打包发给香橙派。')
    [void]$fn.Add('// 参数：id = 转发的帧 ID（保持与协议一致，便于上位机/日志对照）')
    [void]$fn.Add('//       payload = 载荷首地址，len = 载荷长度（不含帧头/长度/校验）')
    [void]$fn.Add('// 用途：目标姿态(0x0A)、目标速度(0x0B)、PWM控制量(0x20)、姿态控制量(0x21)')
    [void]$fn.Add('void AnoPTv7ForwardFrame(u8 id, const u8 *payload, u16 len)')
    [void]$fn.Add('{')
    [void]$fn.Add('    if (len > 200) return;                       // 防御：dataBuf 256 字节')
    [void]$fn.Add('    myTAnoPTv7Frame txFrame;')
    [void]$fn.Add('    txFrame.frame.head = 0xAA;')
    [void]$fn.Add('    txFrame.frame.addr = 0xFF;')
    [void]$fn.Add('    txFrame.frame.ID = id;')
    [void]$fn.Add('    txFrame.frame.dataLen = (u8)len;')
    [void]$fn.Add('    for (u16 i = 0; i < len; i++) txFrame.frame.dataBuf[i] = payload[i];')
    [void]$fn.Add('')
    [void]$fn.Add('    u8 sc1 = 0, sc2 = 0;')
    [void]$fn.Add('    for (u16 i = 0; i < (txFrame.frame.dataLen + 4); i++) { sc1 += txFrame.rawBytes[i]; sc2 += sc1; }')
    [void]$fn.Add('    txFrame.rawBytes[txFrame.frame.dataLen + 4] = sc1;')
    [void]$fn.Add('    txFrame.rawBytes[txFrame.frame.dataLen + 5] = sc2;')
    [void]$fn.Add('    anoPTv7AddTxFrame(LinkType_ToRasp, &txFrame);')
    [void]$fn.Add('}')
    $arr.InsertRange($ci + 1, $fn)
    "1b) 已追加 AnoPTv7ForwardFrame() 实现"
}
Write-Gbk $srcC ($arr -join $nl)
"    -> 已写回 AnoDTRasp.c"

# ---------------- 2) AnoDTRasp.h ----------------
Backup $srcH
$h = Read-Gbk $srcH
if ($h -match 'void AnoPTv7ForwardFrame\(') {
    "2) 头文件声明已存在，跳过"
} else {
    $old = 'void AnoPTv7SendPWM(void);'
    if ($h.IndexOf($old) -lt 0) { throw '头文件里找不到 AnoPTv7SendPWM 声明' }
    $hnl = if ($h.Contains("`r`n")) { "`r`n" } else { "`n" }
    $h = $h.Replace($old, $old + $hnl + 'void AnoPTv7ForwardFrame(u8 id, const u8 *payload, u16 len);')
    Write-Gbk $srcH $h
    "2) 头文件已加声明"
}

# ---------------- 3) ANO_DT_LX.c ----------------
Backup $srcP
$p = Read-Gbk $srcP
$o2 = To-Lines $p; $arr2 = $o2.arr; $nl2 = $o2.nl

# 3a) 顶部加函数原型（ANO_DT_LX.c 不 include AnoDTRasp.h，避免循环包含）
if ($p -match 'AnoPTv7ForwardFrame') {
    "3a) 原型/调用已存在，跳过"
} else {
    for ($i = 0; $i -lt $arr2.Count; $i++) {
        if ($arr2[$i] -match '^#include "User_Task.h"') {
            $arr2.Insert($i + 1, '')
            $arr2.Insert($i + 2, '// 由 AnoDTRasp.c 提供：把一帧载荷原样转发给香橙派（见 AnoDTRasp.c）')
            $arr2.Insert($i + 3, 'void AnoPTv7ForwardFrame(u8 id, const u8 *payload, u16 len);')
            "3a) 已加函数原型"
            break
        }
    }
}

# 3b) 替换 0x20 分支（按 LEN 读，修越界；并原样转发）
$i0 = -1
for ($i = 0; $i -lt $arr2.Count; $i++) {
    if ($arr2[$i] -match '== 0X20\)' -and $i -gt 10) { $i0 = $i; break }
}
if ($i0 -lt 0) { throw '找不到 0x20 分支' }
$i1 = -1
for ($i = $i0 + 1; $i -lt $arr2.Count; $i++) {
    if ($arr2[$i] -match '^\s*\}\s*$') { $i1 = $i; break }      # 注意：行首可能有 tab
}
if ($i1 -lt 0) { throw '找不到 0x20 分支结尾' }
if (($i1 - $i0) -gt 25) { throw "0x20 分支范围异常大（第 $($i0+1)~$($i1+1) 行），放弃修改" }

$blk = New-Object System.Collections.ArrayList
[void]$blk.Add('	if (*(data + 2) == 0X20)      // PWM控制量：0-10000，0.01%油门；4轴只发前4路')
[void]$blk.Add('	{')
[void]$blk.Add('		u8 n = (u8)(len - 6);')
[void]$blk.Add('')
[void]$blk.Add('		// 先清零，避免上一次的残值被当成有效数据')
[void]$blk.Add('		pwm_to_esc.pwm_m1 = 0; pwm_to_esc.pwm_m2 = 0;')
[void]$blk.Add('		pwm_to_esc.pwm_m3 = 0; pwm_to_esc.pwm_m4 = 0;')
[void]$blk.Add('		pwm_to_esc.pwm_m5 = 0; pwm_to_esc.pwm_m6 = 0;')
[void]$blk.Add('		pwm_to_esc.pwm_m7 = 0; pwm_to_esc.pwm_m8 = 0;')
[void]$blk.Add('')
[void]$blk.Add('		// ★2026-10-10 修正：原来一次读死 8 路(16字节)，四轴只发 8 字节 →')
[void]$blk.Add('		//   m5~m8 读到帧外内存（实测香橙派收到 3016%/6553% 这种垃圾）。改成按 LEN 读。')
[void]$blk.Add('		if (n >= 2)  pwm_to_esc.pwm_m1 = *((u16 *)(data + 4));')
[void]$blk.Add('		if (n >= 4)  pwm_to_esc.pwm_m2 = *((u16 *)(data + 6));')
[void]$blk.Add('		if (n >= 6)  pwm_to_esc.pwm_m3 = *((u16 *)(data + 8));')
[void]$blk.Add('		if (n >= 8)  pwm_to_esc.pwm_m4 = *((u16 *)(data + 10));')
[void]$blk.Add('		if (n >= 10) pwm_to_esc.pwm_m5 = *((u16 *)(data + 12));')
[void]$blk.Add('		if (n >= 12) pwm_to_esc.pwm_m6 = *((u16 *)(data + 14));')
[void]$blk.Add('		if (n >= 14) pwm_to_esc.pwm_m7 = *((u16 *)(data + 16));')
[void]$blk.Add('		if (n >= 16) pwm_to_esc.pwm_m8 = *((u16 *)(data + 18));')
[void]$blk.Add('')
[void]$blk.Add('		AnoPTv7ForwardFrame(0x20, (u8 *)(data + 4), (u16)n);   // 原样转发给香橙派')
[void]$blk.Add('	}')
[void]$blk.Add('	// ★2026-10-10 新增：原样转发下面几帧给香橙派（ID 与协议一致）')
[void]$blk.Add('	else if (*(data + 2) == 0X21)   // 姿态控制量 s16×4: ROL/PIT/THR/YAW(±5000)')
[void]$blk.Add('	{')
[void]$blk.Add('		AnoPTv7ForwardFrame(0x21, (u8 *)(data + 4), (u16)(len - 6));')
[void]$blk.Add('	}')
[void]$blk.Add('	else if (*(data + 2) == 0X0A)   // 目标姿态 s16×3: ROL/PIT/YAW(×0.01度)')
[void]$blk.Add('	{')
[void]$blk.Add('		AnoPTv7ForwardFrame(0x0A, (u8 *)(data + 4), (u16)(len - 6));')
[void]$blk.Add('	}')
[void]$blk.Add('	else if (*(data + 2) == 0X0B)   // 目标速度 s16×3: XYZ(cm/s)')
[void]$blk.Add('	{')
[void]$blk.Add('		AnoPTv7ForwardFrame(0x0B, (u8 *)(data + 4), (u16)(len - 6));')
[void]$blk.Add('	}')

$arr2.RemoveRange($i0, $i1 - $i0 + 1)
$arr2.InsertRange($i0, $blk)
"3b) 已替换 0x20 分支并加入 0x21/0x0A/0x0B 转发（原第 $($i0+1)~$($i1+1) 行）"

Write-Gbk $srcP ($arr2 -join $nl2)
"    -> 已写回 ANO_DT_LX.c"
"完成。"
