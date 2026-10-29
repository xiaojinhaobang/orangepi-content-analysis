# 给 407 固件加两路遥测：
#   ① 取消注释 AnoPTv7SendVEL(fc_vel...)  → 把"飞控自己的速度估计"发到香橙派（帧 0x07）
#   ② 新增 AnoPTv7SendPWM()               → 8 路电机输出（帧 0x09，0x09 原协议未占用）
#
# 特性：GBK 读写（源文件是 GBK，避免中文注释被写坏）、带备份、幂等（重复跑不会重复插入）
# 用法: pwsh -NoProfile -File .\tools\fc407_add_pwm.ps1
$ErrorActionPreference = 'Stop'

$src  = 'C:\Users\金洲毅\Desktop\学习资料\国一32飞控代码\FcSrc'
$cPath = Join-Path $src 'AnoDTRasp.c'
$hPath = Join-Path $src 'AnoDTRasp.h'
$gbk  = [System.Text.Encoding]::GetEncoding(936)
$stamp = 'bak-20261009-pwm'

function Read-Gbk($p) { [System.IO.File]::ReadAllText($p, $gbk) }
function Write-Gbk($p, $t) { [System.IO.File]::WriteAllText($p, $t, $gbk) }
function Backup($p) {
    $b = "$p.$stamp"
    if (-not (Test-Path -LiteralPath $b)) { Copy-Item -LiteralPath $p -Destination $b; "备份 -> $b" }
    else { "备份已存在，跳过: $b" }
}

# ---------- 1) AnoDTRasp.c ----------
Backup $cPath
$t = Read-Gbk $cPath
$nl = if ($t.Contains("`r`n")) { "`r`n" } else { "`n" }
$lines = New-Object System.Collections.ArrayList
foreach ($l in ($t -split [regex]::Escape($nl))) { [void]$lines.Add($l) }

# ① 取消注释 AnoPTv7SendVEL（行首是 //）
$vi = -1
for ($i = 0; $i -lt $lines.Count; $i++) {
    if ($lines[$i] -match '^\s*//\s*AnoPTv7SendVEL\(') { $vi = $i; break }
}
if ($vi -lt 0) {
    # 已经改过了？
    $done = $false
    for ($i = 0; $i -lt $lines.Count; $i++) {
        if ($lines[$i] -match '^\s*AnoPTv7SendVEL\(') { $done = $true; break }
    }
    if ($done) { "① AnoPTv7SendVEL 已是启用状态，跳过" }
    else { throw '① 找不到 AnoPTv7SendVEL 的注释行，未做任何修改' }
} else {
    $tab = "`t`t"
    $lines[$vi] = $tab + 'AnoPTv7SendVEL(fc_vel.st_data.vel_x,fc_vel.st_data.vel_y,fc_vel.st_data.vel_z);'
    $lines.Insert($vi + 1, $tab + 'AnoPTv7SendPWM();          // 8 路电机输出(0.1%)，帧 0x09')
    "① 已取消注释 AnoPTv7SendVEL，并在其后插入 AnoPTv7SendPWM()"
}

# ② 插入 AnoPTv7SendPWM 函数实现（放在 AnoPTv7SendVEL 函数之后）
$hasFunc = $false
for ($i = 0; $i -lt $lines.Count; $i++) {
    if ($lines[$i] -match '^void AnoPTv7SendPWM\(') { $hasFunc = $true; break }
}
if ($hasFunc) {
    "② AnoPTv7SendPWM 实现已存在，跳过"
} else {
    $fi = -1
    for ($i = 0; $i -lt $lines.Count; $i++) {
        if ($lines[$i] -match '^void AnoPTv7SendVEL\(') { $fi = $i; break }
    }
    if ($fi -lt 0) { throw '② 找不到 AnoPTv7SendVEL 函数定义' }
    $ci = -1
    for ($i = $fi + 1; $i -lt $lines.Count; $i++) {
        if ($lines[$i].TrimEnd() -eq '}') { $ci = $i; break }
    }
    if ($ci -lt 0) { throw '② 找不到 AnoPTv7SendVEL 函数结尾' }

    $fn = New-Object System.Collections.ArrayList
    [void]$fn.Add('')
    [void]$fn.Add('// 2026-10-09 新增：8 路电机输出（单位 0.1%），帧 ID 0x09。')
    [void]$fn.Add('// 这 8 路是控制链最末端：IMU 板算完的混控输出，407 拿去驱动电调。')
    [void]$fn.Add('// 用途：判定航向慢摆是硬件还是软件 —— 输出在 8~12 秒周期上摆 = 飞控自己在驱动；')
    [void]$fn.Add('//       输出平 = 外部扰动，飞控只是被动在纠。')
    [void]$fn.Add('void AnoPTv7SendPWM(void)')
    [void]$fn.Add('{')
    [void]$fn.Add('    myTAnoPTv7Frame txFrame;')
    [void]$fn.Add('    txFrame.frame.head = 0xAA;')
    [void]$fn.Add('    txFrame.frame.addr = 0xFF;')
    [void]$fn.Add('    txFrame.frame.ID = 0x09;')
    [void]$fn.Add('    txFrame.frame.dataLen = 16;')
    for ($k = 1; $k -le 8; $k++) {
        $o = ($k - 1) * 2
        [void]$fn.Add(("    txFrame.frame.dataBuf[$o] = BYTE0(pwm_to_esc.pwm_m$k);"))
        [void]$fn.Add(("    txFrame.frame.dataBuf[$($o+1)] = BYTE1(pwm_to_esc.pwm_m$k);"))
    }
    [void]$fn.Add('')
    [void]$fn.Add('    u8 sc1 = 0;')
    [void]$fn.Add('    u8 sc2 = 0;')
    [void]$fn.Add('    for(u16 i=0; i<(txFrame.frame.dataLen+4); i++)')
    [void]$fn.Add('    {')
    [void]$fn.Add('        sc1 += txFrame.rawBytes[i];')
    [void]$fn.Add('        sc2 += sc1;')
    [void]$fn.Add('    }')
    [void]$fn.Add('    txFrame.rawBytes[txFrame.frame.dataLen+4] = sc1;')
    [void]$fn.Add('    txFrame.rawBytes[txFrame.frame.dataLen+5] = sc2;')
    [void]$fn.Add('    anoPTv7AddTxFrame(LinkType_ToRasp, &txFrame);')
    [void]$fn.Add('}')
    $lines.InsertRange($ci + 1, $fn)
    "② 已在 AnoPTv7SendVEL 之后插入 AnoPTv7SendPWM() 实现"
}

Write-Gbk $cPath (($lines -join $nl))
"   -> 已写回 $cPath"

# ---------- 2) AnoDTRasp.h ----------
Backup $hPath
$t2 = Read-Gbk $hPath
if ($t2 -match 'void AnoPTv7SendPWM\(void\);') {
    "③ 头文件声明已存在，跳过"
} else {
    $nl2 = if ($t2.Contains("`r`n")) { "`r`n" } else { "`n" }
    $old = 'void AnoPTv7SendVEL(s16 vel_x,s16 vel_y,s16 vel_z);'
    if ($t2.IndexOf($old) -lt 0) { throw '③ 头文件里找不到 AnoPTv7SendVEL 声明' }
    $t2 = $t2.Replace($old, $old + $nl2 + 'void AnoPTv7SendPWM(void);')
    Write-Gbk $hPath $t2
    "③ 头文件已加声明"
}

"完成。"
