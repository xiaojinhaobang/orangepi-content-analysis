# 给 407 固件加一路遥测：把 IMU 的 0x02 帧（罗盘/气压/温度）原样转发给香橙派。
#
# 为什么：磁罗盘被干扰会让飞控航向基准跑偏 → 偏航环持续输出差速 → 自转/漂移
#   （2026-10-16 实测：铁柱旁悬停 err 25~56°、自转撞墙；同一飞机在空地 err 1~8°）。
#   飞控自己会检测磁干扰并点灯（飞控手册:623 罗盘指示：绿=无干扰 / 红=有干扰），
#   但香橙派看不到数值。转发 0x02 后，香橙派可以实时监测 |B| 和罗盘状态，
#   在干扰大的位置直接拒绝起飞。
#
# 0x02 载荷 14 字节：MAG_X/Y/Z(s16×3) + ALT_BAR(s32) + TMP(s16) + BAR_STA(u8) + MAG_STA(u8)
#   来源：匿名通信协议.txt 2) ID：0x02
#
# 用法: pwsh -NoProfile -File .\tools\fc407_add_mag_fwd.ps1
$ErrorActionPreference = 'Stop'

$src   = 'C:\Users\金洲毅\Desktop\学习资料\国一32飞控代码\FcSrc'
$gbk   = [System.Text.Encoding]::GetEncoding(936)
$stamp = 'bak-20261016-mag'
$p     = Join-Path $src 'ANO_DT_LX.c'

$raw = [System.IO.File]::ReadAllText($p, $gbk)
if ($raw -match '0X02.*转发|ForwardFrame\(0x02') {
    "ANO_DT_LX.c 已加过磁力计转发，跳过"
    exit 0
}

$eol = if ($raw.Contains("`r`n")) { "`r`n" } else { "`n" }
Copy-Item -LiteralPath $p -Destination "$p.$stamp"
$L = [System.IO.File]::ReadAllLines($p, $gbk)

# 锚点：0x0B 转发那一段（在它后面插入 0x02）
$i = -1
for ($k = 0; $k -lt $L.Count; $k++) {
    if ($L[$k] -match 'ForwardFrame\(0x0B') { $i = $k; break }
}
if ($i -lt 0) { throw '找不到 0x0B 转发锚点' }
$close = $i + 1                       # 0x0B 分支的收尾大括号（紧跟 ForwardFrame 那行）
if ($L[$close] -notmatch '^\s*\}\s*$') { throw "锚点下一行不是大括号: $($L[$close])" }

$ins = @(
    '	// ★2026-10-16：罗盘/气压/温度（14B）原样转发给香橙派 —— 磁干扰监测',
    '	else if (*(data + 2) == 0X02)',
    '	{',
    '		AnoPTv7ForwardFrame(0x02, (u8 *)(data + 4), (u16)(len - 6));',
    '	}'
)
$L = $L[0..$close] + $ins + $L[($close + 1)..($L.Count - 1)]
[System.IO.File]::WriteAllText($p, (($L -join $eol) + $eol), $gbk)

"ANO_DT_LX.c 已加磁力计(0x02)转发（备份 $stamp）"
