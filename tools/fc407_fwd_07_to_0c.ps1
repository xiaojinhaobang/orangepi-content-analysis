# 验证"转发函数"是否有效：把已验证飞控一定会发的 0x07（飞控速度）
# 用 AnoPTv7ForwardFrame 转发到 ID 0x0C。收到 0x0C = 转发有效。
# 用法: pwsh -NoProfile -File .\tools\fc407_fwd_07_to_0c.ps1
$ErrorActionPreference = 'Stop'
$p = 'C:\Users\金洲毅\Desktop\学习资料\国一32飞控代码\FcSrc\ANO_DT_LX.c'
$gbk = [System.Text.Encoding]::GetEncoding(936)
$b = "$p.bak-20261010-fwd07"
if (-not (Test-Path -LiteralPath $b)) { Copy-Item -LiteralPath $p -Destination $b; "备份 -> $b" }
$t = [System.IO.File]::ReadAllText($p, $gbk)
if ($t -match '0x0C') { "已存在 0x0C，跳过"; return }
$nl = if ($t.Contains("`r`n")) { "`r`n" } else { "`n" }
$arr = New-Object System.Collections.ArrayList (,[string[]]($t -split [regex]::Escape($nl)))

$i0 = -1
for ($i = 0; $i -lt $arr.Count; $i++) {
    if ($arr[$i] -match '== 0X07\)') { $i0 = $i; break }
}
if ($i0 -lt 0) { throw '找不到 0X07 分支' }
# 从 i0+1 的 "{" 开始数括号，找外层结束
$depth = 0; $i1 = -1
for ($i = $i0 + 1; $i -lt $arr.Count; $i++) {
    foreach ($ch in $arr[$i].ToCharArray()) {
        if ($ch -eq '{') { $depth++ }
        elseif ($ch -eq '}') { $depth--; if ($depth -eq 0) { $i1 = $i; break } }
    }
    if ($i1 -ge 0) { break }
}
if ($i1 -lt 0) { throw '找不到 0X07 分支结尾' }
if (($i1 - $i0) -gt 20) { throw "0X07 分支范围异常（$($i0+1)~$($i1+1)），放弃" }

$arr.Insert($i1, "`t`tAnoPTv7ForwardFrame(0x0C, (u8 *)(data + 4), (u16)(len - 6));   // 验证转发：0x07 -> 0x0C")
[System.IO.File]::WriteAllText($p, ($arr -join $nl), $gbk)
"已在 0X07 分支（第 $($i0+1)~$($i1+1) 行）里加入转发到 0x0C"
$L = ([System.IO.File]::ReadAllText($p, $gbk)) -split "`r?`n"
for ($k = $i0; $k -lt $i1 + 2; $k++) { "{0,4}|{1}" -f ($k + 1), $L[$k] }
