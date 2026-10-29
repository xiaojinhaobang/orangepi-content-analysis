# 把 AnoPTv7SendPWM() 改成发 0x20/4路，并恢复它的调用 —— 走"已验证通道"验证烧录
# 用法: pwsh -NoProfile -File .\tools\fc407_pwm_to_0x20.ps1
$ErrorActionPreference = 'Stop'
$p = 'C:\Users\金洲毅\Desktop\学习资料\国一32飞控代码\FcSrc\AnoDTRasp.c'
$gbk = [System.Text.Encoding]::GetEncoding(936)
$b = "$p.bak-20261010-pwm20"
if (-not (Test-Path -LiteralPath $b)) { Copy-Item -LiteralPath $p -Destination $b; "备份 -> $b" }

$t = [System.IO.File]::ReadAllText($p, $gbk)
$nl = if ($t.Contains("`r`n")) { "`r`n" } else { "`n" }
$arr = New-Object System.Collections.ArrayList (,[string[]]($t -split [regex]::Escape($nl)))

# 1) 恢复调用
$done1 = $false
for ($i = 0; $i -lt $arr.Count; $i++) {
    if ($arr[$i] -match '^\s*//\s*AnoPTv7SendPWM\(\);') {
        $arr[$i] = "`t`t" + 'AnoPTv7SendPWM();   // 走 5ms 已验证通道发 0x20（4 路）'
        $done1 = $true; "1) 已恢复 AnoPTv7SendPWM() 调用"; break
    }
}
if (-not $done1) { "1) 调用已是启用状态，跳过" }

# 2) 在 SendPWM 函数内改 ID 与长度
$fi = -1
for ($i = 0; $i -lt $arr.Count; $i++) { if ($arr[$i] -match '^void AnoPTv7SendPWM\(') { $fi = $i; break } }
if ($fi -lt 0) { throw '找不到 AnoPTv7SendPWM' }
$changed = 0
for ($i = $fi; $i -lt $fi + 40 -and $i -lt $arr.Count; $i++) {
    if ($arr[$i] -match 'frame\.ID = 0x09;') { $arr[$i] = $arr[$i] -replace '0x09', '0x20'; $changed++ }
    elseif ($arr[$i] -match 'frame\.dataLen = 16;') { $arr[$i] = $arr[$i] -replace '= 16;', '= 8;'; $changed++ }
}
if ($changed -eq 0) { "2) ID/长度已是目标值，跳过" } else { "2) 已改 $changed 处（ID->0x20，dataLen->8）" }

[System.IO.File]::WriteAllText($p, ($arr -join $nl), $gbk)
"已写回 $p"

# 3) 回读验证
$L = ([System.IO.File]::ReadAllText($p, $gbk)) -split "`r?`n"
$fi = ($L | Select-String -Pattern '^void AnoPTv7SendPWM\(' | Select-Object -First 1).LineNumber
"--- SendPWM 头部 ---"
for ($k = $fi - 1; $k -lt $fi + 8; $k++) { "{0,4}|{1}" -f ($k+1), $L[$k] }
"--- 5ms 块调用行 ---"
($L | Select-String -Pattern 'AnoPTv7SendVEL\(fc_vel|AnoPTv7SendPWM\(\)' | ForEach-Object { "{0,4}|{1}" -f $_.LineNumber, $_.Line.Trim() })
