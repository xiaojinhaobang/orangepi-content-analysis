# 给 407 固件修"起飞命令投递"：把一次性投递改成"以飞控回显为准"的事务。
#
# v1（2026-10-16 白天）用 CK 应答当成功判据 —— 实测错了：CK 应答只是传输层的
#   "帧收到了"，飞控收到后照样可以拒绝执行。那一版烧进去后第一次起飞仍然被拒。
# v2（本脚本）用飞控自己的回显做判据：
#   fc_sta.cmd_fun.CMD_1 == 0x05   （就是香橙派看的 cmd1）
# 这个字段由 ANO_DT_LX.c 从飞控状态帧解出，F407 自己就能读，所以能真正闭环。
#
# 逻辑（1ms 任务里跑）：
#   收到 0x66[1,h] → mission_tk_req = 1
#   槽空闲就装帧发出（OneKey_Takeoff），进入等待
#   TAKEOFF_VERIFY_MS 内看到 fc_sta.cmd_fun.CMD_1==5 → 成功，放行速度通道
#   超时没看到 → 算一次尝试、重新排队；累计 TAKEOFF_RETRY_MAX 次后补发一次
#   解锁命令（= CH6 高位那条路径发的同一条，实测能清掉飞控"拒收起飞"的粘住
#   状态）再试一轮；仍不行才置 mission_tk_fail。
#
# 特性：GBK 读写（源文件是 GBK）、带备份、幂等（重复跑不会重复插入）
# 用法: pwsh -NoProfile -File .\tools\fc407_fix_takeoff.ps1
$ErrorActionPreference = 'Stop'

$src   = 'C:\Users\金洲毅\Desktop\学习资料\国一32飞控代码\FcSrc'
$gbk   = [System.Text.Encoding]::GetEncoding(936)
$stamp = 'bak-20261016-tk'

function Get-Lines($p) { [System.IO.File]::ReadAllLines($p, $gbk) }
function Get-Eol($p) {
    $raw = [System.IO.File]::ReadAllText($p, $gbk)
    if ($raw.Contains("`r`n")) { return "`r`n" } else { return "`n" }
}
function Save-Lines($p, $lines, $eol) {
    [System.IO.File]::WriteAllText($p, (($lines -join $eol) + $eol), $gbk)
}
function Find-Idx($lines, $pattern, $from = 0) {
    for ($i = $from; $i -lt $lines.Count; $i++) {
        if ($lines[$i] -match $pattern) { return $i }
    }
    return -1
}
function Assert($cond, $msg) { if (-not $cond) { throw "断言失败: $msg" } }

$p = Join-Path $src 'AnoDTRasp.c'
$raw = [System.IO.File]::ReadAllText($p, $gbk)
if ($raw -match 'mission_tk_sent') {
    "AnoDTRasp.c 已打过补丁，跳过"
} else {
    $eol = Get-Eol $p
    Copy-Item -LiteralPath $p -Destination "$p.$stamp"
    $L = Get-Lines $p

    # 1) 事务变量
    $i = Find-Idx $L '^u16 mission_tk_h'
    Assert ($i -ge 0) 'AnoDTRasp.c: 找不到 mission_tk_h 声明'
    $ins = @(
        'u8  mission_tk_sent    = 0;   // 2026-10-16: frame queued, waiting for the FC echo',
        'u8  mission_tk_retry   = 0;   // 2026-10-16: attempts of the current takeoff transaction',
        'u8  mission_tk_fail    = 0;   // 2026-10-16: 1 = takeoff never accepted (budget exhausted)',
        'u8  mission_tk_reset   = 0;   // 2026-10-16: 1 = the unlock-reset has been sent once',
        'u16 mission_tk_wait_ms = 0;   // 2026-10-16: ms spent waiting for the FC echo',
        '#define TAKEOFF_VERIFY_MS 300   // 2026-10-16: how long to wait for cmd1==5',
        '#define TAKEOFF_RETRY_MAX 4    // 2026-10-16: attempts before the reset/unlock retry'
    )
    $L = $L[0..$i] + $ins + $L[($i + 1)..($L.Count - 1)]

    # 2) 用"等飞控回显"取代"装进槽就算成功"
    $i = Find-Idx $L '//----- takeoff request: retry until accepted -----'
    Assert ($i -ge 0) 'AnoDTRasp.c: 找不到 takeoff request 注释'
    $j = Find-Idx $L 'if\(OneKey_Takeoff\(h\)\)' $i
    Assert ($j -gt $i) 'AnoDTRasp.c: 找不到 if(OneKey_Takeoff(h))'
    Assert ($L[$j + 2] -match 'mission_tk_req = 0;') 'AnoDTRasp.c: 上下文不符（mission_tk_req = 0）'
    Assert ($L[$j + 5] -match '^\s*\}\s*$') 'AnoDTRasp.c: 上下文不符（if 的结尾大括号）'
    Assert ($L[$j + 6] -match '^\s*\}\s*$') 'AnoDTRasp.c: 上下文不符（外层 if 的结尾大括号）'
    $new = @(
        '    /*=== 2026-10-16 takeoff transaction (v2) ===',
        '     * "Queued" is NOT success any more: the CK ack only means the frame',
        '     * arrived, and the FC may still refuse to act on it. The only real',
        '     * confirmation is the FC own echo of the command: cmd1 == 0x05',
        '     * (fc_sta.cmd_fun.CMD_1, decoded in ANO_DT_LX.c).',
        '     */',
        '    if(mission_tk_req)',
        '    {',
        '        u16 h = mission_tk_h ? mission_tk_h : 150;',
        '        if(!mission_tk_sent)',
        '        {',
        '            if(OneKey_Takeoff(h))          /* slot free -> queued */',
        '            {',
        '                mission_tk_sent    = 1;',
        '                mission_tk_wait_ms = 0;',
        '            }',
        '        }',
        '        else if(fc_sta.cmd_fun.CID == 0x10 && fc_sta.cmd_fun.CMD_1 == 0x05)',
        '        {',
        '            /* the FC really took the takeoff */',
        '            mission_tk_req   = 0;',
        '            mission_tk_sent  = 0;',
        '            mission_tk_retry = 0;',
        '            mission_tk_fail  = 0;',
        '            mission_tk_reset = 0;',
        '            takeoff_if       = 1;    // release the velocity channel',
        '            flown_flag       = 1;    // release horizontal velocity',
        '        }',
        '        else if(++mission_tk_wait_ms >= TAKEOFF_VERIFY_MS)',
        '        {',
        '            /* not accepted: count an attempt and re-queue */',
        '            mission_tk_wait_ms = 0;',
        '            mission_tk_sent    = 0;',
        '            mission_tk_retry++;',
        '            if(mission_tk_retry >= TAKEOFF_RETRY_MAX)',
        '            {',
        '                if(!mission_tk_reset)',
        '                {',
        '                    /* clear the FC stuck state: the same unlock command the',
        '                       CH6-high path sends (bench-verified 2026-10-16) */',
        '                    if(FC_Unlock()) { mission_tk_reset = 1; mission_tk_retry = 0; }',
        '                }',
        '                else',
        '                {',
        '                    mission_tk_req  = 0;',
        '                    mission_tk_fail = 1;   /* refused for good (report it) */',
        '                }',
        '            }',
        '        }',
        '    }'
    )
    $tail = $j + 7
    Assert ($L[$tail].Trim() -eq '') 'AnoDTRasp.c: 装帧块后面应是空行'
    $L = $L[0..($i - 1)] + $new + $L[$tail..($L.Count - 1)]

    # 3) 新的起飞请求：复位事务状态
    $i = Find-Idx $L 'mission_tk_req = 1;'
    Assert ($i -ge 0) 'AnoDTRasp.c: 找不到 mission_tk_req = 1;'
    $new = @(
        '                    mission_tk_req     = 1;',
        '                    mission_tk_sent    = 0;   /* 2026-10-16: fresh transaction */',
        '                    mission_tk_retry   = 0;',
        '                    mission_tk_fail    = 0;',
        '                    mission_tk_reset   = 0;',
        '                    mission_tk_wait_ms = 0;'
    )
    $L = $L[0..($i - 1)] + $new + $L[($i + 1)..($L.Count - 1)]

    Save-Lines $p $L $eol
    "AnoDTRasp.c 已打 v2 补丁（备份 $stamp）"
}

"完成（ANO_DT_LX.c 未改动）。回滚：把 AnoDTRasp.c.$stamp 覆盖回原名即可。"
