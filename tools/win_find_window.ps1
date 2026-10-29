# 找出指定进程的所有顶层窗口，并把跑到屏幕外的窗口搬回可见区域。
# 用法:
#   powershell -File tools\win_find_window.ps1 -ProcName 匿名上位机V7            # 只列出
#   powershell -File tools\win_find_window.ps1 -ProcName 匿名上位机V7 -Fix        # 搬回来
param(
    [Parameter(Mandatory=$true)][string]$ProcName,
    [switch]$Fix,
    [int]$X = 40,
    [int]$Y = 40,
    [int]$W = 1280,
    [int]$H = 860
)

Add-Type @"
using System;
using System.Text;
using System.Collections.Generic;
using System.Runtime.InteropServices;
public class WinUtil {
    public delegate bool EnumProc(IntPtr h, IntPtr l);
    [DllImport("user32.dll")] public static extern bool EnumWindows(EnumProc cb, IntPtr l);
    [DllImport("user32.dll")] public static extern uint GetWindowThreadProcessId(IntPtr h, out uint pid);
    [DllImport("user32.dll")] public static extern bool IsWindowVisible(IntPtr h);
    [DllImport("user32.dll")] public static extern bool IsIconic(IntPtr h);
    [DllImport("user32.dll", CharSet=CharSet.Unicode)] public static extern int GetWindowText(IntPtr h, StringBuilder s, int n);
    [DllImport("user32.dll", CharSet=CharSet.Unicode)] public static extern int GetClassName(IntPtr h, StringBuilder s, int n);
    [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out RECT r);
    [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h, int cmd);
    [DllImport("user32.dll")] public static extern bool MoveWindow(IntPtr h, int x, int y, int w, int hh, bool repaint);
    [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
    [StructLayout(LayoutKind.Sequential)] public struct RECT { public int Left, Top, Right, Bottom; }
    public static List<IntPtr> ForPid(uint want) {
        var outp = new List<IntPtr>();
        EnumWindows(delegate(IntPtr h, IntPtr l) {
            uint pid; GetWindowThreadProcessId(h, out pid);
            if (pid == want) outp.Add(h);
            return true;
        }, IntPtr.Zero);
        return outp;
    }
    public static string Text(IntPtr h) {
        var sb = new StringBuilder(512); GetWindowText(h, sb, sb.Capacity); return sb.ToString();
    }
    public static string Cls(IntPtr h) {
        var sb = new StringBuilder(256); GetClassName(h, sb, sb.Capacity); return sb.ToString();
    }
}
"@

$procs = @(Get-Process -Name $ProcName -ErrorAction SilentlyContinue)
if (-not $procs) { Write-Error "没找到进程 $ProcName"; exit 1 }

foreach ($p in $procs) {
    Write-Output ("进程 {0}  PID={1}  路径={2}" -f $p.ProcessName, $p.Id, $p.Path)
    $hwnds = [WinUtil]::ForPid([uint32]$p.Id)
    if ($hwnds.Count -eq 0) { Write-Output "  （没有顶层窗口）"; continue }
    foreach ($h in $hwnds) {
        $r = New-Object WinUtil+RECT
        [void][WinUtil]::GetWindowRect($h, [ref]$r)
        $vis  = [WinUtil]::IsWindowVisible($h)
        $icon = [WinUtil]::IsIconic($h)
        $txt  = [WinUtil]::Text($h)
        $cls  = [WinUtil]::Cls($h)
        Write-Output ("  hwnd=0x{0:X}  可见={1}  最小化={2}  区域=({3},{4})-({5},{6})  类={7}  标题='{8}'" -f `
            [int64]$h, $vis, $icon, $r.Left, $r.Top, $r.Right, $r.Bottom, $cls, $txt)
        if ($Fix) {
            [void][WinUtil]::ShowWindow($h, 9)            # SW_RESTORE
            [void][WinUtil]::MoveWindow($h, $X, $Y, $W, $H, $true)
            [void][WinUtil]::SetForegroundWindow($h)
            Write-Output ("    -> 已搬回 ({0},{1}) {2}x{3}" -f $X, $Y, $W, $H)
        }
    }
}
