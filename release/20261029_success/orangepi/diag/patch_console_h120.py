#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把一键按钮的高度基准从 150 改成 120（实测飞控高度单位≈真实 cm，150 会飞到 ~145cm）。

改动：起飞接口 takeoff150 → takeoff120；扫描基准 150 → 120（真实约 115cm）。
幂等。用法（在板上）: python3 ~/diag/patch_console_h120.py
"""
import io
import os
import shutil

PAGE = os.path.expanduser('~/diag/console_page.html')

REPL = [
    ("'/api/takeoff150'", "'/api/takeoff120'"),
    ("el.textContent = '起飞到 ~100cm（0x66[1,150]）…';",
     "el.textContent = '起飞到 ~120cm（0x66[1,120]）…';"),
    ("  const base = 150, d = Math.round(base * 0.15);",
     "  const base = 120, d = Math.round(base * 0.15);"),
    ("el.textContent = '高度扫描完成 → 自动降落…';",
     "el.textContent = '高度扫描完成 → 自动降落…';"),
]


def main():
    if not os.path.exists(PAGE):
        raise SystemExit('找不到 %s' % PAGE)
    s = io.open(PAGE, encoding='utf-8', errors='ignore').read()
    if 'takeoff120' in s and 'const base = 120' in s:
        print('已经打过补丁，跳过')
        return
    n = 0
    for old, new in REPL:
        if old in s and old != new:
            s = s.replace(old, new)
            n += 1
    if n == 0:
        raise SystemExit('一处都没匹配上，页面结构变了？')
    bak = PAGE + '.bak-h120'
    if not os.path.exists(bak):
        shutil.copyfile(PAGE, bak)
        print('备份 -> %s' % bak)
    io.open(PAGE, 'w', encoding='utf-8').write(s)
    print('完成：改了 %d 处（起飞+扫描基准 = 120，真实约 115cm）。刷新页面生效' % n)


if __name__ == '__main__':
    main()
