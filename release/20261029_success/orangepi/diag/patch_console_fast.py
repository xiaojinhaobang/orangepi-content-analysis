#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把「起飞100cm + 自动高度扫描」调快：档位 5→4、每档 10→6 秒、稳定等待 8→4 秒。

6 秒一档在 200Hz 采样下是 1200 点，去掉过渡段仍够做频谱。幂等。
用法（在板上）: python3 ~/diag/patch_console_fast.py
"""
import io
import os
import shutil

PAGE = os.path.expanduser('~/diag/console_page.html')

REPL = [
    ("  const plan = [base, base + d, base - d, base + Math.round(d * 0.7), base];",
     "  const plan = [base, base + d, base - d, base];"),
    ("el.textContent = '高度扫描：' + plan.join(' → ') + '（每档 10s）';",
     "el.textContent = '高度扫描：' + plan.join(' → ') + '（每档 6s）';"),
    ("      await new Promise(s => setTimeout(s, 10000));",
     "      await new Promise(s => setTimeout(s, 6000));"),
    ("el.textContent = '已离地，等 8 秒稳一下…';",
     "el.textContent = '已离地，等 4 秒稳一下…';"),
    ("  await new Promise(s => setTimeout(s, 8000));",
     "  await new Promise(s => setTimeout(s, 4000));"),
]


def main():
    if not os.path.exists(PAGE):
        raise SystemExit('找不到 %s' % PAGE)
    s = io.open(PAGE, encoding='utf-8', errors='ignore').read()
    if '每档 6s' in s:
        print('已经打过补丁，跳过')
        return
    n = 0
    for old, new in REPL:
        if old in s:
            s = s.replace(old, new)
            n += 1
    if n == 0:
        raise SystemExit('一处都没匹配上，页面结构变了？')
    bak = PAGE + '.bak-fast'
    if not os.path.exists(bak):
        shutil.copyfile(PAGE, bak)
        print('备份 -> %s' % bak)
    io.open(PAGE, 'w', encoding='utf-8').write(s)
    print('完成：改了 %d 处（总时长约 45 秒）。刷新页面 F5 生效' % n)


if __name__ == '__main__':
    main()
