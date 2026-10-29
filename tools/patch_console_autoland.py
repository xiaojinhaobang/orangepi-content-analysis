#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""让「★ 起飞100cm + 自动高度扫描」按钮在扫描完成后**自动降落**。

只把扫描结束那句提示换成"发降落指令（/api/land = 0x66[2]）"。
幂等，可重复跑。用法（在板上）: python3 ~/diag/patch_console_autoland.py
"""
import io
import os
import shutil

PAGE = os.path.expanduser('~/diag/console_page.html')

OLD = """    el.textContent = '高度扫描完成（已回 150）→ 可以点「降落」';"""
NEW = """    el.textContent = '高度扫描完成 → 自动降落…';
    await fetch('/api/land', {method:'POST'});
    el.textContent = '已发降落指令，等它落地';"""


def main():
    if not os.path.exists(PAGE):
        raise SystemExit('找不到 %s' % PAGE)
    s = io.open(PAGE, encoding='utf-8', errors='ignore').read()
    if '已发降落指令' in s:
        print('已经打过补丁，跳过')
        return
    if OLD not in s:
        raise SystemExit('没找到结束提示那句，页面结构变了？')
    s = s.replace(OLD, NEW)
    bak = PAGE + '.bak-autoland'
    if not os.path.exists(bak):
        shutil.copyfile(PAGE, bak)
        print('备份 -> %s' % bak)
    io.open(PAGE, 'w', encoding='utf-8').write(s)
    print('完成：扫描结束后自动降落（刷新页面 F5 生效）')


if __name__ == '__main__':
    main()
