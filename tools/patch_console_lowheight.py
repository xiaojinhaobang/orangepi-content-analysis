#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""给控制台加两个低高度快捷键：→100 / →80（用已有的 setHeight()，不用改后端）。

用法（在板上）: python3 ~/diag/patch_console_lowheight.py
幂等，靠 marker 判断。
"""
import io
import os
import shutil

PAGE = os.path.expanduser('~/diag/console_page.html')
MARK = 'setHeight(80)'

ADD = ('      <button onclick="setHeight(100)">→100</button>\n'
       '      <button onclick="setHeight(80)">→80</button>\n')


def main():
    if not os.path.exists(PAGE):
        raise SystemExit('找不到 %s' % PAGE)
    s = io.open(PAGE, encoding='utf-8', errors='ignore').read()
    if MARK in s:
        print('已经打过补丁，跳过')
        return
    anchor = 'onclick="setHeight(116)"'
    a = s.find(anchor)
    if a < 0:
        raise SystemExit('没找到 setHeight(116) 按钮，页面结构变了？')
    e = s.find('</button>', a)
    if e < 0:
        raise SystemExit('找不到按钮结束标签')
    e += len('</button>') + 1
    s = s[:e] + ADD + s[e:]
    bak = PAGE + '.bak-lowheight'
    if not os.path.exists(bak):
        shutil.copyfile(PAGE, bak)
        print('备份 -> %s' % bak)
    io.open(PAGE, 'w', encoding='utf-8').write(s)
    print('完成：已加「→100」「→80」按钮（刷新页面 F5 生效）')


if __name__ == '__main__':
    main()
