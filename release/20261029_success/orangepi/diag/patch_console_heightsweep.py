#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""给飞行控制台页面加「★ 高度扫描（振动诊断）」按钮。幂等，可在板上重复跑。

为什么需要：振动频谱分析要在**不同转速**下各采一个窗口。高度一变，飞控油门
（转速）跟着变，就能把"跟着转速走的转动频率"和"固定不动的结构谐振频率"分开。

按钮只调用**已有的** /api/set_height（0x66[1,h] 改高度），
**绝不发降落/返航/锁定** —— 后端一行都不用改。

用法（在板上）: python3 ~/diag/patch_console_heightsweep.py
"""
import io
import os
import shutil

PAGE = os.path.expanduser('~/diag/console_page.html')

BUTTON = ('<button class="warn" onclick="heightSweep()">'
          '★ 高度扫描（振动诊断）</button>\n')

JS = '''
async function heightSweep(){
  const el = document.getElementById('msg');
  const base = Math.round(Number(document.getElementById('hgt').value) || 120);
  const step = Math.max(8, Math.round(base * 0.15));
  const plan = [base, base + step, base - step, base + Math.round(step * 0.7), base];
  const dwell = 10;                       // 每档 10 秒
  el.textContent = '高度扫描：' + plan.join(' → ') + '（每档 ' + dwell + 's）';
  try{
    for (let i = 0; i < plan.length; i++){
      await fetch('/api/set_height', {method:'POST',
        headers:{'Content-Type':'application/json'},
        body: JSON.stringify({h: plan[i]})});
      el.textContent = '高度扫描 ' + (i+1) + '/' + plan.length + '：' +
                       plan[i] + 'cm（保持 ' + dwell + 's）';
      await new Promise(s => setTimeout(s, dwell * 1000));
    }
    el.textContent = '高度扫描完成（已回 ' + base + 'cm）';
  }catch(e){ el.textContent = '高度扫描失败: ' + e; }
}

'''


def main():
    if not os.path.exists(PAGE):
        raise SystemExit('找不到 %s' % PAGE)
    s = io.open(PAGE, encoding='utf-8', errors='ignore').read()
    if 'heightSweep' in s:
        print('已经打过补丁，跳过')
        return

    # ① 按钮：插在「→116」那个按钮后面
    anchor = s.find('onclick="setHeight(116)"')
    if anchor < 0:
        raise SystemExit('没找到 setHeight(116) 按钮，页面结构变了？')
    end = s.find('</button>', anchor)
    if end < 0:
        raise SystemExit('找不到按钮结束标签')
    end += len('</button>') + 1
    s = s[:end] + '      ' + BUTTON + s[end:]

    # ② JS 函数：插在 loadRoute 之前
    anchor2 = s.find('async function loadRoute(){')
    if anchor2 < 0:
        raise SystemExit('没找到 loadRoute，页面结构变了？')
    s = s[:anchor2] + JS.lstrip('\n') + s[anchor2:]

    bak = PAGE + '.bak-heightsweep'
    if not os.path.exists(bak):
        shutil.copyfile(PAGE, bak)
        print('备份 -> %s' % bak)
    io.open(PAGE, 'w', encoding='utf-8').write(s)
    print('已加入「★ 高度扫描（振动诊断）」按钮（页面刷新后可见，不用重启控制台）')


if __name__ == '__main__':
    main()
