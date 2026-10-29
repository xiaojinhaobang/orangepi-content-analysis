#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把控制台那个「★ 高度扫描」按钮升级成"一键"：
    起飞到 ~100cm（0x66[1,150]，真实约 100cm）
      → 自动等离地（轮询 /api/status）
      → 等 8 秒稳
      → 自动高度扫描（150 → +15% → -15% → +10% → 回 150，每档 10 秒）

只调用已有接口：/api/takeoff150、/api/status、/api/set_height。
**绝不发降落/返航/锁定**。幂等，可重复跑。

用法（在板上）: python3 ~/diag/patch_console_takeoffsweep.py
"""
import io
import os
import shutil

PAGE = os.path.expanduser('~/diag/console_page.html')

OLD_BTN = ('<button class="warn" onclick="heightSweep()">'
           '★ 高度扫描（振动诊断）</button>')
NEW_BTN = ('<button class="warn" onclick="takeoffSweep()">'
           '★ 起飞100cm + 自动高度扫描</button>')

JS = '''
async function takeoffSweep(){
  const el = document.getElementById('msg');
  el.textContent = '起飞到 ~100cm（0x66[1,150]）…';
  try { await fetch('/api/takeoff150', {method:'POST'}); }
  catch(e){ el.textContent = '起飞指令发送失败: ' + e; return; }

  // 等离地（最多 25 秒）
  let ok = false;
  for (let i = 0; i < 50; i++){
    await new Promise(s => setTimeout(s, 500));
    try{
      const j = await (await fetch('/api/status')).json();
      const u = (j.fc || [])[1];
      if (u === 1 && j.height > 40){ ok = true; break; }
    }catch(e){}
  }
  if (!ok){ el.textContent = '没等到离地（先确认已用遥控解锁）'; return; }

  el.textContent = '已离地，等 8 秒稳一下…';
  await new Promise(s => setTimeout(s, 8000));

  const base = 150, d = Math.round(base * 0.15);
  const plan = [base, base + d, base - d, base + Math.round(d * 0.7), base];
  el.textContent = '高度扫描：' + plan.join(' → ') + '（每档 10s）';
  try{
    for (let i = 0; i < plan.length; i++){
      await fetch('/api/set_height', {method:'POST',
        headers:{'Content-Type':'application/json'},
        body: JSON.stringify({h: plan[i]})});
      el.textContent = '高度扫描 ' + (i+1) + '/' + plan.length + '：' +
                       plan[i] + '（真实约 ' + Math.round(plan[i] / 1.5) +
                       'cm，保持 10s）';
      await new Promise(s => setTimeout(s, 10000));
    }
    el.textContent = '高度扫描完成（已回 150）→ 可以点「降落」';
  }catch(e){ el.textContent = '高度扫描失败: ' + e; }
}

'''


def main():
    if not os.path.exists(PAGE):
        raise SystemExit('找不到 %s' % PAGE)
    s = io.open(PAGE, encoding='utf-8', errors='ignore').read()
    if 'takeoffSweep' in s:
        print('已经打过补丁，跳过')
        return

    # ① 按钮：优先直接替换上一次加的那个；没有就插在 setHeight(116) 后面
    if OLD_BTN in s:
        s = s.replace(OLD_BTN, NEW_BTN)
        print('按钮：已把旧按钮升级为「★ 起飞100cm + 自动高度扫描」')
    else:
        a = s.find('onclick="setHeight(116)"')
        if a < 0:
            raise SystemExit('没找到插入锚点，页面结构变了？')
        e = s.find('</button>', a)
        if e < 0:
            raise SystemExit('找不到按钮结束标签')
        e += len('</button>') + 1
        s = s[:e] + '      ' + NEW_BTN + '\n' + s[e:]
        print('按钮：已插入')

    # ② JS
    a2 = s.find('async function loadRoute(){')
    if a2 < 0:
        raise SystemExit('没找到 loadRoute，页面结构变了？')
    s = s[:a2] + JS.lstrip('\n') + s[a2:]

    bak = PAGE + '.bak-takeoffsweep'
    if not os.path.exists(bak):
        shutil.copyfile(PAGE, bak)
        print('备份 -> %s' % bak)
    io.open(PAGE, 'w', encoding='utf-8').write(s)
    print('完成：刷新控制台页面（F5）后，点新按钮即可一键起飞+扫描')


if __name__ == '__main__':
    main()
