#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把交接文档/地面站文档里的旧 IP（手机热点时代）替换成新的专用链路地址。"""
import sys

BACK = chr(96)          # 反引号，避免各种转义问题

REPL = [
    ("| 香橙派 IP | " + BACK + "172.25.105.210" + BACK + "（手机热点 Magic7，IP 可能变） |",
     "| 香橙派 IP | **" + BACK + "192.168.137.186" + BACK + "**（电脑移动热点链路，固定不变；"
     "9/28 的 172.25.105.210 已作废） |"),
    ("orangepi@172.25.105.210" + BACK + "（密码 " + BACK + "orangepi" + BACK + "）",
     "orangepi@192.168.137.186" + BACK + "（密码 " + BACK + "orangepi" + BACK + "）"),
    ("| 机上控制台 | " + BACK + "http://172.25.105.210:8090" + BACK,
     "| 机上控制台 | " + BACK + "http://192.168.137.186:8090" + BACK
     + "（网页地面站 " + BACK + "http://192.168.137.186:8081" + BACK + "）"),
    ("| 当前电脑 IP | " + BACK + "172.25.105.29" + BACK + "（换网络后必须同步改机上桥的 gs_ip，见 §7.2） |",
     "| 当前电脑 IP | **" + BACK + "192.168.137.1" + BACK + "**（Windows 移动热点网关，固定；见 §7.2） |"),
    ("1. 打开 " + BACK + "http://172.25.105.210:8090" + BACK,
     "1. 打开 " + BACK + "http://192.168.137.186:8090" + BACK),
    ("（当前写死 " + BACK + "172.25.105.29" + BACK + "）",
     "（当前写死 " + BACK + "192.168.137.1" + BACK + "）"),
    ("orangepi@172.25.105.210:/home/orangepi/traces", "orangepi@192.168.137.186:/home/orangepi/traces"),
    ('orangepi@172.25.105.210\n', 'orangepi@192.168.137.186\n'),
    ('{"gs_ip":"172.25.105.29"}', '{"gs_ip":"192.168.137.1"}'),
    # 地面站说明文档
    ("机载 IP " + BACK + "172.25.105.210" + BACK, "机载 IP " + BACK + "192.168.137.186" + BACK),
    ("`http://172.25.105.210:8090` 的「跑遍历任务」", "`http://192.168.137.186:8090` 的「跑遍历任务」"),
    ("来源 172.25.105.210", "来源 172.25.105.210（9/28 手机热点时代）"),
]


def main():
    for path in sys.argv[1:]:
        s = open(path, encoding='utf-8').read()
        n = 0
        for a, b in REPL:
            if a in s:
                s = s.replace(a, b)
                n += 1
        open(path, 'w', encoding='utf-8').write(s)
        left = s.count('172.25.105.210') + s.count('172.25.105.29')
        print('%-52s 替换 %2d 处，剩余旧地址 %d 处' % (path, n, left))


if __name__ == '__main__':
    main()
