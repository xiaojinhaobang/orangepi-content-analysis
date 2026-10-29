#!/usr/bin/env bash
# 只测 restart_link.sh 里的 IP 识别函数，不重启任何东西
source <(sed -n '/^detect_gs_ip()/,/^}/p' "$HOME/restart_link.sh")

echo "0) 当前环境(有 SSH):        $(detect_gs_ip)"
echo "1) 命令行参数优先:          $(detect_gs_ip 10.1.2.3)"
echo "2) 环境变量 GS_IP:          $(GS_IP=10.9.9.9 detect_gs_ip)"
echo "3) 去掉 SSH_CLIENT 看回退:  $(env -u SSH_CLIENT bash -c "
        source <(sed -n '/^detect_gs_ip()/,/^}/p' \"\$HOME/restart_link.sh\"); detect_gs_ip")"
