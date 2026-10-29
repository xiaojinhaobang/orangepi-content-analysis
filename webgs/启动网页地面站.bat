@echo off
chcp 65001 >nul
title D题 网页版地面站
cd /d "%~dp0"
echo ============================================
echo  D题 单板双面盘点 - 网页版地面站
echo  关闭本窗口 = 停止服务
echo  队友用浏览器打开：http://本机IP:8080
echo ============================================
echo.
start "" cmd /c "timeout /t 2 >nul & start http://127.0.0.1:8080"
python server.py --port 8080
echo.
echo 服务已退出。若提示 python 不是命令，请先装 Python 3 并勾选 Add to PATH。
pause
