#!/bin/bash
echo "=== installed packages ==="
dpkg -l 2>/dev/null | grep -i nomachine
echo "=== server version ==="
/usr/NX/bin/nxserver --version 2>/dev/null | head -3 || echo "(nxserver not found)"
echo "=== services ==="
for s in nxserver nxnode nxclient todeskd; do printf "%s: " "$s"; systemctl is-active $s 2>/dev/null; done
for s in nxserver todeskd; do printf "%s enabled: " "$s"; systemctl is-enabled $s 2>/dev/null; done
echo "=== listening 4000 (NX default port) ==="
ss -tlnp 2>/dev/null | grep -E ":4000|:4011|:4080"
echo "=== sessions dir ==="
du -sh /home/orangepi/.nx 2>/dev/null
ls /home/orangepi/.nx 2>/dev/null | head -8
echo "session count: $(ls /home/orangepi/.nx 2>/dev/null | wc -l)"
echo "=== media/nomachine ==="
ls -la /media/nomachine 2>/dev/null | head -5
echo "=== server config (non-comment) ==="
grep -vE "^#|^$" /usr/NX/etc/server.cfg 2>/dev/null | head -20
echo "=== node config (non-comment) ==="
grep -vE "^#|^$" /usr/NX/etc/node.cfg 2>/dev/null | head -10
echo "=== users allowed ==="
grep -vE "^#|^$" /usr/NX/etc/users.db 2>/dev/null | head -5
echo "=== deb files in home ==="
ls -la /home/orangepi/nomachine*.deb* 2>/dev/null
md5sum /home/orangepi/nomachine*.deb* 2>/dev/null
