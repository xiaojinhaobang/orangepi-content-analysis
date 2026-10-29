import csv, hashlib, json, os, pathlib, shutil, subprocess, time
r = pathlib.Path('/home/orangepi/diag/candidates/first_leg_20261023')
def sha(p): return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()
expected=json.loads((r/'baseline_sha.json').read_text())
targets={
 'uart_to_stm32.cpp':pathlib.Path('/home/orangepi/ros2_ws/src/uart_to_stm32/src/uart_to_stm32.cpp'),
 'uart_to_stm32.hpp':pathlib.Path('/home/orangepi/ros2_ws/src/uart_to_stm32/include/uart_to_stm32/uart_to_stm32.hpp'),
 'mission_node.py':pathlib.Path('/home/orangepi/diag/mission_node.py')}
for name,path in targets.items(): assert sha(path)==expected[name], str(path)+' changed'
fcpath=pathlib.Path('/tmp/fcstat_pwm.csv')
assert time.time()-fcpath.stat().st_mtime<2, 'stale status'
fc=list(csv.DictReader(fcpath.open()))[-1]
assert int(fc['unlock_sta'])==0 and all(float(fc[k])==0 for k in ('pwm0','pwm1','pwm2','pwm3')),fc
assert subprocess.run(['pgrep','-f','[m]ission_node.py|[w]aypoint_nav.py|[p]os_hold.py'],capture_output=True).returncode==1,'task process still exists'
backup=pathlib.Path('/home/orangepi/diag/backups/first_leg_'+time.strftime('%Y%m%d_%H%M%S'))
backup.mkdir()
binary=pathlib.Path('/home/orangepi/ros2_ws/build/uart_to_stm32/uart_to_stm32_node')
shutil.copy2(binary,backup/binary.name)
for name,path in targets.items(): shutil.copy2(path,backup/name)
for name,path in targets.items():
 src=r/'mission_node.py' if name=='mission_node.py' else r/'src/uart_to_stm32'/('src/uart_to_stm32.cpp' if name.endswith('.cpp') else 'include/uart_to_stm32/uart_to_stm32.hpp')
 shutil.copy2(src,path)
shutil.copy2(r/'build/uart_to_stm32/uart_to_stm32_node',binary.with_suffix('.new'))
os.replace(binary.with_suffix('.new'),binary)
result={'backup':str(backup),'ground':fc,'source_sha':{n:sha(p) for n,p in targets.items()},'binary_sha':sha(binary),'scope':'UART async A2 and mission readiness only; no flight command'}
(r/'deployment.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result))
subprocess.run(['sudo','systemctl','restart','flight-link.service'],check=True)
