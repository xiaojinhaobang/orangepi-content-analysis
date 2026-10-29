import os, re
roots = [os.path.expanduser(p) for p in [
    '~/ros2_ws/src','~/kian_ws/src','~/diansai_ws/src','~/2026diansai_ws/src','~/Desktop','~/cartographer_ws/src']]
SKIP = {'build','install','log','.git','__pycache__','node_modules','third_party','deps','.cache'}
EXT = {'.c','.cpp','.cc','.h','.hpp','.py','.lua'}
bad, seen = [], 0
for root in roots:
    for dp, dn, fn in os.walk(root):
        dn[:] = [d for d in dn if d not in SKIP]
        for f in fn:
            if os.path.splitext(f)[1].lower() not in EXT: continue
            p = os.path.join(dp, f); seen += 1
            try:
                b = open(p,'rb').read()
            except Exception: continue
            reasons = []
            if len(b) == 0: reasons.append('EMPTY')
            if b and not b.endswith(b'\n'): reasons.append('no-trailing-newline')
            try: t = b.decode('utf-8')
            except UnicodeDecodeError:
                try: t = b.decode('gbk')
                except Exception: t = None
            if t:
                if t.count('{') != t.count('}'): reasons.append('brace %d/%d'%(t.count('{'),t.count('}')))
                if t.count('(') != t.count(')'): reasons.append('paren %d/%d'%(t.count('('),t.count(')')))
                if f.endswith('.'+'h') or f.endswith('.hpp'):
                    if '#ifndef' in t and '#endif' not in t: reasons.append('no-#endif')
                if f.endswith('.py'):
                    try: compile(t, p, 'exec')
                    except SyntaxError as e: reasons.append('py-syntax:%s'%e.msg)
            if reasons:
                bad.append((p, ', '.join(reasons)))
print('scanned %d source files, suspicious %d'%(seen, len(bad)))
for p, r in bad[:40]:
    print('%-72s %s' % (p.replace('/home/orangepi/',''), r))
