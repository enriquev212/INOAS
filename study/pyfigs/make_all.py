"""Regenera todas las figuras del articulo.

    python export_data.py     # solo si han cambiado los .mat de study/out
    python make_all.py

Cada figNN_*.py es independiente: lee los CSV de data/ y escribe su PDF y su PNG
en study/figures_py. Este script solo los llama en orden y avisa si alguno falla,
para que una figura rota no pase desapercibida entre la salida de las demas.
"""
import glob
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

scripts = sorted(glob.glob(os.path.join(HERE, 'fig*.py')))
if not scripts:
    sys.exit('no hay scripts fig*.py en %s' % HERE)

fallos = []
for s in scripts:
    name = os.path.basename(s)
    print('== %s' % name)
    r = subprocess.run([sys.executable, name], cwd=HERE,
                       capture_output=True, text=True)
    for ln in (r.stdout or '').splitlines():
        print('   ' + ln)
    if r.returncode != 0:
        fallos.append(name)
        print('   FALLO:')
        for ln in (r.stderr or '').strip().splitlines()[-12:]:
            print('   | ' + ln)

print()
if fallos:
    print('%d de %d figuras han fallado: %s' % (len(fallos), len(scripts),
                                                ', '.join(fallos)))
    sys.exit(1)
print('%d figuras regeneradas en %s'
      % (len(scripts), os.path.join(os.path.dirname(HERE), 'figures_py')))
