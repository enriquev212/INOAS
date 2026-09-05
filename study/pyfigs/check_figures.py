"""Comprueba que los PDF cumplen lo que pide una editorial antes de enviarlos.

    python check_figures.py

Cuatro cosas, que son las que suelen rebotar un envio:

  1. Tamano exacto. El PDF tiene que medir el ancho de columna (3.45 in) o el de
     pagina (7.16 in). Si no lo mide, en LaTeX se acabara escalando, y al escalar
     una figura se escala su tipografia: los 8 pt dejan de ser 8 pt y la figura
     desentona con el texto.
  2. Fuentes incrustadas. Sin incrustar, el visor sustituye la fuente y la figura
     se imprime con otra letra.
  3. Nada de Type 3. Es lo que genera matplotlib por defecto y lo que rechazan
     IEEE PDF eXpress y la mayoria de comprobadores: no se puede buscar ni
     extraer texto de una figura Type 3.
  4. Sin imagenes incrustadas. Una figura de lineas guardada con un mapa de bits
     dentro se ve pixelada al ampliar y pesa de mas.
"""
import glob
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FIGDIR = os.path.join(os.path.dirname(HERE), 'figures_py')

SINGLE, DOUBLE = 3.45, 7.16
TOL = 0.02

pdfs = sorted(glob.glob(os.path.join(FIGDIR, '*.pdf')))
if not pdfs:
    sys.exit('no hay PDF en %s' % FIGDIR)

problemas = 0
print('%-32s %14s %7s %9s %s' % ('figura', 'tamano [in]', 'fuentes', 'tipo', 'notas'))
print('-' * 88)
for p in pdfs:
    d = open(p, 'rb').read()
    name = os.path.basename(p)[:-4]
    notas = []

    m = re.search(rb'/MediaBox\s*\[([^\]]*)\]', d)
    w, h = 0.0, 0.0
    if m:
        v = [float(x) / 72 for x in m.group(1).split()]
        w, h = v[2] - v[0], v[3] - v[1]
    if abs(w - SINGLE) < TOL:
        col = 'columna'
    elif abs(w - DOUBLE) < TOL:
        col = 'pagina'
    else:
        col = 'RARO'
        notas.append('ancho %.2f in: no es 3.45 ni 7.16' % w)
        problemas += 1

    nfonts = len(set(re.findall(rb'/BaseFont\s*/([A-Za-z0-9+\-#]+)', d)))
    embedded = d.count(b'/FontFile2') + d.count(b'/FontFile3') + d.count(b'/FontFile ')
    if nfonts and not embedded:
        notas.append('fuentes SIN incrustar')
        problemas += 1

    if re.search(rb'/Subtype\s*/Type3', d):
        notas.append('contiene Type 3')
        problemas += 1

    if re.search(rb'/Subtype\s*/Image', d):
        notas.append('contiene un mapa de bits')
        problemas += 1

    if h > 4.2:
        notas.append('%.2f in de alto: mucho para una columna' % h)

    print('%-32s %6.2f x %-5.2f %7d %9s %s'
          % (name, w, h, nfonts, col, '; '.join(notas)))

print()
if problemas:
    print('%d problemas' % problemas)
    sys.exit(1)
print('%d figuras, todas listas para enviar' % len(pdfs))
