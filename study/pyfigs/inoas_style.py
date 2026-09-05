"""Estilo comun de las figuras del articulo.

Un solo sitio donde se decide como se ve una figura: tipografia, paleta, grosores
y como se guarda. Los scripts de figuras solo llaman a figure(), finish() y
save().

Criterios, todos pensados para 3.5 in de ancho impreso en dos columnas:

  * Times y matematicas STIX, que es lo que compone IEEEtran, para que la figura
    no cante al lado del texto.
  * Marco abierto y separado de los datos: solo eje izquierdo e inferior, y
    desplazados 3 pt hacia fuera. El rectangulo cerrado de matplotlib encierra la
    figura sin aportar informacion.
  * Rejilla horizontal punteada, tenue y por detras.
  * Paleta de Paul Tol: distinguible con daltonismo y separable en gris. Se
    acompana siempre de marcador o trazo distinto, porque una figura impresa en
    blanco y negro tiene que seguir leyendose.
  * PDF vectorial con las fuentes incrustadas como TrueType (Type 42), que es lo
    que exigen las editoriales, y PNG a 600 ppp para revisar.
"""
import os

import matplotlib as mpl

mpl.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter

# Anchos de columna de IEEE
SINGLE = 3.45
DOUBLE = 7.16

HERE = os.path.dirname(os.path.abspath(__file__))
FIGDIR = os.path.join(os.path.dirname(HERE), 'figures_py')
DATADIR = os.path.join(HERE, 'data')
os.makedirs(FIGDIR, exist_ok=True)

# --- paleta ---------------------------------------------------------------
# Paul Tol, conjunto 'bright'. Los nombres son los que se usan en los scripts.
C = {
    'blue':   '#3B6FA8',
    'red':    '#CC3311',
    'green':  '#117733',
    'orange': '#EE7733',
    'purple':  '#882E72',
    'teal':   '#009988',
    'grey':   '#5A5A5A',
    'lgrey':  '#BFBFBF',
    'vlgrey': '#E8E8E8',
    'ink':    '#1A1A1A',
}
CYCLE = [C['blue'], C['green'], C['orange'], C['red'], C['purple'], C['teal']]
MARKERS = ['o', 's', '^', 'D', 'v', 'P']
DASHES = [(None, None), (4, 1.4), (1.2, 1.2), (5, 1.4, 1.2, 1.4), (3, 1.2, 1.2, 1.2)]

_RC = {
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'STIXGeneral', 'DejaVu Serif'],
    'mathtext.fontset': 'stix',
    'font.size': 8,
    'axes.labelsize': 8,
    'axes.titlesize': 8,
    'xtick.labelsize': 7.5,
    'ytick.labelsize': 7.5,
    'legend.fontsize': 7,

    'axes.linewidth': 0.6,
    'axes.edgecolor': C['ink'],
    'axes.labelcolor': C['ink'],
    'axes.labelpad': 2.5,
    'axes.spines.top': False,
    'axes.spines.right': False,
    'axes.axisbelow': True,
    'axes.prop_cycle': mpl.cycler(color=CYCLE),

    'xtick.direction': 'out',
    'ytick.direction': 'out',
    'xtick.major.size': 2.6,
    'ytick.major.size': 2.6,
    'xtick.major.width': 0.6,
    'ytick.major.width': 0.6,
    'xtick.minor.size': 1.4,
    'ytick.minor.size': 1.4,
    'xtick.minor.width': 0.5,
    'ytick.minor.width': 0.5,
    'xtick.major.pad': 2.5,
    'ytick.major.pad': 2.0,
    'xtick.color': C['ink'],
    'ytick.color': C['ink'],

    'grid.color': '#9A9A9A',
    'grid.linestyle': ':',
    'grid.linewidth': 0.5,
    'grid.alpha': 0.55,

    'lines.linewidth': 1.2,
    'lines.markersize': 3.4,
    'lines.markeredgewidth': 0.7,
    'patch.linewidth': 0.6,

    'legend.frameon': False,
    'legend.handlelength': 1.5,
    'legend.handletextpad': 0.5,
    'legend.labelspacing': 0.32,
    'legend.borderpad': 0.2,
    'legend.borderaxespad': 0.3,
    'legend.columnspacing': 1.1,

    'figure.dpi': 200,
    'savefig.dpi': 600,
    'savefig.bbox': 'tight',
    'savefig.pad_inches': 0.012,
    'figure.facecolor': 'white',
    'savefig.facecolor': 'white',

    # Fuentes incrustadas como TrueType, no como subconjunto Type 3
    'pdf.fonttype': 42,
    'ps.fonttype': 42,
    'pdf.compression': 6,
}


def use():
    """Aplica el estilo. Llamar una vez al principio de cada script."""
    mpl.rcParams.update(_RC)


def figure(width=SINGLE, height=2.15, nrows=1, ncols=1, **kw):
    """Una figura del ancho de columna. Devuelve (fig, ax) o (fig, axes).

    width y height son el tamano FINAL del recuadro impreso, etiquetas incluidas.
    save() se encarga de que la caja recortada mida exactamente eso.
    """
    use()
    kw.setdefault('constrained_layout', False)
    fig, ax = plt.subplots(nrows, ncols, figsize=(width, height), **kw)
    fig._inoas_target = (float(width), float(height))
    return fig, ax


def finish(ax, grid='y', offset=3, legend=None, **legend_kw):
    """Acabado del eje: marco abierto y desplazado, rejilla por detras.

    grid : 'y' | 'x' | 'both' | 'none'
    """
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_visible(True)
        ax.spines[side].set_linewidth(0.6)
        ax.spines[side].set_color(C['ink'])
        if offset:
            ax.spines[side].set_position(('outward', offset))

    ax.grid(False)
    if grid in ('y', 'both'):
        ax.grid(True, axis='y', which='major')
    if grid in ('x', 'both'):
        ax.grid(True, axis='x', which='major')
    ax.set_axisbelow(True)

    ax.tick_params(which='both', top=False, right=False)

    if legend:
        lg = ax.legend(**legend_kw)
        lg.set_zorder(5)
        return lg
    return None


def plain_log(ax, axis='x'):
    """Etiquetas '5', '30', '100' en vez de 10^0 en un eje logaritmico."""
    def fmt(v, _):
        if v <= 0:
            return ''
        if v >= 1:
            return '%g' % v
        return ('%g' % v)
    a = ax.xaxis if axis == 'x' else ax.yaxis
    a.set_major_formatter(FuncFormatter(fmt))
    a.set_minor_formatter(FuncFormatter(lambda v, p: ''))


def label_line(ax, x, y, text, color, ha='left', va='bottom', halo=True, **kw):
    """Rotulo pegado a una curva. Con halo blanco cuando cae sobre datos."""
    import matplotlib.patheffects as pe
    t = ax.text(x, y, text, color=color, ha=ha, va=va,
                fontsize=kw.pop('fontsize', 7), zorder=6, **kw)
    if halo:
        t.set_path_effects([pe.withStroke(linewidth=1.8, foreground='white')])
    return t


def panel_tag(ax, tag, x=-0.20, y=1.04):
    """(a), (b), ... en la esquina del panel."""
    ax.text(x, y, tag, transform=ax.transAxes, fontsize=8, fontweight='bold',
            va='bottom', ha='left', color=C['ink'])


def _fit_exact(fig, target, tries=6, tol=0.004):
    """Ajusta el lienzo para que la caja recortada mida exactamente el objetivo.

    matplotlib recorta con bbox_inches='tight' DESPUES de dibujar, asi que el PDF
    sale mas estrecho que el figsize pedido: los margenes por defecto se van con
    el recorte. Si luego en LaTeX se estira ese PDF hasta el ancho de columna, se
    estiran con el las fuentes, y los 8 pt que se han elegido aqui dejan de serlo.

    Como el tamano de la letra esta en puntos y no depende del lienzo, basta con
    agrandar el lienzo hasta que lo recortado mida lo que se quiere. El margen no
    escala con el lienzo, asi que no converge en un paso: se itera.
    """
    tw, th = target
    fig.canvas.draw()
    for _ in range(tries):
        bb = fig.get_tightbbox(fig.canvas.get_renderer())
        if abs(bb.width - tw) < tol and abs(bb.height - th) < tol:
            break
        w, h = fig.get_size_inches()
        fig.set_size_inches(w * tw / bb.width, h * th / bb.height)
        fig.canvas.draw()
    return fig.get_tightbbox(fig.canvas.get_renderer())


def save(fig, name, tight=True):
    """Guarda PDF vectorial y PNG a 600 ppp en study/figures_py.

    El PDF sale con el tamano exacto que pidio figure(), de modo que en LaTeX se
    incluye sin escalar y la tipografia de la figura coincide con la del texto.
    """
    pdf = os.path.join(FIGDIR, name + '.pdf')
    png = os.path.join(FIGDIR, name + '.png')
    target = getattr(fig, '_inoas_target', None)
    if tight and target:
        bb = _fit_exact(fig, target)
        kw = {'bbox_inches': bb, 'pad_inches': 0.0}
        size = (bb.width, bb.height)
    elif tight:
        kw = {'bbox_inches': 'tight', 'pad_inches': 0.012}
        size = None
    else:
        kw = {}
        size = tuple(fig.get_size_inches())
    fig.savefig(pdf, **kw)
    fig.savefig(png, dpi=600, **kw)
    plt.close(fig)
    if size:
        print('  %s.pdf / .png   %.2f x %.2f in' % (name, size[0], size[1]))
    else:
        print('  %s.pdf / .png' % name)
    return pdf


def read_csv(name):
    """Lee un CSV de data/ saltando las lineas de comentario. -> dict de arrays."""
    import numpy as np
    path = os.path.join(DATADIR, name)
    with open(path, encoding='utf-8') as fh:
        lines = [ln for ln in fh if not ln.startswith('#')]
    header = lines[0].strip().split(',')
    cols = {h: [] for h in header}
    for ln in lines[1:]:
        if not ln.strip():
            continue
        for h, v in zip(header, ln.strip().split(',')):
            cols[h].append(float(v) if v not in ('', 'nan') else np.nan)
    return {h: np.asarray(v) for h, v in cols.items()}


def meta():
    import json
    with open(os.path.join(DATADIR, 'meta.json'), encoding='utf-8') as fh:
        return json.load(fh)
