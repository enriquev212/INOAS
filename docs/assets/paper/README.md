# Paper Figure Assets

Team-supplied figures from the submitted IEEE Aerospace 2027 paper, added on
10 October 2026. The PDFs are unchanged originals; PNGs are rendered previews
for GitHub, not redrawn or regenerated scientific results.

| Paper figure | Vector source | GitHub preview |
| --- | --- | --- |
| Fig. 1: navigation and guidance architecture | [architecture.pdf](architecture.pdf) | [architecture.png](architecture.png) |
| Fig. 3: supporting-plane avoidance geometry | [avoidance-geometry.pdf](avoidance-geometry.pdf) | [avoidance-geometry.png](avoidance-geometry.png) |

Supplied source names:

- `fig1_arquitectura_letra_grande_v3 (1).pdf`
- `fig3_koz_linealizacion_letra_grande_v2 (1).pdf`

To re-render from the repository root with Poppler:

```shell
pdftoppm -png -singlefile -scale-to-x 2400 -scale-to-y -1 docs/assets/paper/architecture.pdf docs/assets/paper/architecture
pdftoppm -png -singlefile -scale-to-x 1500 -scale-to-y -1 docs/assets/paper/avoidance-geometry.pdf docs/assets/paper/avoidance-geometry
```

The architecture is a functional diagram. For the exact closed-loop observation
emulation, UKF input and distinction between correction enable and receiver
power, see [model architecture](../../model-architecture.md).

These are project-team figure assets, not third-party references redistributed
under the source-code license. Superseded challenge figures are isolated under
`../history/` and [project history](../../history.md).
