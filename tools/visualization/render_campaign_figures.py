"""Render paper-style INOAS figures from campaign CSV exports.

Expected workflow:

1. In MATLAB or MATLAB Online, run:
   run_baseline_campaign

2. From the repository root, run:
   python tools/visualization/render_campaign_figures.py --case results/campaign/baseline

The script writes PDF and PNG figures under <case>/figures.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


NAVY = "#172a45"
BLUE = "#2474b8"
GREEN = "#188a55"
MAGENTA = "#b516a8"
ORANGE = "#d46b2c"
GREY = "#6d7785"
RED = "#b33234"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--case",
        type=Path,
        default=Path("results/campaign/baseline"),
        help="Campaign case directory containing timeseries/control/navigation CSV files.",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=None,
        help="Output directory for figures. Defaults to <case>/figures.",
    )
    parser.add_argument(
        "--wide",
        action="store_true",
        help="Use two-column figure widths for all plots.",
    )
    return parser.parse_args()


def configure_matplotlib() -> None:
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 8.2,
            "axes.titlesize": 9.0,
            "axes.labelsize": 8.2,
            "legend.fontsize": 7.2,
            "xtick.labelsize": 7.4,
            "ytick.labelsize": 7.4,
            "axes.grid": True,
            "grid.alpha": 0.28,
            "grid.linewidth": 0.55,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
        }
    )


def read_csv(path: Path) -> np.ndarray:
    if not path.exists():
        raise FileNotFoundError(path)
    return np.genfromtxt(path, delimiter=",", names=True, dtype=float, encoding="utf-8")


def col(data: np.ndarray, name: str, default: float = math.nan) -> np.ndarray:
    if data.dtype.names is None or name not in data.dtype.names:
        return np.full(data.shape, default, dtype=float)
    values = np.asarray(data[name], dtype=float)
    if values.ndim == 0:
        values = values.reshape(1)
    return values


def finite_mask(*arrays: np.ndarray) -> np.ndarray:
    mask = np.ones_like(arrays[0], dtype=bool)
    for arr in arrays:
        mask &= np.isfinite(arr)
    return mask


def metric(metrics: np.ndarray, name: str, default: float = math.nan) -> float:
    # genfromtxt cannot keep the metric column as strings with dtype=float.
    # metrics.csv is optional for plotting, so fall back if it was read as NaN.
    return default


def load_metrics_text(path: Path) -> dict[str, float]:
    values: dict[str, float] = {}
    if not path.exists():
        return values
    lines = path.read_text(encoding="utf-8").splitlines()
    if len(lines) <= 1:
        return values
    for line in lines[1:]:
        if not line.strip():
            continue
        parts = line.split(",", 1)
        if len(parts) != 2:
            continue
        try:
            values[parts[0]] = float(parts[1])
        except ValueError:
            pass
    return values


def save_figure(fig: plt.Figure, outdir: Path, name: str) -> list[Path]:
    outdir.mkdir(parents=True, exist_ok=True)
    png = outdir / f"{name}.png"
    pdf = outdir / f"{name}.pdf"
    fig.savefig(png, dpi=500, bbox_inches="tight")
    fig.savefig(pdf, bbox_inches="tight")
    plt.close(fig)
    return [png, pdf]


def plot_navigation(case: Path, outdir: Path, wide: bool) -> list[Path]:
    nav = read_csv(case / "navigation.csv")
    time = col(nav, "time_s")
    lam = col(nav, "lambda")
    nis = col(nav, "nis")
    err = col(nav, "estimation_error_m")

    width = 7.15 if wide else 3.48
    fig, axes = plt.subplots(3, 1, figsize=(width, 4.55), sharex=True)

    if np.any(np.isfinite(lam)):
        axes[0].step(time, np.where(lam > 0.5, 1.0, 0.0), where="post", color=GREEN, lw=1.55)
    axes[0].set_yticks([0, 1], ["Prop.", "GNSS"])
    axes[0].set_ylim(-0.12, 1.12)
    axes[0].set_ylabel("Mode")
    axes[0].set_title("Navigation mode and estimator consistency", color=NAVY, weight="bold")

    axes[1].plot(time, err, color=BLUE, lw=1.55)
    axes[1].set_ylabel("Est. error [m]")

    if np.any(np.isfinite(nis)):
        axes[2].plot(time, nis, color=MAGENTA, lw=1.55, label="NIS")
        axes[2].axhline(12, color=RED, ls="--", lw=1.0, label="Threshold")
        axes[2].legend(loc="upper right", frameon=False)
    else:
        axes[2].text(0.02, 0.55, "NIS signal not logged", transform=axes[2].transAxes, color=GREY)
    axes[2].set_ylabel("NIS [-]")
    axes[2].set_xlabel("Time [s]")

    fig.tight_layout()
    return save_figure(fig, outdir, "fig6_navigation")


def plot_collision(case: Path, outdir: Path, wide: bool) -> list[Path]:
    ts = read_csv(case / "timeseries.csv")
    time = col(ts, "time_s")
    distance = col(ts, "debris_distance_m")
    nominal = col(ts, "nominal_debris_distance_m")
    safe = col(ts, "dynamic_safe_first_m")
    d0 = col(ts, "safe_radius_m")
    margin = col(ts, "robust_margin_m")

    width = 7.15 if wide else 3.48
    fig, axes = plt.subplots(3, 1, figsize=(width, 4.65), sharex=True)
    axes[0].plot(time, distance, color=GREEN, lw=1.8, label="Controlled")
    axes[0].plot(time, nominal, color=GREY, lw=1.25, ls="--", label="Nominal")
    axes[0].plot(time, safe, color=BLUE, lw=1.35, label="$d_{safe}$")
    if np.any(np.isfinite(d0)):
        axes[0].axhline(float(np.nanmedian(d0)), color=ORANGE, ls="--", lw=1.0, label="$d_0$")
    axes[0].set_ylabel("Distance [m]")
    axes[0].set_title("Debris separation and robust margin", color=NAVY, weight="bold")
    axes[0].legend(loc="upper right", frameon=False, ncol=2)

    axes[1].plot(time, safe, color=BLUE, lw=1.6)
    if np.any(np.isfinite(d0)):
        axes[1].axhline(float(np.nanmedian(d0)), color=ORANGE, ls="--", lw=1.0)
    axes[1].set_ylabel("$d_{safe}$ [m]")

    axes[2].plot(time, margin, color=GREEN, lw=1.7)
    axes[2].axhline(0, color=RED, ls="--", lw=1.0)
    axes[2].set_ylabel("Margin [m]")
    axes[2].set_xlabel("Time [s]")

    fig.tight_layout()
    return save_figure(fig, outdir, "fig6_collision_margin")


def plot_control(case: Path, outdir: Path, wide: bool) -> list[Path]:
    control = read_csv(case / "control.csv")
    time = col(control, "time_s")
    u1 = col(control, "u_1_mps2")
    u2 = col(control, "u_2_mps2")
    u3 = col(control, "u_3_mps2")
    umax = col(control, "u_max_mps2")
    sat = col(control, "axis_saturation_ratio")
    dv = col(control, "delta_v_mps")

    width = 7.15 if wide else 3.48
    fig, axes = plt.subplots(3, 1, figsize=(width, 4.65), sharex=True)
    axes[0].plot(time, u1, color=BLUE, lw=1.35, label="$u_1$")
    axes[0].plot(time, u2, color=GREEN, lw=1.35, label="$u_2$")
    axes[0].plot(time, u3, color=ORANGE, lw=1.35, label="$u_3$")
    if np.any(np.isfinite(umax)):
        limit = float(np.nanmedian(umax))
        axes[0].axhline(limit, color=RED, ls="--", lw=0.95)
        axes[0].axhline(-limit, color=RED, ls="--", lw=0.95)
    axes[0].set_ylabel("Accel. [m/s$^2$]")
    axes[0].set_title("Control authority and manoeuvre cost", color=NAVY, weight="bold")
    axes[0].legend(loc="upper right", frameon=False, ncol=3)

    axes[1].plot(time, sat, color=MAGENTA, lw=1.55)
    axes[1].axhline(1, color=RED, ls="--", lw=0.95)
    axes[1].set_ylim(0, max(1.12, np.nanmax(sat) * 1.08 if np.any(np.isfinite(sat)) else 1.12))
    axes[1].set_ylabel("Sat. ratio [-]")

    axes[2].plot(time, dv, color=GREEN, lw=1.8)
    axes[2].set_ylabel("$\\Delta v$ [m/s]")
    axes[2].set_xlabel("Time [s]")

    fig.tight_layout()
    return save_figure(fig, outdir, "fig6_control_delta_v")


def plot_3d_overview(case: Path, outdir: Path) -> list[Path]:
    ts = read_csv(case / "timeseries.csv")
    control = read_csv(case / "control.csv")

    sc_r = col(ts, "sc_debris_R_m")
    sc_i = col(ts, "sc_debris_I_m")
    sc_n = col(ts, "sc_debris_N_m")
    ref_r = col(ts, "ref_debris_R_m")
    ref_i = col(ts, "ref_debris_I_m")
    ref_n = col(ts, "ref_debris_N_m")
    d0 = col(ts, "safe_radius_m")

    ctime = col(control, "time_s")
    u1 = col(control, "u_1_mps2")
    u2 = col(control, "u_2_mps2")
    u3 = col(control, "u_3_mps2")
    umax = col(control, "u_max_mps2")

    mask = finite_mask(sc_r, sc_i, sc_n, ref_r, ref_i, ref_n)
    sc_r, sc_i, sc_n = sc_r[mask], sc_i[mask], sc_n[mask]
    ref_r, ref_i, ref_n = ref_r[mask], ref_i[mask], ref_n[mask]

    d0_value = float(np.nanmedian(d0)) if np.any(np.isfinite(d0)) else 150.0
    current_idx = int(0.55 * max(len(sc_r) - 1, 0))

    fig = plt.figure(figsize=(7.15, 3.65))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.28, 1.0], height_ratios=[1, 1], wspace=0.28, hspace=0.34)

    ax3 = fig.add_subplot(gs[:, 0], projection="3d")
    ax3.plot(ref_i, ref_r, ref_n, color=GREY, ls="--", lw=1.0, label="Nominal")
    ax3.plot(sc_i, sc_r, sc_n, color=BLUE, lw=1.7, label="MPC")
    ax3.scatter([0], [0], [0], marker="x", s=38, color="black", label="Debris")
    if len(sc_r):
        ax3.scatter(sc_i[current_idx], sc_r[current_idx], sc_n[current_idx], s=26, color=ORANGE, edgecolor="white", linewidth=0.5, label="Current")

    phi = np.linspace(0, 2 * np.pi, 28)
    th = np.linspace(0, np.pi, 14)
    xs = d0_value * np.outer(np.cos(phi), np.sin(th))
    ys = d0_value * np.outer(np.sin(phi), np.sin(th))
    zs = d0_value * np.outer(np.ones_like(phi), np.cos(th))
    ax3.plot_wireframe(xs, ys, zs, rstride=4, cstride=4, color=ORANGE, alpha=0.25, linewidth=0.45)
    ax3.set_title("3D LVLH encounter geometry", color=NAVY, fontweight="bold", pad=2)
    ax3.set_xlabel("In-track [m]", labelpad=0)
    ax3.set_ylabel("Radial [m]", labelpad=0)
    ax3.set_zlabel("Cross-track [m]", labelpad=0)
    ax3.tick_params(pad=0)
    ax3.view_init(elev=23, azim=-56)
    ax3.set_box_aspect((1.45, 1.0, 0.55))
    ax3.legend(loc="upper left", frameon=False, bbox_to_anchor=(-0.08, 0.96))

    ax = fig.add_subplot(gs[0, 1])
    ax.plot(ref_i, ref_r, color=GREY, ls="--", lw=1.0, label="Nominal")
    ax.plot(sc_i, sc_r, color=BLUE, lw=1.45, label="MPC")
    ang = np.linspace(0, 2 * np.pi, 200)
    ax.plot(d0_value * np.cos(ang), d0_value * np.sin(ang), color=ORANGE, ls="--", lw=0.9, label="$d_0$")
    ax.scatter([0], [0], marker="x", color="black", s=30, label="Debris")
    if len(sc_r):
        ax.scatter(sc_i[current_idx], sc_r[current_idx], s=22, color=ORANGE, edgecolor="white", linewidth=0.5)
    ax.set_title("Radial/in-track plane", color=NAVY, fontweight="bold")
    ax.set_xlabel("In-track [m]")
    ax.set_ylabel("Radial [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.legend(loc="upper right", frameon=False, ncol=2)

    ax = fig.add_subplot(gs[1, 1])
    ax.plot(ctime, u1, color=BLUE, lw=1.1, label="$u_1$")
    ax.plot(ctime, u2, color=GREEN, lw=1.1, label="$u_2$")
    ax.plot(ctime, u3, color=ORANGE, lw=1.1, label="$u_3$")
    if np.any(np.isfinite(umax)):
        limit = float(np.nanmedian(umax))
        ax.axhline(limit, color=RED, ls="--", lw=0.8)
        ax.axhline(-limit, color=RED, ls="--", lw=0.8)
    ax.set_title("MPC control effort", color=NAVY, fontweight="bold")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Accel. [m/s$^2$]")
    ax.legend(loc="upper right", frameon=False, ncol=3)

    fig.suptitle("Close-encounter avoidance playback", color=NAVY, fontweight="bold", y=1.02, fontsize=10.2)
    return save_figure(fig, outdir, "fig6_3d_overview")


def main() -> None:
    args = parse_args()
    configure_matplotlib()

    case = args.case
    outdir = args.out if args.out is not None else case / "figures"
    outdir.mkdir(parents=True, exist_ok=True)

    assets: list[Path] = []
    assets.extend(plot_navigation(case, outdir, args.wide))
    assets.extend(plot_collision(case, outdir, args.wide))
    assets.extend(plot_control(case, outdir, args.wide))
    assets.extend(plot_3d_overview(case, outdir))

    print("Generated campaign figures:")
    for asset in assets:
        print(f"  {asset}")


if __name__ == "__main__":
    main()
