"""Generate INOAS presentation visuals from exported simulation data.

Expected workflow:

1. Run a MATLAB/Simulink simulation.
2. In MATLAB, run: export_visualization_data
3. From the repository root, run:
   python tools/visualization/generate_visualization_assets.py

The script reads results/visualization/inoas_visualization_data.mat by default
and creates PNG/GIF assets under results/visualization/.
"""

from __future__ import annotations

import argparse
import math
import shutil
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from scipy.io import loadmat


NAVY = "#172a45"
BLUE = "#2474b8"
GREEN = "#188a55"
MAGENTA = "#b516a8"
GREY = "#6d7785"
LIGHT_GREY = "#d6dbe2"
ORANGE = "#d46b2c"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mat",
        type=Path,
        default=Path("results/visualization/inoas_visualization_data.mat"),
        help="MAT file exported by export_visualization_data.m",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=Path("results/visualization"),
        help="Output directory for generated assets",
    )
    parser.add_argument(
        "--frames",
        type=int,
        default=120,
        help="Number of frames in the debris-avoidance GIF",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=16,
        help="Frames per second in the debris-avoidance GIF",
    )
    parser.add_argument(
        "--window-half-width",
        type=float,
        default=220.0,
        help="Half-width around the debris encounter used for the GIF [s]",
    )
    parser.add_argument(
        "--gnss-power-w",
        type=float,
        default=8.0,
        help="Nominal GNSS receiver power used for energy plots [W]",
    )
    return parser.parse_args()


def mat_1d(mat: dict, name: str, default: np.ndarray | None = None) -> np.ndarray:
    if name not in mat:
        return np.array([]) if default is None else default
    data = np.asarray(mat[name]).squeeze()
    if data.size == 0:
        return np.array([])
    return data.reshape(-1).astype(float)


def mat_2d(mat: dict, name: str, columns: int | None = None) -> np.ndarray:
    if name not in mat:
        return np.empty((0, 0))
    data = np.asarray(mat[name]).squeeze()
    if data.size == 0:
        return np.empty((0, 0))
    if data.ndim == 1:
        data = data.reshape((-1, 1))
    if columns is not None and data.shape[1] != columns and data.shape[0] == columns:
        data = data.T
    return data.astype(float)


def mat_scalar(mat: dict, name: str, default: float = math.nan) -> float:
    if name not in mat:
        return default
    data = np.asarray(mat[name]).squeeze()
    if data.size == 0:
        return default
    value = float(data.reshape(-1)[0])
    return value if np.isfinite(value) else default


def finite_rows(*arrays: np.ndarray) -> np.ndarray:
    mask = None
    for arr in arrays:
        if arr.ndim == 1:
            current = np.isfinite(arr)
        else:
            current = np.all(np.isfinite(arr), axis=1)
        mask = current if mask is None else (mask & current)
    return mask if mask is not None else np.array([], dtype=bool)


def cumulative_trapezoid(y: np.ndarray, t: np.ndarray) -> np.ndarray:
    if y.size == 0:
        return np.array([])
    out = np.zeros_like(y, dtype=float)
    if y.size > 1:
        out[1:] = np.cumsum(0.5 * (y[1:] + y[:-1]) * np.diff(t))
    return out


def zoh_sample(source_t: np.ndarray, source_y: np.ndarray, target_t: np.ndarray) -> np.ndarray:
    if source_t.size == 0 or source_y.size == 0:
        return np.ones_like(target_t)
    order = np.argsort(source_t)
    source_t = source_t[order]
    source_y = source_y[order]
    idx = np.searchsorted(source_t, target_t, side="right") - 1
    idx = np.clip(idx, 0, source_y.size - 1)
    return source_y[idx]


def set_equal_3d(ax, points: np.ndarray) -> None:
    finite = points[np.all(np.isfinite(points), axis=1)]
    if finite.size == 0:
        return
    mins = finite.min(axis=0)
    maxs = finite.max(axis=0)
    center = 0.5 * (mins + maxs)
    radius = 0.5 * float(np.max(maxs - mins))
    radius = max(radius, 1.0)
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)


def rtn_components(reference_r: np.ndarray, reference_v: np.ndarray, vector_eci: np.ndarray) -> np.ndarray:
    r_norm = np.linalg.norm(reference_r, axis=1)
    radial = reference_r / np.maximum(r_norm[:, None], 1e-12)

    normal = np.cross(reference_r, reference_v)
    normal_norm = np.linalg.norm(normal, axis=1)
    normal = normal / np.maximum(normal_norm[:, None], 1e-12)

    along_track = np.cross(normal, radial)

    return np.column_stack(
        [
            np.sum(vector_eci * radial, axis=1),
            np.sum(vector_eci * along_track, axis=1),
            np.sum(vector_eci * normal, axis=1),
        ]
    )


def save_orbit_context(outdir: Path, data: dict) -> Path:
    time = data["time"]
    truth = data["truth"] / 1000.0
    reference = data["reference"] / 1000.0
    estimate = data["estimate"] / 1000.0
    debris = data["debris"] / 1000.0

    stride = max(1, int(len(time) / 2500))
    path = outdir / "inoas_orbit_context.png"

    fig = plt.figure(figsize=(10.5, 8.0), dpi=160)
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(reference[::stride, 0], reference[::stride, 1], reference[::stride, 2], "--", color=GREY, lw=1.7, label="Reference")
    ax.plot(truth[::stride, 0], truth[::stride, 1], truth[::stride, 2], color=GREEN, lw=1.8, label="Truth")
    ax.plot(estimate[::stride, 0], estimate[::stride, 1], estimate[::stride, 2], color=BLUE, lw=1.2, label="Estimate")
    if debris.size:
        ax.plot(debris[::stride, 0], debris[::stride, 1], debris[::stride, 2], color=ORANGE, lw=1.5, label="Debris")

    ax.set_title("INOAS orbital context", color=NAVY, fontweight="bold", pad=14)
    ax.set_xlabel("ECI x [km]")
    ax.set_ylabel("ECI y [km]")
    ax.set_zlabel("ECI z [km]")
    ax.legend(loc="upper left")
    ax.grid(True, alpha=0.35)
    set_equal_3d(ax, np.vstack([truth, reference, estimate, debris]) if debris.size else np.vstack([truth, reference, estimate]))
    fig.tight_layout()
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    return path


def save_debris_distance(outdir: Path, data: dict) -> Path:
    time = data["time"]
    distance = data["distance"]
    nominal_distance = data["nominal_distance"]
    position_error = data["position_error"]
    safe_radius = data["safe_radius"]
    dynamic_time = data["dynamic_safe_time"]
    dynamic_first = data["dynamic_safe_first"]
    dynamic_horizon = data["dynamic_safe_horizon"]
    t_debris = data["t_debris"]

    path = outdir / "inoas_debris_distance.png"
    fig, axes = plt.subplots(2, 1, figsize=(12.5, 8.0), dpi=160, sharex=True)

    ax = axes[0]
    ax.plot(time, distance, color=GREEN, lw=2.3, label="Controlled trajectory")
    ax.plot(time, nominal_distance, "--", color=GREY, lw=1.8, label="Nominal reference")
    if np.isfinite(safe_radius):
        ax.axhline(safe_radius, color=ORANGE, ls="--", lw=1.6, label="Baseline safety radius")
    if dynamic_time.size and dynamic_first.size:
        ax.plot(dynamic_time, dynamic_first, color=MAGENTA, lw=1.2, label="Covariance-aware radius")
    if dynamic_time.size and dynamic_horizon.size:
        ax.plot(dynamic_time, dynamic_horizon, color=BLUE, lw=1.1, alpha=0.85, label="Horizon max radius")
    if np.isfinite(t_debris):
        ax.axvline(t_debris, color=MAGENTA, ls=":", lw=1.8)
    ax.set_ylabel("Distance [m]")
    ax.set_title("Debris separation and safety margin", color=NAVY, fontweight="bold")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="best", ncols=2)

    ax = axes[1]
    ax.plot(time, position_error, color=BLUE, lw=2.0)
    if np.isfinite(t_debris):
        ax.axvline(t_debris, color=MAGENTA, ls=":", lw=1.8)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Position error [m]")
    ax.set_title("Reference-tracking error", color=NAVY, fontweight="bold")
    ax.grid(True, alpha=0.35)

    fig.tight_layout()
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    return path


def save_control_energy_summary(outdir: Path, data: dict, gnss_power_w: float) -> Path:
    time = data["time"]
    control_time = data["control_time"]
    control_norm = np.linalg.norm(data["control"], axis=1)
    delta_v = cumulative_trapezoid(control_norm, control_time)

    lambda_time = data["lambda_time"]
    lambda_raw = data["lambda"]
    has_lambda = lambda_time.size > 0 and lambda_raw.size > 0
    lambda_on = zoh_sample(lambda_time, lambda_raw, time)
    lambda_on = np.where(lambda_on > 0.5, 1.0, 0.0)

    on_time = cumulative_trapezoid(lambda_on, time)
    always_energy = gnss_power_w * (time - time[0])
    duty_energy = gnss_power_w * on_time
    saved_energy = always_energy - duty_energy

    path = outdir / "inoas_control_energy_summary.png"
    fig, axes = plt.subplots(3, 1, figsize=(12.5, 9.4), dpi=160, sharex=False)

    axes[0].plot(control_time, delta_v, color=GREEN, lw=2.4)
    axes[0].set_title("Cumulative maneuver Delta-V", color=NAVY, fontweight="bold")
    axes[0].set_xlabel("Time [s]")
    axes[0].set_ylabel("Delta-V [m/s]")
    axes[0].grid(True, alpha=0.35)

    axes[1].step(time, lambda_on, where="post", color=MAGENTA if has_lambda else GREY, lw=2.0)
    axes[1].set_ylim(-0.08, 1.08)
    axes[1].set_yticks([0, 1], ["Kalman", "GNSS"])
    axes[1].set_title("Navigation mode selector", color=NAVY, fontweight="bold")
    axes[1].set_xlabel("Time [s]")
    axes[1].grid(True, alpha=0.35)
    if not has_lambda:
        axes[1].text(0.02, 0.15, "No logged selector found; always-on GNSS shown as fallback.", transform=axes[1].transAxes, color=GREY)

    axes[2].plot(time, always_energy, "--", color=GREY, lw=2.0, label="GNSS always on")
    axes[2].plot(time, duty_energy, color=GREEN, lw=2.4, label="Duty-cycled GNSS")
    axes[2].fill_between(time, duty_energy, always_energy, color=GREEN, alpha=0.13, label="Energy saved")
    axes[2].set_title("GNSS receiver energy estimate", color=NAVY, fontweight="bold")
    axes[2].set_xlabel("Time [s]")
    axes[2].set_ylabel("Energy [J]")
    axes[2].grid(True, alpha=0.35)
    axes[2].legend(loc="best")

    fig.tight_layout()
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    return path


def save_debris_gif(outdir: Path, data: dict, frames: int, fps: int, window_half_width: float) -> Path | None:
    time = data["time"]
    debris = data["debris"]
    if debris.size == 0:
        return None

    t_debris = data["t_debris"]
    if not np.isfinite(t_debris):
        t_debris = float(time[np.argmin(data["distance"])])

    mask = (time >= t_debris - window_half_width) & (time <= t_debris + window_half_width)
    if not np.any(mask):
        mask = np.ones_like(time, dtype=bool)

    idx_window = np.where(mask)[0]
    frame_indices = np.unique(np.linspace(idx_window[0], idx_window[-1], max(2, frames)).astype(int))

    sc_to_debris_rtn = rtn_components(data["reference"], data["reference_velocity"], data["truth"] - debris)
    ref_to_debris_rtn = rtn_components(data["reference"], data["reference_velocity"], data["reference"] - debris)

    # Plot along-track horizontally and radial vertically, centered on debris.
    x_sc = sc_to_debris_rtn[:, 1]
    y_sc = sc_to_debris_rtn[:, 0]
    x_ref = ref_to_debris_rtn[:, 1]
    y_ref = ref_to_debris_rtn[:, 0]

    window_values = np.concatenate([x_sc[idx_window], y_sc[idx_window], x_ref[idx_window], y_ref[idx_window]])
    safe_radius = data["safe_radius"] if np.isfinite(data["safe_radius"]) else 150.0
    limit = max(float(np.nanmax(np.abs(window_values))) * 1.15, safe_radius * 1.35, 25.0)

    distance = data["distance"]
    path = outdir / "inoas_debris_avoidance.gif"
    frames_dir = outdir / "_gif_frames_tmp"
    if frames_dir.exists():
        shutil.rmtree(frames_dir)
    frames_dir.mkdir(parents=True, exist_ok=True)

    image_paths: list[Path] = []
    theta = np.linspace(0.0, 2.0 * np.pi, 240)

    for frame_number, idx in enumerate(frame_indices):
        frame_path = frames_dir / f"frame_{frame_number:04d}.png"
        fig, axes = plt.subplots(1, 2, figsize=(12.5, 5.6), dpi=130, gridspec_kw={"width_ratios": [1.05, 1.0]})

        ax = axes[0]
        ax.plot(x_ref[idx_window], y_ref[idx_window], "--", color=GREY, lw=1.6, label="Nominal reference")
        partial = idx_window[idx_window <= idx]
        ax.plot(x_sc[partial], y_sc[partial], color=GREEN, lw=2.2, label="Controlled trajectory")
        ax.plot(0, 0, "x", color=ORANGE, ms=9, mew=2.3, label="Debris")
        ax.plot(x_sc[idx], y_sc[idx], "o", color=GREEN, ms=8)
        ax.plot(safe_radius * np.cos(theta), safe_radius * np.sin(theta), color=ORANGE, ls="--", lw=1.5, label="Safety radius")
        ax.set_xlim(-limit, limit)
        ax.set_ylim(-limit, limit)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("In-track wrt debris [m]")
        ax.set_ylabel("Radial wrt debris [m]")
        ax.set_title(f"Debris avoidance geometry | t = {time[idx]:.0f} s", color=NAVY, fontweight="bold")
        ax.grid(True, alpha=0.35)
        ax.legend(loc="upper right", fontsize=8)

        ax = axes[1]
        ax.plot(time[idx_window], distance[idx_window], color=LIGHT_GREY, lw=2.2)
        ax.plot(time[partial], distance[partial], color=BLUE, lw=2.5)
        ax.axvline(time[idx], color=MAGENTA, ls=":", lw=1.8)
        if np.isfinite(data["safe_radius"]):
            ax.axhline(data["safe_radius"], color=ORANGE, ls="--", lw=1.5)
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Distance to debris [m]")
        ax.set_title("Separation over time", color=NAVY, fontweight="bold")
        ax.grid(True, alpha=0.35)

        fig.tight_layout()
        fig.savefig(frame_path, bbox_inches="tight")
        plt.close(fig)
        image_paths.append(frame_path)

    images = [Image.open(frame) for frame in image_paths]
    duration_ms = max(20, int(1000 / max(1, fps)))
    images[0].save(path, save_all=True, append_images=images[1:], duration=duration_ms, loop=0)
    for image in images:
        image.close()
    shutil.rmtree(frames_dir)
    return path


def load_data(path: Path) -> dict:
    mat = loadmat(path, squeeze_me=True, struct_as_record=False)

    time = mat_1d(mat, "time_s")
    truth = mat_2d(mat, "truth_eci_m", 3)
    estimate = mat_2d(mat, "estimated_eci_m", 3)
    reference = mat_2d(mat, "reference_eci_m", 3)
    reference_velocity = mat_2d(mat, "reference_velocity_eci_mps", 3)
    debris = mat_2d(mat, "debris_eci_m_at_time", 3)
    control_time = mat_1d(mat, "control_time_s")
    control = mat_2d(mat, "control_eci_mps2", 3)

    mask = finite_rows(time, truth, estimate, reference, reference_velocity)
    time = time[mask]
    truth = truth[mask]
    estimate = estimate[mask]
    reference = reference[mask]
    reference_velocity = reference_velocity[mask]
    if debris.shape[0] == mask.size:
        debris = debris[mask]
    else:
        debris = np.empty((0, 0))

    control_mask = finite_rows(control_time, control)
    control_time = control_time[control_mask]
    control = control[control_mask]

    position_error = np.linalg.norm(truth - reference, axis=1)
    if debris.size:
        distance = np.linalg.norm(truth - debris, axis=1)
        nominal_distance = np.linalg.norm(reference - debris, axis=1)
    else:
        distance = np.full_like(time, np.nan)
        nominal_distance = np.full_like(time, np.nan)

    return {
        "time": time,
        "truth": truth,
        "estimate": estimate,
        "reference": reference,
        "reference_velocity": reference_velocity,
        "debris": debris,
        "control_time": control_time,
        "control": control,
        "position_error": position_error,
        "distance": distance,
        "nominal_distance": nominal_distance,
        "safe_radius": mat_scalar(mat, "safe_radius_m"),
        "t_debris": mat_scalar(mat, "t_debris_s"),
        "u_max": mat_scalar(mat, "u_max_mps2"),
        "mpc_horizon": mat_scalar(mat, "mpc_horizon"),
        "sample_time": mat_scalar(mat, "sample_time_s"),
        "dynamic_safe_time": mat_1d(mat, "dynamic_safe_time_s"),
        "dynamic_safe_first": mat_1d(mat, "dynamic_safe_first_m"),
        "dynamic_safe_horizon": mat_1d(mat, "dynamic_safe_horizon_m"),
        "lambda_time": mat_1d(mat, "lambda_time_s"),
        "lambda": mat_1d(mat, "lambda"),
    }


def write_summary(outdir: Path, data: dict, assets: list[Path], gnss_power_w: float) -> Path:
    control_norm = np.linalg.norm(data["control"], axis=1)
    delta_v = cumulative_trapezoid(control_norm, data["control_time"])
    final_delta_v = float(delta_v[-1]) if delta_v.size else math.nan

    lambda_time = data["lambda_time"]
    lambda_raw = data["lambda"]
    has_lambda = lambda_time.size > 0 and lambda_raw.size > 0
    lambda_on = zoh_sample(lambda_time, lambda_raw, data["time"])
    lambda_on = np.where(lambda_on > 0.5, 1.0, 0.0)
    on_time = cumulative_trapezoid(lambda_on, data["time"])
    duration = max(float(data["time"][-1] - data["time"][0]), 1e-9)
    duty_ratio = float(on_time[-1] / duration) if on_time.size else math.nan
    energy_always = gnss_power_w * duration
    energy_duty = gnss_power_w * float(on_time[-1]) if on_time.size else math.nan
    energy_saved = energy_always - energy_duty if np.isfinite(energy_duty) else math.nan

    min_distance = float(np.nanmin(data["distance"])) if data["distance"].size else math.nan
    min_distance_idx = int(np.nanargmin(data["distance"])) if data["distance"].size and np.any(np.isfinite(data["distance"])) else 0
    min_distance_time = float(data["time"][min_distance_idx]) if data["time"].size else math.nan

    path = outdir / "inoas_visualization_summary.txt"
    lines = [
        "INOAS visualization summary",
        f"duration_s = {duration:.3f}",
        f"mpc_horizon = {data['mpc_horizon']:.0f}",
        f"sample_time_s = {data['sample_time']:.3f}",
        f"t_debris_s = {data['t_debris']:.3f}",
        f"safe_radius_m = {data['safe_radius']:.3f}",
        f"minimum_debris_distance_m = {min_distance:.3f}",
        f"minimum_debris_distance_time_s = {min_distance_time:.3f}",
        f"final_position_error_m = {data['position_error'][-1]:.3f}",
        f"maximum_position_error_m = {np.nanmax(data['position_error']):.3f}",
        f"final_delta_v_mps = {final_delta_v:.6f}",
        f"gnss_selector_logged = {str(has_lambda).lower()}",
        f"gnss_duty_ratio = {duty_ratio:.6f}",
        f"gnss_power_w = {gnss_power_w:.3f}",
        f"gnss_energy_always_on_j = {energy_always:.3f}",
        f"gnss_energy_duty_cycle_j = {energy_duty:.3f}",
        f"gnss_energy_saved_j = {energy_saved:.3f}",
        "",
        "assets:",
    ]
    lines.extend(f"- {asset.as_posix()}" for asset in assets if asset is not None)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def main() -> None:
    args = parse_args()
    if not args.mat.exists():
        raise FileNotFoundError(
            f"MAT file not found: {args.mat}. Run export_visualization_data in MATLAB first."
        )

    args.out.mkdir(parents=True, exist_ok=True)
    data = load_data(args.mat)

    assets: list[Path] = []
    assets.append(save_orbit_context(args.out, data))
    if data["debris"].size:
        assets.append(save_debris_distance(args.out, data))
        gif = save_debris_gif(args.out, data, args.frames, args.fps, args.window_half_width)
        if gif is not None:
            assets.append(gif)
    assets.append(save_control_energy_summary(args.out, data, args.gnss_power_w))
    assets.append(write_summary(args.out, data, assets, args.gnss_power_w))

    print("Generated INOAS visualization assets:")
    for asset in assets:
        print(f"  {asset}")


if __name__ == "__main__":
    main()
