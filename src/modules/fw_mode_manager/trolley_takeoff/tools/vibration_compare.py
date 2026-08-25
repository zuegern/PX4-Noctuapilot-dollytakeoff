#!/usr/bin/env python3

"""Compare trolley ground-roll vibration between designs, excluding the standstill at each run's ends.

Group the runs by trolley; the tool keeps only the *moving* part of every run (ground speed above
--speed), so the brief stops at the start and end of a run do not dilute the result. For each trolley
it reports, per body axis:

    * RMS vibration: acceleration [m/s^2] and angular rate [deg/s]
    * PX4's own accel / gyro vibration metric and the accelerometer clipping count (vehicle_imu_status)

and draws ONE comparison figure: RMS bars per axis per trolley plus the overlaid acceleration spectra.

Usage:
    python3 vibration_compare.py "Trolley A=log_10.ulg,log_11.ulg" "Trolley B=log_20.ulg,log_21.ulg"
    python3 vibration_compare.py --speed 0.5 --logdir /path/to/logs --out compare.png "A=..." "B=..."

Reuses export_trolley_debug.py (same folder). Requires: pyulog, numpy, matplotlib.
"""

import argparse
import importlib.util
from pathlib import Path

import numpy as np
from pyulog import ULog

# Reuse the maintained exporter's extraction helpers (its __main__ guard makes importing safe).
_TOOL = Path(__file__).with_name("export_trolley_debug.py")
_spec = importlib.util.spec_from_file_location("export_trolley_debug", _TOOL)
etd = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(etd)

ACCEL = [("accel_forward_m_s2", "X fwd"), ("accel_right_m_s2", "Y right"), ("accel_down_m_s2", "Z down")]
GYRO = [("gyro_roll_rad_s", "roll"), ("gyro_pitch_rad_s", "pitch"), ("gyro_yaw_rad_s", "yaw")]
COLORS = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd", "#e8890c"]


def ground_speed_series(ulog):
    for dataset in ulog.data_list:
        if dataset.name == "vehicle_local_position":
            timestamps = np.asarray(dataset.data["timestamp"], dtype=np.int64)
            vx = np.asarray(dataset.data["vx"], dtype=float)
            vy = np.asarray(dataset.data["vy"], dtype=float)
            return timestamps, np.hypot(vx, vy)
    return None, None


def moving_mask(target_timestamps, ulog, speed_threshold):
    """Boolean mask over target_timestamps: True while the trolley is moving (drops the end stops)."""
    timestamps, speed = ground_speed_series(ulog)

    if timestamps is None:
        return np.ones(len(target_timestamps), dtype=bool), None

    mapped, _ = etd.nearest_samples(target_timestamps, timestamps, speed)
    return mapped > speed_threshold, mapped


def px4_vibration_metric(ulog, speed_threshold):
    """Median accel/gyro vibration metric and clipping increase over the moving window."""
    result = {"accel_metric": np.nan, "gyro_metric": np.nan, "accel_clip": 0, "gyro_clip": 0}

    for dataset in ulog.data_list:
        if dataset.name == "vehicle_imu_status" and getattr(dataset, "multi_id", 0) == 0:
            timestamps = np.asarray(dataset.data["timestamp"], dtype=np.int64)
            moving, _ = moving_mask(timestamps, ulog, speed_threshold)

            if moving.sum() < 2:
                moving = np.ones(len(timestamps), dtype=bool)

            def median(field):
                values = np.asarray(dataset.data[field], dtype=float)[moving]
                return float(np.nanmedian(values)) if len(values) else np.nan

            def clip_increase(prefix):
                total = 0.0
                for axis in range(3):
                    values = np.asarray(dataset.data[f"{prefix}[{axis}]"], dtype=float)[moving]
                    if len(values):
                        total += max(0.0, float(values[-1] - values[0]))
                return int(total)

            result["accel_metric"] = median("accel_vibration_metric")
            result["gyro_metric"] = median("gyro_vibration_metric")
            result["accel_clip"] = clip_increase("accel_clipping")
            result["gyro_clip"] = clip_increase("gyro_clipping")
            break

    return result


def rms_ac(values):
    """RMS about the mean (removes gravity / steady acceleration, leaving the vibration)."""
    values = values[np.isfinite(values)]
    return float(np.sqrt(np.mean((values - np.mean(values)) ** 2))) if len(values) else np.nan


def analyse_log(path, speed_threshold):
    ulog = ULog(str(path))
    vibration = etd.extract_vibration_data(ulog)

    if vibration is None:
        print(f"  ! {path.name}: no IMU data, skipped")
        return None

    timestamps = vibration["timestamp_us"]
    moving, speed = moving_mask(timestamps, ulog, speed_threshold)
    rate = etd.sample_rate_hz(timestamps)

    if moving.sum() < 50:
        print(f"  ! {path.name}: only {int(moving.sum())} moving samples above {speed_threshold} m/s "
              "(check the run or lower --speed)")

    result = {
        "name": path.name,
        "n_move": int(moving.sum()),
        "dur_move": float(moving.sum() / rate) if rate == rate else np.nan,
        "vmax": float(np.nanmax(speed)) if speed is not None else np.nan,
        "spectra": etd.compute_spectra(vibration, moving, rate),
    }

    for key, _ in ACCEL:
        result["rms_" + key] = rms_ac(vibration[key][moving])

    for key, _ in GYRO:
        result["rms_" + key] = rms_ac(np.degrees(vibration[key][moving]))

    result.update(px4_vibration_metric(ulog, speed_threshold))
    return result


def parse_groups(specs, logdir):
    groups = {}

    for spec in specs:
        if "=" not in spec:
            raise SystemExit(f"Expected 'Label=log1.ulg,log2.ulg', got: {spec!r}")

        label, files = spec.split("=", 1)
        paths = [Path(logdir) / name.strip() for name in files.split(",") if name.strip()]

        for path in paths:
            if not path.exists():
                raise SystemExit(f"Log not found: {path}")

        groups[label.strip()] = paths

    return groups


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("groups", nargs="+", help="Trolley groups: 'Label=log1.ulg,log2.ulg'")
    parser.add_argument("--speed", type=float, default=0.5,
                        help="Ground speed [m/s] above which the trolley counts as moving (default 0.5)")
    parser.add_argument("--logdir", default=".", help="Folder the log names are relative to")
    parser.add_argument("--out", default="vibration_compare.png", help="Comparison figure path")
    args = parser.parse_args()

    groups = parse_groups(args.groups, args.logdir)
    per_group = {}

    header = (f"{'log':32} {'move[s]':>7} {'vmax':>5} | {'aX':>5} {'aY':>5} {'aZ':>5} "
              f"| {'gR':>5} {'gP':>5} {'gY':>5} | {'accM':>5} {'gyrM':>5} {'clip':>5}")

    for label, paths in groups.items():
        print(f"\n=== {label} ===  (moving = ground speed > {args.speed} m/s; standstill excluded)")
        print(header)
        rows = []

        for path in paths:
            row = analyse_log(path, args.speed)

            if row is None:
                continue

            rows.append(row)
            print(f"{row['name'][:32]:32} {row['dur_move']:7.1f} {row['vmax']:5.1f} | "
                  f"{row['rms_accel_forward_m_s2']:5.2f} {row['rms_accel_right_m_s2']:5.2f} "
                  f"{row['rms_accel_down_m_s2']:5.2f} | {row['rms_gyro_roll_rad_s']:5.1f} "
                  f"{row['rms_gyro_pitch_rad_s']:5.1f} {row['rms_gyro_yaw_rad_s']:5.1f} | "
                  f"{row['accel_metric']:5.1f} {row['gyro_metric']:5.1f} {row['accel_clip']:5d}")

        if not rows:
            continue

        mean = {k: float(np.nanmean([r[k] for r in rows]))
                for k in rows[0] if k.startswith("rms_") or k in ("accel_metric", "gyro_metric")}
        mean["accel_clip"] = int(np.sum([r["accel_clip"] for r in rows]))
        per_group[label] = {"rows": rows, "mean": mean}
        print(f"{'  -> mean (' + str(len(rows)) + ' runs)':32} {'':7} {'':5} | "
              f"{mean['rms_accel_forward_m_s2']:5.2f} {mean['rms_accel_right_m_s2']:5.2f} "
              f"{mean['rms_accel_down_m_s2']:5.2f} | {mean['rms_gyro_roll_rad_s']:5.1f} "
              f"{mean['rms_gyro_pitch_rad_s']:5.1f} {mean['rms_gyro_yaw_rad_s']:5.1f} | "
              f"{mean['accel_metric']:5.1f} {mean['gyro_metric']:5.1f} {mean['accel_clip']:5d}")

    if not per_group:
        raise SystemExit("No usable IMU data in any group")

    make_figure(per_group, args.out)
    print(f"\nwrote {args.out}")


def make_figure(per_group, out_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    labels = list(per_group)
    plt.rcParams.update({"font.size": 11, "axes.grid": True, "grid.alpha": 0.3})
    fig, axes = plt.subplots(3, 1, figsize=(9, 11))

    # (a) accel RMS, (b) gyro RMS: grouped bars per axis
    for panel, (axis_defs, unit, title) in enumerate([
            (ACCEL, "m/s²", "(a) Acceleration vibration RMS — lower is better"),
            (GYRO, "deg/s", "(b) Angular-rate vibration RMS — lower is better")]):
        ax = axes[panel]
        width = 0.8 / len(labels)
        x = np.arange(len(axis_defs))

        for i, label in enumerate(labels):
            mean = per_group[label]["mean"]
            heights = [mean["rms_" + key] for key, _ in axis_defs]
            ax.bar(x + i * width, heights, width, color=COLORS[i % len(COLORS)], label=label)

        ax.set_xticks(x + width * (len(labels) - 1) / 2)
        ax.set_xticklabels([lbl for _, lbl in axis_defs])
        ax.set_ylabel(f"RMS [{unit}]")
        ax.set_title(title)
        ax.legend(fontsize=9)

    # (c) vertical-axis acceleration spectrum, one thin line per run, coloured by trolley
    ax = axes[2]
    for i, label in enumerate(labels):
        for j, row in enumerate(per_group[label]["rows"]):
            freqs, psd = row["spectra"]["accel_down_m_s2"]
            if freqs is None:
                continue
            ax.semilogy(freqs, psd, color=COLORS[i % len(COLORS)], lw=1.0, alpha=0.75,
                        label=label if j == 0 else None)
    ax.set_xlabel("Frequency [Hz]")
    ax.set_ylabel("PSD [(m/s²)²/Hz]")
    ax.set_title("(c) Vertical (Z) acceleration spectrum — where the vibration energy sits")
    ax.legend(fontsize=9)
    ax.set_xlim(0, None)

    fig.suptitle("Trolley ground-roll vibration (standstill excluded)", fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.98])
    fig.savefig(out_path, dpi=130)
    plt.close(fig)


if __name__ == "__main__":
    main()
