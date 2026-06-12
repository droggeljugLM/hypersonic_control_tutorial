"""Generate SVG figures for the tutorial's reproducible example."""

from __future__ import annotations

import argparse
from pathlib import Path

from hgv_control.plotting.svg import bar_chart, line_plot, read_numeric_csv


def column(rows: list[dict[str, float | str]], name: str) -> list[float]:
    return [float(row[name]) for row in rows]


def comparison_label(rows: list[dict[str, float | str]]) -> str:
    if not rows:
        return "candidate - baseline"
    baseline = str(rows[0].get("baseline", "baseline"))
    candidate = str(rows[0].get("candidate", "candidate"))
    return f"{candidate} - {baseline}"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline-trace", type=Path, required=True)
    parser.add_argument("--guarded-trace", type=Path, required=True)
    parser.add_argument("--lqr-trace", type=Path, default=None)
    parser.add_argument("--ndi-trace", type=Path, default=None)
    parser.add_argument("--backstepping-trace", type=Path, default=None)
    parser.add_argument("--smc-trace", type=Path, default=None)
    parser.add_argument("--barrier-trace", type=Path, default=None)
    parser.add_argument("--pairs-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--q-limit", type=float, default=70_000.0)
    args = parser.parse_args()

    baseline = read_numeric_csv(args.baseline_trace)
    guarded = read_numeric_csv(args.guarded_trace)
    lqr = read_numeric_csv(args.lqr_trace) if args.lqr_trace is not None else None
    ndi = read_numeric_csv(args.ndi_trace) if args.ndi_trace is not None else None
    backstepping = read_numeric_csv(args.backstepping_trace) if args.backstepping_trace is not None else None
    smc = read_numeric_csv(args.smc_trace) if args.smc_trace is not None else None
    barrier = read_numeric_csv(args.barrier_trace) if args.barrier_trace is not None else None
    pairs = read_numeric_csv(args.pairs_csv)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    altitude_series = [
        ("reference", column(baseline, "time"), column(baseline, "altitude_ref"), "#222222"),
        ("baseline", column(baseline, "time"), column(baseline, "altitude"), "#2f6fbb"),
        ("guarded", column(guarded, "time"), column(guarded, "altitude"), "#258a45"),
    ]
    velocity_series = [
        ("reference", column(baseline, "time"), column(baseline, "velocity_ref"), "#222222"),
        ("baseline", column(baseline, "time"), column(baseline, "velocity"), "#2f6fbb"),
        ("guarded", column(guarded, "time"), column(guarded, "velocity"), "#258a45"),
    ]
    qbar_series = [
        ("baseline", column(baseline, "time"), column(baseline, "qbar"), "#2f6fbb"),
        ("guarded", column(guarded, "time"), column(guarded, "qbar"), "#258a45"),
    ]
    if lqr is not None:
        altitude_series.append(("lqr", column(lqr, "time"), column(lqr, "altitude"), "#8a4fbf"))
        velocity_series.append(("lqr", column(lqr, "time"), column(lqr, "velocity"), "#8a4fbf"))
        qbar_series.append(("lqr", column(lqr, "time"), column(lqr, "qbar"), "#8a4fbf"))
    if ndi is not None:
        altitude_series.append(("ndi", column(ndi, "time"), column(ndi, "altitude"), "#c47f1f"))
        velocity_series.append(("ndi", column(ndi, "time"), column(ndi, "velocity"), "#c47f1f"))
        qbar_series.append(("ndi", column(ndi, "time"), column(ndi, "qbar"), "#c47f1f"))
    if backstepping is not None:
        altitude_series.append(("backstep", column(backstepping, "time"), column(backstepping, "altitude"), "#b43c52"))
        velocity_series.append(("backstep", column(backstepping, "time"), column(backstepping, "velocity"), "#b43c52"))
        qbar_series.append(("backstep", column(backstepping, "time"), column(backstepping, "qbar"), "#b43c52"))
    if smc is not None:
        altitude_series.append(("smc", column(smc, "time"), column(smc, "altitude"), "#5c8f8f"))
        velocity_series.append(("smc", column(smc, "time"), column(smc, "velocity"), "#5c8f8f"))
        qbar_series.append(("smc", column(smc, "time"), column(smc, "qbar"), "#5c8f8f"))
    if barrier is not None:
        altitude_series.append(("barrier", column(barrier, "time"), column(barrier, "altitude"), "#6a7d2c"))
        velocity_series.append(("barrier", column(barrier, "time"), column(barrier, "velocity"), "#6a7d2c"))
        qbar_series.append(("barrier", column(barrier, "time"), column(barrier, "qbar"), "#6a7d2c"))

    line_plot(
        args.output_dir / "altitude_tracking.svg",
        "Altitude Tracking",
        "time (s)",
        "altitude (m)",
        altitude_series,
    )
    line_plot(
        args.output_dir / "velocity_tracking.svg",
        "Velocity Tracking",
        "time (s)",
        "velocity (m/s)",
        velocity_series,
    )
    line_plot(
        args.output_dir / "dynamic_pressure.svg",
        "Dynamic Pressure",
        "time (s)",
        "qbar (Pa)",
        qbar_series,
        hlines=[("q limit", args.q_limit, "#c23b3b")],
    )
    labels = [str(int(row["seed"])) for row in pairs]
    bar_chart(
        args.output_dir / "paired_delta_q_violation.svg",
        "Paired Dynamic-Pressure Violation Difference",
        comparison_label(pairs),
        labels,
        column(pairs, "delta_q_violation_integral"),
    )


if __name__ == "__main__":
    main()
