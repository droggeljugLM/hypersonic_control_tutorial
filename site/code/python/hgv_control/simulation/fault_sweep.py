"""Batch fault-injection scenarios and emit paper-style CSV tables."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

from hgv_control.simulation.fault_injection import (
    CONTROLLER_NAMES,
    FaultConfig,
    default_magnitude,
    run,
)


DEFAULT_FAULTS = [
    "none",
    "alpha_sensor_bias",
    "elevator_loss",
    "elevator_stuck",
    "throttle_loss",
    "input_delay",
]

PAIR_FIELDS = [
    "h_rms",
    "v_rms",
    "q_max",
    "q_violation_integral",
    "post_fault_h_rms",
    "post_fault_v_rms",
    "post_fault_q_violation_integral",
    "fault_detection_delay",
    "fault_residual_max",
    "q_est_margin_min",
]


def scenario_start_time(fault_type: str) -> float:
    return 0.0 if fault_type == "none" else 18.0


def run_sweep(
    controllers: list[str],
    faults: list[str],
    duration: float = 45.0,
    dt: float = 0.02,
) -> list[dict[str, float | bool | str]]:
    rows: list[dict[str, float | bool | str]] = []
    for fault_type in faults:
        config = FaultConfig(
            fault_type=fault_type,
            start_time=scenario_start_time(fault_type),
            magnitude=default_magnitude(fault_type),
        )
        for controller_name in controllers:
            _, metrics = run(controller_name, config, duration=duration, dt=dt)
            row: dict[str, float | bool | str] = {
                "controller": controller_name,
                "fault_type": fault_type,
                "fault_start_time": config.start_time,
                "fault_magnitude": config.magnitude,
            }
            row.update(metrics)
            rows.append(row)
    return rows


def _as_float(value: object) -> float:
    if isinstance(value, bool):
        return 1.0 if value else 0.0
    try:
        numeric = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return math.nan
    return numeric


def paired_fault_deltas(
    rows: list[dict[str, float | bool | str]],
    baseline: str,
    candidate: str,
) -> list[dict[str, float | bool | str]]:
    by_key = {(row["fault_type"], row["controller"]): row for row in rows}
    pairs: list[dict[str, float | bool | str]] = []
    for fault_type, _controller in sorted(by_key):
        if _controller != baseline:
            continue
        baseline_row = by_key.get((fault_type, baseline))
        candidate_row = by_key.get((fault_type, candidate))
        if baseline_row is None or candidate_row is None:
            continue
        pair: dict[str, float | bool | str] = {
            "fault_type": fault_type,
            "baseline": baseline,
            "candidate": candidate,
            "baseline_pass": baseline_row["pass"],
            "candidate_pass": candidate_row["pass"],
            "baseline_fault_detected": baseline_row["fault_detected"],
            "candidate_fault_detected": candidate_row["fault_detected"],
        }
        for field in PAIR_FIELDS:
            pair[f"baseline_{field}"] = baseline_row[field]
            pair[f"candidate_{field}"] = candidate_row[field]
            pair[f"delta_{field}"] = _as_float(candidate_row[field]) - _as_float(baseline_row[field])
        pairs.append(pair)
    return pairs


def summarize_pairs(pairs: list[dict[str, float | bool | str]]) -> dict[str, float]:
    if not pairs:
        return {
            "pairs": 0.0,
            "candidate_pass_rate": 0.0,
            "baseline_pass_rate": 0.0,
            "safety_improved_rate": 0.0,
            "mean_delta_post_fault_q_violation_integral": 0.0,
            "mean_delta_post_fault_h_rms": 0.0,
            "mean_delta_post_fault_v_rms": 0.0,
        }

    def mean(field: str) -> float:
        values = [_as_float(pair[field]) for pair in pairs]
        finite = [value for value in values if math.isfinite(value)]
        return sum(finite) / len(finite) if finite else math.nan

    safety_improved = [
        _as_float(pair["delta_post_fault_q_violation_integral"]) <= 0.0
        and _as_float(pair["delta_q_max"]) <= 0.0
        for pair in pairs
    ]
    return {
        "pairs": float(len(pairs)),
        "candidate_pass_rate": sum(bool(pair["candidate_pass"]) for pair in pairs) / len(pairs),
        "baseline_pass_rate": sum(bool(pair["baseline_pass"]) for pair in pairs) / len(pairs),
        "safety_improved_rate": sum(safety_improved) / len(safety_improved),
        "mean_delta_post_fault_q_violation_integral": mean("delta_post_fault_q_violation_integral"),
        "mean_delta_post_fault_h_rms": mean("delta_post_fault_h_rms"),
        "mean_delta_post_fault_v_rms": mean("delta_post_fault_v_rms"),
    }


def write_rows(rows: list[dict[str, float | bool | str]], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0].keys()) if rows else []
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--controllers", nargs="+", choices=sorted(CONTROLLER_NAMES), default=["baseline", "guarded"])
    parser.add_argument("--faults", nargs="+", default=DEFAULT_FAULTS)
    parser.add_argument("--baseline", choices=sorted(CONTROLLER_NAMES), default="baseline")
    parser.add_argument("--candidate", choices=sorted(CONTROLLER_NAMES), default="guarded")
    parser.add_argument("--duration", type=float, default=45.0)
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--metrics-output", type=Path, default=None)
    parser.add_argument("--pairs-output", type=Path, default=None)
    args = parser.parse_args()

    unknown_faults = sorted(set(args.faults) - set(DEFAULT_FAULTS))
    if unknown_faults:
        raise SystemExit(f"Unsupported fault type(s): {', '.join(unknown_faults)}")

    controllers = list(dict.fromkeys([*args.controllers, args.baseline, args.candidate]))
    rows = run_sweep(controllers, args.faults, duration=args.duration, dt=args.dt)
    pairs = paired_fault_deltas(rows, baseline=args.baseline, candidate=args.candidate)
    summary = summarize_pairs(pairs)

    if args.metrics_output is not None:
        write_rows(rows, args.metrics_output)
    if args.pairs_output is not None:
        write_rows(pairs, args.pairs_output)

    for controller_name in controllers:
        controller_rows = [row for row in rows if row["controller"] == controller_name]
        pass_rate = sum(bool(row["pass"]) for row in controller_rows) / len(controller_rows)
        detected_faults = [
            row for row in controller_rows if row["fault_type"] != "none" and bool(row["fault_detected"])
        ]
        detectable = [row for row in controller_rows if row["fault_type"] != "none"]
        detection_rate = len(detected_faults) / len(detectable) if detectable else 0.0
        print(f"{controller_name}: scenarios={len(controller_rows)}, pass_rate={pass_rate:.3f}, detection_rate={detection_rate:.3f}")

    print(
        f"paired {args.candidate} - {args.baseline}: "
        f"pairs={summary['pairs']:.0f}, "
        f"candidate_pass_rate={summary['candidate_pass_rate']:.3f}, "
        f"baseline_pass_rate={summary['baseline_pass_rate']:.3f}, "
        f"safety_improved_rate={summary['safety_improved_rate']:.3f}, "
        f"mean_delta_post_fault_q_violation_integral={summary['mean_delta_post_fault_q_violation_integral']:.3f}, "
        f"mean_delta_post_fault_h_rms={summary['mean_delta_post_fault_h_rms']:.3f}, "
        f"mean_delta_post_fault_v_rms={summary['mean_delta_post_fault_v_rms']:.3f}"
    )


if __name__ == "__main__":
    main()
