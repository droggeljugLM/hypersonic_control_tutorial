"""Generate SVG figures for fault-sweep teaching results."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

from hgv_control.plotting.svg import bar_chart, read_numeric_csv


def _finite_or_zero(value: float | str) -> float:
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return 0.0
    return numeric if math.isfinite(numeric) else 0.0


def _fault_labels(rows: list[dict[str, float | str]]) -> list[str]:
    return [str(row["fault_type"]).replace("_", " ") for row in rows]


def _pair_values(rows: list[dict[str, float | str]], field: str) -> list[float]:
    return [_finite_or_zero(row[field]) for row in rows]


def _candidate_detection_rows(
    rows: list[dict[str, float | str]],
    candidate: str,
) -> list[dict[str, float | str]]:
    selected = [
        row
        for row in rows
        if str(row.get("controller", "")) == candidate
        and str(row.get("fault_type", "")) != "none"
        and math.isfinite(_finite_or_zero(row.get("fault_detection_delay", 0.0)))
    ]
    return selected


def generate_fault_figures(
    metrics_csv: Path,
    pairs_csv: Path,
    output_dir: Path,
    candidate: str = "guarded",
) -> list[Path]:
    metrics = read_numeric_csv(metrics_csv)
    pairs = read_numeric_csv(pairs_csv)
    detection_rows = _candidate_detection_rows(metrics, candidate)
    output_dir.mkdir(parents=True, exist_ok=True)

    outputs = [
        output_dir / "fault_delta_post_q_violation.svg",
        output_dir / "fault_delta_post_h_rms.svg",
        output_dir / "fault_candidate_detection_delay.svg",
    ]
    bar_chart(
        outputs[0],
        "Fault Sweep: Post-Fault q Violation Difference",
        "candidate - baseline",
        _fault_labels(pairs),
        _pair_values(pairs, "delta_post_fault_q_violation_integral"),
    )
    bar_chart(
        outputs[1],
        "Fault Sweep: Post-Fault Altitude RMS Difference",
        "candidate - baseline",
        _fault_labels(pairs),
        _pair_values(pairs, "delta_post_fault_h_rms"),
        color="#258a45",
    )
    bar_chart(
        outputs[2],
        f"Fault Detection Delay: {candidate}",
        "delay (s)",
        _fault_labels(detection_rows),
        _pair_values(detection_rows, "fault_detection_delay"),
        color="#8a4fbf",
    )
    return outputs


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--metrics-csv", type=Path, required=True)
    parser.add_argument("--pairs-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--candidate", default="guarded")
    args = parser.parse_args()

    for output in generate_fault_figures(args.metrics_csv, args.pairs_csv, args.output_dir, args.candidate):
        print(output)


if __name__ == "__main__":
    main()
