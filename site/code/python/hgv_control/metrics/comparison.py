"""Paired controller comparison utilities."""

from __future__ import annotations

from statistics import mean


Row = dict[str, float | int | str | bool]


def paired_deltas(rows: list[Row], baseline: str = "baseline", candidate: str = "guarded") -> list[Row]:
    by_case: dict[tuple[int, int], dict[str, Row]] = {}
    for row in rows:
        key = (int(row["case_id"]), int(row["seed"]))
        by_case.setdefault(key, {})[str(row["controller"])] = row

    pairs: list[Row] = []
    for (case_id, seed), controllers in sorted(by_case.items()):
        if baseline not in controllers or candidate not in controllers:
            continue
        base = controllers[baseline]
        cand = controllers[candidate]
        delta_q_violation = float(cand["q_violation_integral"]) - float(base["q_violation_integral"])
        delta_q_max = float(cand["q_max"]) - float(base["q_max"])
        delta_h_rms = float(cand["h_rms"]) - float(base["h_rms"])
        delta_v_rms = float(cand["v_rms"]) - float(base["v_rms"])
        pair: Row = {
            "case_id": case_id,
            "seed": seed,
            "baseline": baseline,
            "candidate": candidate,
            "baseline_pass": bool(base["pass"]),
            "candidate_pass": bool(cand["pass"]),
            "delta_h_rms": delta_h_rms,
            "delta_v_rms": delta_v_rms,
            "delta_q_max": delta_q_max,
            "delta_q_violation_integral": delta_q_violation,
            "safety_improved": delta_q_violation < 0.0 and delta_q_max < 0.0,
            "candidate_tracks_within_limits": bool(cand["pass"]) or (
                float(cand["h_rms"]) <= 900.0 and float(cand["v_rms"]) <= 450.0
            ),
        }
        pairs.append(pair)
    return pairs


def summarize_pairs(pairs: list[Row]) -> Row:
    if not pairs:
        return {
            "pairs": 0,
            "candidate_pass_rate": 0.0,
            "baseline_pass_rate": 0.0,
            "safety_improved_rate": 0.0,
            "mean_delta_q_violation_integral": 0.0,
            "mean_delta_q_max": 0.0,
            "mean_delta_h_rms": 0.0,
            "mean_delta_v_rms": 0.0,
        }

    return {
        "pairs": len(pairs),
        "candidate_pass_rate": mean(1.0 if pair["candidate_pass"] else 0.0 for pair in pairs),
        "baseline_pass_rate": mean(1.0 if pair["baseline_pass"] else 0.0 for pair in pairs),
        "safety_improved_rate": mean(1.0 if pair["safety_improved"] else 0.0 for pair in pairs),
        "mean_delta_q_violation_integral": mean(float(pair["delta_q_violation_integral"]) for pair in pairs),
        "mean_delta_q_max": mean(float(pair["delta_q_max"]) for pair in pairs),
        "mean_delta_h_rms": mean(float(pair["delta_h_rms"]) for pair in pairs),
        "mean_delta_v_rms": mean(float(pair["delta_v_rms"]) for pair in pairs),
    }

