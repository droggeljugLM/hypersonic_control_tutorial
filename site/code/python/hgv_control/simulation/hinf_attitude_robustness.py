"""H-infinity style robust attitude-control teaching case.

This module supports case C03 in Volume 5. It uses a single-axis pitch
attitude model and frequency-grid metrics to teach mixed-sensitivity ideas.
It is not a replacement for full-order hinfsyn, mu analysis, or nonlinear
six-DOF validation.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path

from hgv_control.models.attitude import AttitudeParams
from hgv_control.models.longitudinal import clamp


BASELINE_KP = 95_000.0
BASELINE_KD = 15_500.0
MIXED_GAMMA_TARGET = 180.0
ROBUST_STABILITY_TARGET = 0.90
ATTITUDE_RMS_TARGET = math.radians(2.9)
ATTITUDE_PEAK_TARGET = math.radians(6.5)
RATE_TARGET = math.radians(7.0)
SATURATION_FRACTION_TARGET = 0.03


@dataclass(frozen=True)
class PitchPlant:
    name: str
    inertia_y: float
    damping_q: float
    actuator_tau: float
    description: str


def make_uncertain_plants(params: AttitudeParams | None = None) -> list[PitchPlant]:
    params = params or AttitudeParams()
    return [
        PitchPlant("nominal", params.iy, params.damping_q, params.actuator_time_constant, "nominal pitch axis"),
        PitchPlant("high_inertia", 1.20 * params.iy, params.damping_q, params.actuator_time_constant, "+20% pitch inertia"),
        PitchPlant("low_damping", params.iy, 0.50 * params.damping_q, params.actuator_time_constant, "-50% pitch damping"),
        PitchPlant("slow_actuator", params.iy, params.damping_q, 1.75 * params.actuator_time_constant, "slower moment actuator"),
        PitchPlant(
            "combined_bad",
            1.20 * params.iy,
            0.50 * params.damping_q,
            1.75 * params.actuator_time_constant,
            "high inertia, low damping, slow actuator",
        ),
    ]


def logspace(low: float, high: float, count: int) -> list[float]:
    if count < 2:
        return [low]
    log_low = math.log10(low)
    log_high = math.log10(high)
    return [10.0 ** (log_low + index * (log_high - log_low) / (count - 1)) for index in range(count)]


def weights(omega: float) -> tuple[float, float, float, float]:
    """Return magnitudes of WS, WT, WU, and multiplicative uncertainty weight.

    WS is large at low frequency to penalize attitude error. WT and WU prevent
    high-bandwidth/noisy control from looking artificially good.
    """

    sensitivity_weight = math.hypot(omega / 2.0, 0.9) / math.hypot(omega, 0.9 * 0.02)
    complementary_weight = math.hypot(omega / 8.0, 0.04) / math.hypot(omega / (1.45 * 8.0), 1.0)
    control_weight = 1.0 / 6_200.0
    uncertainty_weight = 0.08 + 0.28 * (omega / 7.0) / math.hypot(1.0, omega / 7.0)
    return sensitivity_weight, complementary_weight, control_weight, uncertainty_weight


def frequency_metrics(
    kp: float,
    kd: float,
    plant: PitchPlant,
    frequencies: list[float] | None = None,
) -> dict[str, float | bool]:
    frequencies = frequencies or logspace(0.05, 100.0, 181)
    sensitivity_peak = 0.0
    complementary_peak = 0.0
    control_peak = 0.0
    mixed_peak = 0.0
    mixed_peak_frequency = 0.0
    robust_stability_peak = 0.0

    for omega in frequencies:
        s = complex(0.0, omega)
        plant_tf = 1.0 / ((plant.actuator_tau * s + 1.0) * (plant.inertia_y * s * s + plant.damping_q * s))
        controller_tf = kp + kd * s
        loop_tf = plant_tf * controller_tf
        sensitivity = 1.0 / (1.0 + loop_tf)
        complementary = loop_tf / (1.0 + loop_tf)
        control_sensitivity = controller_tf * sensitivity
        ws_mag, wt_mag, wu_mag, uncertainty_mag = weights(omega)

        sensitivity_peak = max(sensitivity_peak, abs(sensitivity))
        complementary_peak = max(complementary_peak, abs(complementary))
        control_peak = max(control_peak, abs(control_sensitivity))
        robust_stability_peak = max(robust_stability_peak, uncertainty_mag * abs(complementary))
        mixed_value = math.sqrt(
            (ws_mag * abs(sensitivity)) ** 2
            + (wt_mag * abs(complementary)) ** 2
            + (wu_mag * abs(control_sensitivity)) ** 2
        )
        if mixed_value > mixed_peak:
            mixed_peak = mixed_value
            mixed_peak_frequency = omega

    return {
        "sensitivity_peak": sensitivity_peak,
        "complementary_peak": complementary_peak,
        "control_peak_nm_per_rad": control_peak,
        "mixed_sensitivity_peak": mixed_peak,
        "mixed_peak_frequency_rad_s": mixed_peak_frequency,
        "robust_stability_peak": robust_stability_peak,
        "frequency_pass": mixed_peak <= MIXED_GAMMA_TARGET,
        "robust_stability_pass": robust_stability_peak <= ROBUST_STABILITY_TARGET,
    }


def pitch_reference(time_s: float) -> float:
    if time_s < 1.0:
        return 0.0
    if time_s < 8.0:
        return math.radians(3.0)
    return math.radians(-2.0)


def disturbance_moment(time_s: float) -> float:
    if 5.0 <= time_s <= 5.8:
        return 850.0
    if 11.0 <= time_s <= 11.6:
        return -600.0
    return 0.0


def simulate_pitch(
    kp: float,
    kd: float,
    plant: PitchPlant,
    params: AttitudeParams | None = None,
    duration: float = 16.0,
    dt: float = 0.02,
) -> dict[str, float | bool]:
    params = params or AttitudeParams()
    theta = math.radians(-1.0)
    q_rate = 0.0
    moment = 0.0
    theta_errors: list[float] = []
    rate_values: list[float] = []
    moment_values: list[float] = []
    command_values: list[float] = []
    saturated_steps = 0
    steps = int(duration / dt)

    for step in range(steps + 1):
        time_s = step * dt
        theta_ref = pitch_reference(time_s)
        error = theta_ref - theta
        command = clamp(kp * error - kd * q_rate, -params.moment_limit, params.moment_limit)
        if abs(command) >= params.moment_limit:
            saturated_steps += 1
        theta_errors.append(error)
        rate_values.append(abs(q_rate))
        moment_values.append(abs(moment))
        command_values.append(abs(command))
        disturbance = disturbance_moment(time_s)

        def derivatives(state: tuple[float, float, float]) -> tuple[float, float, float]:
            local_theta, local_rate, local_moment = state
            local_error = theta_ref - local_theta
            local_command = clamp(kp * local_error - kd * local_rate, -params.moment_limit, params.moment_limit)
            desired_moment_rate = (local_command - local_moment) / max(plant.actuator_tau, 1e-6)
            moment_rate = clamp(desired_moment_rate, -params.moment_rate_limit, params.moment_rate_limit)
            theta_dot = local_rate
            rate_dot = (local_moment + disturbance - plant.damping_q * local_rate) / plant.inertia_y
            return theta_dot, rate_dot, moment_rate

        state = (theta, q_rate, moment)
        k1 = derivatives(state)
        k2 = derivatives(tuple(state[index] + 0.5 * dt * k1[index] for index in range(3)))
        k3 = derivatives(tuple(state[index] + 0.5 * dt * k2[index] for index in range(3)))
        k4 = derivatives(tuple(state[index] + dt * k3[index] for index in range(3)))
        theta += dt * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]) / 6.0
        q_rate += dt * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]) / 6.0
        moment += dt * (k1[2] + 2.0 * k2[2] + 2.0 * k3[2] + k4[2]) / 6.0
        moment = clamp(moment, -params.moment_limit, params.moment_limit)

    attitude_rms = math.sqrt(sum(error * error for error in theta_errors) / len(theta_errors))
    attitude_peak = max(abs(error) for error in theta_errors)
    rate_max = max(rate_values)
    moment_peak = max(moment_values)
    command_peak = max(command_values)
    saturation_fraction = saturated_steps / len(theta_errors)
    time_pass = (
        attitude_rms <= ATTITUDE_RMS_TARGET
        and attitude_peak <= ATTITUDE_PEAK_TARGET
        and rate_max <= RATE_TARGET
    )
    input_pass = saturation_fraction <= SATURATION_FRACTION_TARGET and moment_peak <= params.moment_limit + 1e-9
    return {
        "attitude_rms_rad": attitude_rms,
        "attitude_peak_rad": attitude_peak,
        "rate_max_rad_s": rate_max,
        "moment_peak_nm": moment_peak,
        "moment_cmd_peak_nm": command_peak,
        "moment_saturation_fraction": saturation_fraction,
        "time_pass": time_pass,
        "input_pass": input_pass,
    }


def evaluate_controller(
    controller_name: str,
    kp: float,
    kd: float,
    plants: list[PitchPlant],
    params: AttitudeParams | None = None,
) -> list[dict[str, float | str | bool]]:
    rows: list[dict[str, float | str | bool]] = []
    for plant in plants:
        freq = frequency_metrics(kp, kd, plant)
        time = simulate_pitch(kp, kd, plant, params=params)
        row: dict[str, float | str | bool] = {
            "controller": controller_name,
            "plant": plant.name,
            "plant_description": plant.description,
            "kp": kp,
            "kd": kd,
            "inertia_y": plant.inertia_y,
            "damping_q": plant.damping_q,
            "actuator_tau": plant.actuator_tau,
        }
        row.update(freq)
        row.update(time)
        row["case_pass"] = (
            bool(row["frequency_pass"])
            and bool(row["robust_stability_pass"])
            and bool(row["time_pass"])
            and bool(row["input_pass"])
        )
        rows.append(row)
    return rows


def design_weighted_pd(plants: list[PitchPlant]) -> tuple[float, float]:
    best: tuple[float, float, float] | None = None
    for kp in range(25_000, 90_001, 5_000):
        for kd in range(6_000, 24_001, 2_000):
            rows = evaluate_controller("candidate", float(kp), float(kd), plants)
            worst_mixed = max(float(row["mixed_sensitivity_peak"]) for row in rows)
            worst_robust = max(float(row["robust_stability_peak"]) for row in rows)
            worst_rms = max(float(row["attitude_rms_rad"]) for row in rows)
            worst_saturation = max(float(row["moment_saturation_fraction"]) for row in rows)
            if (
                worst_robust <= ROBUST_STABILITY_TARGET
                and worst_rms <= math.radians(2.6)
                and worst_saturation <= 0.02
            ):
                if best is None or worst_mixed < best[0]:
                    best = (worst_mixed, float(kp), float(kd))
    if best is None:
        raise RuntimeError("no weighted PD candidate satisfied the teaching filters")
    return best[1], best[2]


def summarize_rows(rows: list[dict[str, float | str | bool]]) -> dict[str, float | bool]:
    baseline_rows = [row for row in rows if row["controller"] == "baseline_pd"]
    robust_rows = [row for row in rows if row["controller"] == "hinf_weighted_pd"]

    def worst(name: str, selected_rows: list[dict[str, float | str | bool]]) -> float:
        return max(float(row[name]) for row in selected_rows)

    return {
        "baseline_worst_mixed_sensitivity_peak": worst("mixed_sensitivity_peak", baseline_rows),
        "hinf_worst_mixed_sensitivity_peak": worst("mixed_sensitivity_peak", robust_rows),
        "baseline_worst_robust_stability_peak": worst("robust_stability_peak", baseline_rows),
        "hinf_worst_robust_stability_peak": worst("robust_stability_peak", robust_rows),
        "baseline_worst_attitude_rms_rad": worst("attitude_rms_rad", baseline_rows),
        "hinf_worst_attitude_rms_rad": worst("attitude_rms_rad", robust_rows),
        "baseline_worst_saturation_fraction": worst("moment_saturation_fraction", baseline_rows),
        "hinf_worst_saturation_fraction": worst("moment_saturation_fraction", robust_rows),
        "baseline_all_pass": all(bool(row["case_pass"]) for row in baseline_rows),
        "hinf_all_pass": all(bool(row["case_pass"]) for row in robust_rows),
    }


def run(params: AttitudeParams | None = None) -> tuple[list[dict[str, float | str | bool]], dict[str, float | bool]]:
    params = params or AttitudeParams()
    plants = make_uncertain_plants(params)
    robust_kp, robust_kd = design_weighted_pd(plants)
    rows = evaluate_controller("baseline_pd", BASELINE_KP, BASELINE_KD, plants, params=params)
    rows.extend(evaluate_controller("hinf_weighted_pd", robust_kp, robust_kd, plants, params=params))
    summary = summarize_rows(rows)
    summary["hinf_selected_kp"] = robust_kp
    summary["hinf_selected_kd"] = robust_kd
    return rows, summary


def write_csv(rows: list[dict[str, float | str | bool]], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    rows, summary = run()
    if args.output is not None:
        write_csv(rows, args.output)
    for key, value in summary.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
