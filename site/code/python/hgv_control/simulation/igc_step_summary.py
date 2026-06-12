"""Summarize the teaching IGC expansion steps.

This script aggregates metrics from the current Step 1-5 examples and writes a
small evidence table. It intentionally includes unresolved six-DOF gaps so the
report cannot be mistaken for complete IGC validation.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Any

from hgv_control.simulation import (
    aero_table_demo,
    attitude_inner_loop,
    control_allocation,
    coupled_six_dof_skeleton,
    guidance_3d,
    guidance_attitude_interface,
    six_dof_rigid_body_demo,
)


SUMMARY_FIELDS = ["step", "module", "metric", "value", "passed", "evidence_scope"]


def _format_value(value: Any) -> str:
    if isinstance(value, bool):
        return "True" if value else "False"
    if isinstance(value, float):
        return f"{value:.12g}"
    return str(value)


def _row(
    step: str,
    module: str,
    metric: str,
    value: Any,
    passed: bool | None,
    evidence_scope: str,
) -> dict[str, str]:
    return {
        "step": step,
        "module": module,
        "metric": metric,
        "value": _format_value(value),
        "passed": "" if passed is None else ("True" if passed else "False"),
        "evidence_scope": evidence_scope,
    }


def _add_metric_rows(
    rows: list[dict[str, str]],
    step: str,
    module: str,
    metrics: dict[str, float | bool],
    names: list[str],
    evidence_scope: str,
) -> None:
    module_pass = bool(metrics.get("pass", False))
    for name in names:
        rows.append(_row(step, module, name, metrics[name], module_pass, evidence_scope))


def build_summary() -> list[dict[str, str]]:
    guidance_trace, guidance_metrics = guidance_3d.run()
    attitude_trace, attitude_metrics = attitude_inner_loop.run()
    allocation_trace, allocation_metrics = control_allocation.run()
    interface_trace, interface_metrics = guidance_attitude_interface.run()
    coupled_trace, coupled_metrics = coupled_six_dof_skeleton.run()
    aero_trace, aero_metrics = aero_table_demo.run()
    rigid_body_trace, rigid_body_metrics = six_dof_rigid_body_demo.run()
    rows: list[dict[str, str]] = []

    _add_metric_rows(
        rows,
        "Step 1",
        "guidance_3d",
        guidance_metrics,
        [
            "terminal_range_m",
            "q_violation_integral",
            "load_factor_violation_integral",
            "terminal_pass",
            "path_pass",
            "pass",
        ],
        "teaching 3-D point-mass guidance only",
    )
    _add_metric_rows(
        rows,
        "Step 2",
        "attitude_inner_loop",
        attitude_metrics,
        [
            "attitude_rms_rad",
            "rate_max_rad_s",
            "moment_saturation_fraction",
            "tracking_pass",
            "moment_pass",
            "moment_rate_pass",
            "pass",
        ],
        "standalone teaching attitude inner loop only",
    )
    _add_metric_rows(
        rows,
        "Step 3",
        "control_allocation",
        allocation_metrics,
        [
            "allocation_residual_rms_nm",
            "allocation_residual_max_nm",
            "deflection_max_rad",
            "deflection_rate_max_rad_s",
            "residual_pass",
            "deflection_pass",
            "rate_pass",
            "pass",
        ],
        "standalone damped-pseudoinverse teaching allocator only",
    )
    _add_metric_rows(
        rows,
        "Step 4",
        "guidance_attitude_interface",
        interface_metrics,
        [
            "guidance_gamma_error_rms_rad",
            "guidance_heading_error_rms_rad",
            "attitude_interface_rms_rad",
            "allocation_residual_rms_nm",
            "deflection_max_rad",
            "command_pass",
            "attitude_pass",
            "allocation_pass",
            "path_pass",
            "pass",
        ],
        "guidance-to-attitude command compatibility, not closed-loop six-DOF",
    )
    _add_metric_rows(
        rows,
        "Step 5",
        "coupled_six_dof_skeleton",
        coupled_metrics,
        [
            "terminal_range_m",
            "q_violation_integral",
            "load_factor_violation_integral",
            "attitude_rms_rad",
            "gamma_rate_error_rms_rad_s",
            "heading_rate_error_rms_rad_s",
            "allocation_residual_rms_nm",
            "aero_moment_error_rms_nm",
            "aero_force_projection_rms_n",
            "dcm_force_delta_rms_n",
            "dcm_force_delta_max_n",
            "quaternion_norm_error_max",
            "dcm_orthogonality_error_max",
            "alpha_max_rad",
            "beta_max_rad",
            "terminal_pass",
            "path_pass",
            "attitude_pass",
            "allocation_pass",
            "aero_feedback_pass",
            "kinematics_pass",
            "force_transform_pass",
            "pass",
        ],
        "teaching coupled guidance-attitude-allocation-aerotable skeleton with DCM force-transform and quaternion/DCM health checks, not high-fidelity six-DOF",
    )
    _add_metric_rows(
        rows,
        "Support",
        "aero_table_demo",
        aero_metrics,
        [
            "clift_alpha_slope_per_rad",
            "drag_min",
            "lift_to_drag_min",
            "moment_max_nm",
            "pitch_body_flap_delta",
            "side_force_beta_delta",
            "yaw_rudder_delta",
            "lift_slope_pass",
            "drag_pass",
            "pitch_control_pass",
            "sideslip_pass",
            "rudder_pass",
            "pass",
        ],
        "teaching aerodynamic table interpolation, not integrated six-DOF aerodynamics",
    )
    _add_metric_rows(
        rows,
        "Support",
        "six_dof_rigid_body_demo",
        rigid_body_metrics,
        [
            "duration_s",
            "east_final_m",
            "north_final_m",
            "altitude_final_m",
            "altitude_change_m",
            "speed_change_m_s",
            "altitude_min_m",
            "body_rate_max_rad_s",
            "accel_rms_m_s2",
            "quaternion_norm_error_max",
            "dcm_orthogonality_error_max",
            "finite_pass",
            "altitude_pass",
            "kinematics_pass",
            "rate_pass",
            "pass",
        ],
        "standalone teaching six-DOF rigid-body propagation, not closed-loop IGC",
    )

    rows.extend(
        [
            _row(
                "Step 5 gap",
                "six_dof_closed_loop",
                "true_aero_moment_feedback_to_translation",
                "teaching_only",
                False,
                "current feedback uses synthetic tables, teaching ENU force projection, and a DCM force-transform check; requires validated aerodynamic database and full rigid-body force propagation",
            ),
            _row(
                "Step 5 gap",
                "six_dof_closed_loop",
                "closed_loop_attack_sideslip_aerotable",
                "teaching_only",
                False,
                "current alpha/beta table is synthetic; requires validated alpha/beta coefficient tables over the flight envelope",
            ),
            _row(
                "Step 5 gap",
                "six_dof_closed_loop",
                "quaternion_or_dcm_six_dof_rigid_body",
                "teaching_rigid_body_propagation_only",
                False,
                "current code has standalone teaching rigid-body propagation; still requires coupling validated aerodynamics, guidance, allocation, and closed-loop terminal metrics",
            ),
            _row(
                "Step 5 gap",
                "six_dof_closed_loop",
                "high_fidelity_closed_loop_terminal_metrics",
                "missing",
                False,
                "requires terminal metrics from a coupled aerodynamic six-DOF plant",
            ),
            _row(
                "Step 5 gap",
                "six_dof_closed_loop",
                "formal_igc_claim",
                "not_supported",
                False,
                "current evidence is a teaching-level aero-coupled skeleton, not formal IGC validation",
            ),
        ]
    )

    # Keep traces referenced so a future extension can write them without changing
    # the run order. The current summary only writes aggregate evidence.
    _ = (guidance_trace, attitude_trace, allocation_trace, interface_trace, coupled_trace, aero_trace, rigid_body_trace)
    return rows


def write_summary_csv(rows: list[dict[str, str]], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=SUMMARY_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def write_gap_markdown(rows: list[dict[str, str]], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    gap_rows = [row for row in rows if row["step"] == "Step 5 gap"]
    module_rows = [row for row in rows if row["step"] != "Step 5 gap" and row["metric"] == "pass"]
    module_titles = {
        "guidance_3d": "三维质点制导",
        "attitude_inner_loop": "姿态内环",
        "control_allocation": "控制分配",
        "guidance_attitude_interface": "制导到姿态接口",
        "coupled_six_dof_skeleton": "教学级耦合六自由度骨架",
        "aero_table_demo": "教学级气动表插值",
        "six_dof_rigid_body_demo": "教学级六自由度刚体传播",
    }
    pass_labels = {
        "True": "通过",
        "False": "未通过",
    }
    gap_titles = {
        "true_aero_moment_feedback_to_translation": "真实气动力矩反馈到平动",
        "closed_loop_attack_sideslip_aerotable": "闭环攻角和侧滑气动表",
        "quaternion_or_dcm_six_dof_rigid_body": "四元数或 DCM 六自由度刚体链",
        "high_fidelity_closed_loop_terminal_metrics": "高保真闭环终端指标",
        "formal_igc_claim": "正式 IGC 结论",
    }
    lines = [
        "# IGC 教学扩展指标汇总",
        "",
        "本文件由 python -m hgv_control.simulation.igc_step_summary 生成，用于汇总第 15 章第 1 至第 5 步的教学级证据，并列出尚未完成的真实六自由度闭环缺口。",
        "",
        "它不能作为完整的 IGC 验证报告。现有书稿已经具备一个教学级的制导、姿态、控制分配与气动表耦合骨架：分配后的舵面进入合成气动表，生成教学级气动力和力矩；通过 ENU 速度坐标基投影影响姿态以及三维质点的航迹角、航向角速率；另有独立的教学级气动表插值入口，用来训练 Mach、攻角插值、侧滑和舵面效能。该骨架还记录归一化四元数、DCM 正交性健康量，以及风轴载荷经机体系和 body-to-ENU DCM 后的教学力转换差值；并提供一个独立的教学六自由度刚体传播入口，用机体系力和力矩推进位置、速度、四元数和体轴角速度。但这些内容仍未组成带验证气动数据库和终端指标的闭环 IGC。",
        "",
        "## 教学模块通过情况",
        "",
        "| 模块 | 组合通过 | 证据边界 |",
        "| --- | --- | --- |",
    ]
    for row in module_rows:
        lines.append(f"| {module_titles.get(row['module'], row['module'])} | {pass_labels.get(row['value'], row['value'])} | {row['evidence_scope']} |")
    lines.extend(
        [
            "",
            "## 仍未闭合的 IGC 证据",
            "",
            "| 缺口 | 当前状态 | 需要的证据 |",
            "| --- | --- | --- |",
        ]
    )
    for row in gap_rows:
        lines.append(f"| {gap_titles.get(row['metric'], row['metric'])} | {pass_labels.get(row['value'], row['value'])} | {row['evidence_scope']} |")
    lines.append("")
    output.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--metrics-output", type=Path, default=Path("results/igc_step_summary.csv"))
    parser.add_argument("--gaps-output", type=Path, default=Path("results/igc_gap_audit.md"))
    args = parser.parse_args()

    rows = build_summary()
    write_summary_csv(rows, args.metrics_output)
    write_gap_markdown(rows, args.gaps_output)
    failed = [row for row in rows if row["passed"] == "False"]
    module_passes = [row for row in rows if row["metric"] == "pass" and row["passed"] == "True"]
    print(f"summary_rows: {len(rows)}")
    print(f"module_pass_rows: {len(module_passes)}")
    print(f"explicit_gap_or_fail_rows: {len(failed)}")
    print(f"metrics_output: {args.metrics_output}")
    print(f"gaps_output: {args.gaps_output}")


if __name__ == "__main__":
    main()
