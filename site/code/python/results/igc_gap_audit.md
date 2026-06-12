# IGC 教学扩展指标汇总

本文件由 python -m hgv_control.simulation.igc_step_summary 生成，用于汇总第 15 章第 1 至第 5 步的教学级证据，并列出尚未完成的真实六自由度闭环缺口。

它不能作为完整的 IGC 验证报告。现有书稿已经具备一个教学级的制导、姿态、控制分配与气动表耦合骨架：分配后的舵面进入合成气动表，生成教学级气动力和力矩；通过 ENU 速度坐标基投影影响姿态以及三维质点的航迹角、航向角速率；另有独立的教学级气动表插值入口，用来训练 Mach、攻角插值、侧滑和舵面效能。该骨架还记录归一化四元数、DCM 正交性健康量，以及风轴载荷经机体系和 body-to-ENU DCM 后的教学力转换差值；并提供一个独立的教学六自由度刚体传播入口，用机体系力和力矩推进位置、速度、四元数和体轴角速度。但这些内容仍未组成带验证气动数据库和终端指标的闭环 IGC。

## 教学模块通过情况

| 模块 | 组合通过 | 证据边界 |
| --- | --- | --- |
| 三维质点制导 | 通过 | teaching 3-D point-mass guidance only |
| 姿态内环 | 通过 | standalone teaching attitude inner loop only |
| 控制分配 | 通过 | standalone damped-pseudoinverse teaching allocator only |
| 制导到姿态接口 | 通过 | guidance-to-attitude command compatibility, not closed-loop six-DOF |
| 教学级耦合六自由度骨架 | 通过 | teaching coupled guidance-attitude-allocation-aerotable skeleton with DCM force-transform and quaternion/DCM health checks, not high-fidelity six-DOF |
| 教学级气动表插值 | 通过 | teaching aerodynamic table interpolation, not integrated six-DOF aerodynamics |
| 教学级六自由度刚体传播 | 通过 | standalone teaching six-DOF rigid-body propagation, not closed-loop IGC |

## 仍未闭合的 IGC 证据

| 缺口 | 当前状态 | 需要的证据 |
| --- | --- | --- |
| 真实气动力矩反馈到平动 | teaching_only | current feedback uses synthetic tables, teaching ENU force projection, and a DCM force-transform check; requires validated aerodynamic database and full rigid-body force propagation |
| 闭环攻角和侧滑气动表 | teaching_only | current alpha/beta table is synthetic; requires validated alpha/beta coefficient tables over the flight envelope |
| 四元数或 DCM 六自由度刚体链 | teaching_rigid_body_propagation_only | current code has standalone teaching rigid-body propagation; still requires coupling validated aerodynamics, guidance, allocation, and closed-loop terminal metrics |
| 高保真闭环终端指标 | missing | requires terminal metrics from a coupled aerodynamic six-DOF plant |
| 正式 IGC 结论 | not_supported | current evidence is a teaching-level aero-coupled skeleton, not formal IGC validation |
