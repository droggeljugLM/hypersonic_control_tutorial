# Python 教学示例

本目录提供一个最小可运行的高超声速纵向控制教学样例，用来支撑教程中的“动压约束下速度高度跟踪”代表性案例。

本目录的说明、指标和公式与正文保持同一套口径：数学表达直接用 $ 和 $$，普通术语不写成代码样式；如果某个符号在本说明中首次出现，应在就近位置把它讲清楚。

它不是高保真飞行器模型，不能用于总体设计或工程定型。它的目的只有三个：

- 展示状态、控制器、执行机构、动压约束和指标如何进入同一闭环；
- 对比 PID 风格标称控制器、LQR 标称控制器、NDI/反步/滑模教学控制器、动压参考治理和教学级屏障安全层；
- 提供可复现的最小测试入口。

本目录提供三维质点制导教学入口，用来支撑第 15 章从纵向模型走向三维 IGC 的第一步。它只包含位置、速度、航迹角和航向角，不是六自由度刚体模型。

本目录提供三轴姿态内环教学入口，用来支撑第 15 章的第二步。它只包含欧拉角、机体系角速度和三轴力矩执行层，不包含平动、攻角和侧滑角、气动表或控制分配。

本目录提供 $H_\infty$ 风格鲁棒姿态教学入口，用来支撑第五卷第 58 章 C03。它在单轴俯仰线性对象上构造不确定样本、混合灵敏度权重和频域最坏峰值指标，并用时域姿态阶跃和扰动力矩场景检查执行机构饱和。

本目录提供低阶刚柔耦合俯仰教学入口，用来支撑第二卷第 23 章。它把一个刚体俯仰轴、一个柔性模态、角速度测量污染、舵面力矩限幅限速和柔性导致的等效输入效能变化放在同一闭环中，并支持柔性频率、阻尼和测量污染参数扫描，用于训练带宽、传感器污染和失败样本的验证口径。

本目录提供 INDI 姿态教学入口，用来支撑第五卷第 58 章 C06。它在单轴俯仰对象上加入角加速度差分、低通滤波、执行机构实际力矩反馈、控制命令延迟和力矩增量限制，用于训练 INDI 的实现证据链。

本目录提供简化控制分配入口，用来支撑第 15 章的第三步。它把期望三轴力矩分配到四个教学执行机构，并计算分配残差、舵偏和舵速指标；它不是正式 QP 控制分配器。

本目录提供制导到姿态/分配接口入口，用来支撑第 15 章的第四步。它把三维质点制导命令转换为滚转、俯仰、偏航参考，并检查姿态跟踪和控制分配契约；它不是完整六自由度 IGC 闭环。

本目录提供 IGC 教学指标汇总入口，用来支撑第 15 章的第五步。它汇总第 1 步到第 5 步的教学级指标，并整理六自由度闭环仍需继续完善的环节。

本目录提供教学级耦合六自由度骨架入口，用来支撑第 15 章的第五步扩展。它把制导、姿态内环、控制分配和教学气动表放入同一闭环，使分配后的舵面生成气动力/力矩，气动力矩影响姿态，气动力和姿态再影响三维质点航迹角和航向角速率；它仍不是高保真六自由度模型。

本目录提供教学级气动表插值入口，用来支撑第 15 章的气动建模扩展。它演示 Mach/攻角二维表插值、侧滑角、舵面效能和气动力/力矩指标；同一教学气动接口也已接入耦合骨架。

本目录提供输入饱和与抗积分饱和教学入口，用来支撑第五卷第 58 章 C12。它在油门饱和场景中比较无抗积分饱和和抗积分饱和两种速度 PI 控制器，记录未限幅命令、饱和时间、积分状态峰值和释放后的速度恢复指标。

本目录提供吸气式巡航受限协同教学入口，用来支撑第四卷第 51 章和第五卷第 58 章 C04。它在低阶纵向对象上加入油门一阶动态、油门速率限制、进气道裕度代理、动压/热流代理和保护型参考收缩，用于训练速度高度、攻角和推进保护如何进入同一闭环。

本目录提供再入热流约束教学入口，用来支撑第四卷第 49 章和第五卷第 58 章 C14。它用低阶再入质点模型比较名义下滑命令和安全下滑命令，记录热流率、累计热载荷、动压、过载、走廊裕度和终端指标。

本目录提供滑翔能量管理教学入口，用来支撑第四卷第 50 章和第五卷第 58 章 C15。它用低阶滑翔质点模型比较名义横程修正和安全型能量管理，记录能量高度、终端能量、横程、滑翔走廊和倾侧输入指标。

本目录提供预测控制轨迹跟踪教学入口，用来支撑第五卷第 58 章 C18。它使用低维纵向预测模型和有限候选控制集，在动压约束下对比朴素跟踪与有限集合滚动优化，记录预测时域、求解时间、回退次数、经验可行性和跟踪/安全指标。

同级目录中的 MATLAB 对照样例提供标称方案和安全方案的最小实现，用于训练同一案例在不同仿真工具中的复现流程。

这些入口和第 58 章 C01 到 C20 案例的对应关系见 [案例映射页](../../volume5/case_library_code_mapping.md)。阅读这些入口之前，建议先查看该文件，明确每个入口支持哪些案例、证据边界在哪里，以及后续还能向哪里扩展。这样安排不是为了记住入口名字，而是为了先看清每个算例在全书中的位置。

## 运行方式

本目录的入口很多。为了让页面更像书，而不是命令清单，下面按用途分成四组说明。真正执行时，最常用的最小单场景入口仍然是 python -m hgv_control.simulation.run_case，只需在后面指定不同的控制器类型即可，例如标称方案、安全方案、LQR、NDI、反步、滑模和屏障层。

单项教学算例覆盖姿态内环、鲁棒姿态、刚柔耦合俯仰、INDI 姿态、控制分配、制导到姿态接口、IGC 汇总、教学级耦合六自由度骨架、六自由度刚体传播、气动表、输入饱和与抗积分饱和、吸气式巡航协同、再入热流约束、滑翔能量管理、预测控制轨迹跟踪、三维制导和故障注入等入口。它们分别对应姿态内环、H∞ 风格鲁棒姿态、刚柔耦合俯仰、INDI 姿态、控制分配、制导到姿态接口、IGC 汇总、教学级耦合六自由度骨架、六自由度刚体传播、气动表、输入饱和与抗积分饱和、吸气式巡航协同、再入热流约束、滑翔能量管理、预测控制轨迹跟踪、三维制导和故障注入等教学模块。把这两层对应关系读清楚，后面的复现、对照和选题才不会断线。

批量统计主要包括 Monte Carlo、控制器对比和故障扫描。出图入口主要包括姿态图、控制分配图、接口图、耦合六自由度图、刚体传播图、气动表图、制导图、故障图和综合图。它们不是附带材料，而是这本书把“结果如何进入证据链”说清楚的必要出口。

如果需要一次性设置环境，先导出 PYTHONDONTWRITEBYTECODE=1，再把 PYTHONPATH 指向 D:\Phd\code\codex\test\hypersonic_control_tutorial\code\python。之后直接运行上表中的模块入口即可。

单元测试框架仍然是最先检查的一步，用来确认教学算例和绘图脚本都能正常导入与运行。对这类书稿来说，能导入只是起点，后面的重点还在于能否把结果、图表和结论统一到同一口径里。


INDI 姿态入口输出角加速度估计误差、增量命令、执行机构延迟、饱和比例和扰动后恢复指标；在这个样例中，INDI 降低姿态 RMS、扰动后 RMS 和命令饱和比例，但不代表完整三轴 INDI 或控制分配闭环已经完成。控制分配入口用阻尼伪逆把期望三轴力矩分配到四个执行机构，输出残差、舵偏、舵速和饱和指标，用于第 15 章第 3 步；控制分配图表生成器从轨迹记录表生成力矩跟踪、执行机构偏角和分配残差图。制导到姿态接口把三维制导命令转换成姿态参考并检查分配残差，用于第 15 章第 4 步；接口图表生成器从轨迹记录表生成制导误差、姿态参考和分配残差图。IGC 汇总入口整理第 1 步到第 5 步的教学级指标，并说明闭环 IGC 仍需继续完善的证据方向，用于第 15 章第 5 步。教学级耦合六自由度骨架把制导、姿态、分配和气动表放进同一闭环，使分配力矩影响姿态、姿态影响三维质点航迹角和航向角速率，并记录 DCM 力转换和四元数、DCM 健康字段，用于第 15 章第 5 步；对应图表生成器会输出地面航迹、制导速率、姿态参考、分配残差、力投影、力转换和姿态运动学健康图。

教学六自由度刚体传播入口用机体系力和力矩推进 ENU 位置、速度、四元数和体轴角速度；对应图表生成器输出位置、速度、体轴角速度和姿态健康图。气动表插值入口输出 Mach 和攻角插值、侧滑和舵面效能导致的气动力、力矩指标；对应图表生成器输出气动系数、气动力和气动力矩图。输入饱和与抗积分饱和入口输出油门饱和时间、未限幅命令、速度积分状态峰值和释放后速度恢复指标；这里聚焦油门饱和，不代表完整多舵面抗积分饱和。吸气式巡航受限协同入口输出修正后速度和高度参考、进气道裕度代理、保护激活时间、油门峰值和速率、动压和热流代理指标；这里属于低阶纵向对象上的推进保护训练，不代表高保真进气道、燃烧室或喷管闭环。再入热流约束入口输出热流率、累计热载荷、动压、过载、走廊裕度、终端误差和组合通过判据；这里属于低阶质点再入模型，不代表真实气动热或倾侧角再入制导。滑翔能量管理入口输出能量高度、终端能量、能量走廊裕度、横程误差、动压、热流、过载、倾侧角和升阻比指标；这里属于低阶质点滑翔模型，不代表正式轨迹优化或大范围倾侧翻转制导。预测控制轨迹跟踪入口输出预测时域、动压约束、求解时间、回退次数、经验可行性和速度、高度跟踪指标；这里属于有限候选控制集滚动优化，不代表严格 QP、NMPC 或带终端集合证明的预测控制。三维制导入口输出终端距离、动压、过载和路径通过判据，用于第 15 章第 1 步；对应图表生成器输出地面航迹、高度剖面、动压和过载图。故障注入入口可注入高度、速度、攻角传感器偏置、升降舵效率损失、升降舵卡滞、推力损失和输入延迟，输出估计误差、残差检测和故障后安全、跟踪指标。故障扫描入口批量运行故障场景矩阵，输出完整故障结果表和候选控制器相对基线的成对差值结果表；对应图表生成器输出故障后安全差值、跟踪差值和检测延迟图。

单场景输出主要覆盖高度跟踪 RMS、速度跟踪 RMS、最大动压、最小动压裕度、动压违反积分、最大攻角、最大舵偏、最大舵速、油门跨度，以及跟踪、动压、攻角、舵偏、舵速和总通过标志。

## 代码结构

- hgv_control/
  - models/aero_table.py
  - models/attitude.py
  - models/control_allocation.py
  - models/flexible_pitch.py
  - models/frames.py
  - models/rigid_body_6dof.py
  - models/rigid_body_kinematics.py
  - models/point_mass_3d.py
  - models/longitudinal.py
  - controllers/backstepping.py
  - controllers/barrier.py
  - controllers/lqr.py
  - controllers/ndi.py
  - controllers/pid.py
  - controllers/safety_filter.py
  - controllers/sliding_mode.py
  - metrics/comparison.py
  - metrics/summary.py
  - plotting/make_attitude_figures.py
  - plotting/make_aero_table_figures.py
  - plotting/make_allocation_figures.py
  - plotting/make_coupled_six_dof_figures.py
  - plotting/make_fault_figures.py
  - plotting/make_guidance_figures.py
  - plotting/make_interface_figures.py
  - plotting/make_six_dof_rigid_body_figures.py
  - plotting/svg.py
  - plotting/make_figures.py
  - simulation/attitude_inner_loop.py
  - simulation/aero_table_demo.py
  - simulation/control_allocation.py
  - simulation/compare_controllers.py
  - simulation/coupled_six_dof_skeleton.py
  - simulation/airbreathing_cruise_coordination.py
  - simulation/fault_injection.py
  - simulation/fault_sweep.py
  - simulation/flexible_pitch_demo.py
  - simulation/glide_energy_management.py
  - simulation/guidance_3d.py
  - simulation/guidance_attitude_interface.py
  - simulation/hinf_attitude_robustness.py
  - simulation/igc_step_summary.py
  - simulation/indi_attitude_control.py
  - simulation/input_saturation_anti_windup.py
  - simulation/mpc_trajectory_tracking.py
  - simulation/reentry_heat_rate_control.py
  - simulation/six_dof_rigid_body_demo.py
  - simulation/run_case.py
  - simulation/monte_carlo.py
- figures/
  - aero_table_coefficients.svg
  - aero_table_forces.svg
  - aero_table_moments.svg
  - allocation_actuators.svg
  - allocation_moments.svg
  - allocation_residual.svg
  - coupled6dof_allocation_residual.svg
  - coupled6dof_aero_angles.svg
  - coupled6dof_aero_moments.svg
  - coupled6dof_attitude_refs.svg
  - coupled6dof_force_projection.svg
  - coupled6dof_kinematics_health.svg
  - coupled6dof_force_transform.svg
  - coupled6dof_ground_track.svg
  - coupled6dof_guidance_rates.svg
  - sixdof_rigidbody_body_rates.svg
  - sixdof_rigidbody_kinematics_health.svg
  - sixdof_rigidbody_position_speed.svg
  - altitude_tracking.svg
  - attitude_moments.svg
  - attitude_rates.svg
  - attitude_tracking.svg
  - velocity_tracking.svg
  - dynamic_pressure.svg
  - fault_candidate_detection_delay.svg
  - fault_delta_post_h_rms.svg
  - fault_delta_post_q_violation.svg
  - guidance3d_altitude_profile.svg
  - guidance3d_dynamic_pressure.svg
  - guidance3d_ground_track.svg
  - guidance3d_load_factor.svg
  - interface_allocation_residual.svg
  - interface_attitude_refs.svg
  - interface_guidance_errors.svg
  - paired_delta_q_violation.svg
- results/
  - aero_table_trace.csv
  - attitude_inner_loop_trace.csv
  - control_allocation_trace.csv
  - coupled_six_dof_skeleton_trace.csv
  - flexible_pitch_metrics.csv
  - flexible_pitch_sweep.csv
  - flexible_pitch_trace.csv
  - glide_energy_guard_trace.csv
  - glide_energy_metrics.csv
  - input_saturation_no_aw_trace.csv
  - input_saturation_aw_trace.csv
  - mpc_trajectory_metrics.csv
  - reentry_heat_guard_trace.csv
  - reentry_heat_rate_metrics.csv
  - backstepping_trace.csv
  - barrier_trace.csv
  - baseline_trace.csv
  - guarded_trace.csv
  - guidance3d_trace.csv
  - guidance_attitude_interface_trace.csv
  - hinf_attitude_metrics.csv
  - indi_attitude_metrics.csv
  - six_dof_rigid_body_trace.csv
  - igc_gap_audit.md
  - igc_step_summary.csv
  - lqr_trace.csv
  - ndi_trace.csv
  - smc_trace.csv
  - mc4_metrics.csv
  - mc4_pairs.csv
  - mc4_backstepping_metrics.csv
  - mc4_backstepping_pairs.csv
  - mc4_barrier_metrics.csv
  - mc4_barrier_pairs.csv
  - mc4_lqr_metrics.csv
  - mc4_lqr_pairs.csv
  - mc4_ndi_metrics.csv
  - mc4_ndi_pairs.csv
  - mc4_smc_metrics.csv
  - mc4_smc_pairs.csv
- tests/
  - test_aero_table.py
  - test_attitude_inner_loop.py
  - test_frames.py
  - test_rigid_body_6dof.py
  - test_rigid_body_kinematics.py
  - test_control_allocation.py
  - test_coupled_six_dof_skeleton.py
  - test_backstepping.py
  - test_barrier.py
  - test_comparison.py
  - test_dynamic_pressure.py
  - test_fault_injection.py
  - test_fault_sweep.py
  - test_flexible_pitch.py
  - test_glide_energy_management.py
  - test_guidance_3d.py
  - test_guidance_attitude_interface.py
  - test_hinf_attitude_robustness.py
  - test_igc_step_summary.py
  - test_indi_attitude_control.py
  - test_input_saturation_anti_windup.py
  - test_mpc_trajectory_tracking.py
  - test_reentry_heat_rate_control.py
  - test_lqr.py
  - test_ndi.py
  - test_sliding_mode.py
  - test_monte_carlo.py
  - test_plotting.py

## 三轴姿态内环入口

simulation/attitude_inner_loop.py 对应第 15 章“六自由度模型、三维制导与控制分配扩展”中的第 2 步：

- **欧拉角姿态状态**：三轴角速度、PD 风格力矩命令、力矩幅值和速率限制、姿态跟踪与执行机构指标

运行命令：python -m hgv_control.simulation.attitude_inner_loop --output hypersonic_control_tutorial\code\python\results\attitude_inner_loop_trace.csv；python -m hgv_control.plotting.make_attitude_figures --trace-csv hypersonic_control_tutorial\code\python\results\attitude_inner_loop_trace.csv --output-dir hypersonic_control_tutorial\code\python\figures

默认场景输出指标重点看姿态 RMS、三轴角速度峰值、力矩峰值、力矩速率峰值以及各类通过判据。这样可以同时判断姿态误差是否收敛、执行机构是否受限、以及闭环是否仍满足工程可行性。

姿态图表对应 attitude_tracking.svg、attitude_rates.svg 和 attitude_moments.svg，分别展示滚转、俯仰、偏航角和对应参考、机体系三轴角速度，以及三轴实际力矩和力矩上限。

它的作用是训练姿态内环如何把姿态误差转成三轴力矩命令，并检查力矩幅值、力矩速率和角速度。它没有平动方程、气动表、攻角/侧滑角或控制分配矩阵，因此不能写成“六自由度模型”或“完整 IGC”。

## $H_\infty$ 风格鲁棒姿态入口

simulation/hinf_attitude_robustness.py 对应第五卷第 58 章 C03，也支撑第三卷第 31 章：

- **单轴俯仰线性对象**：惯量、阻尼、执行机构不确定样本；混合灵敏度权重 $W_S$、$W_T$、$W_U$；频域最坏峰值和小增益式裕度；时域姿态阶跃、扰动力矩和饱和检查

运行方式：执行 python -m hgv_control.simulation.hinf_attitude_robustness --output hypersonic_control_tutorial\code\python\results\hinf_attitude_metrics.csv

默认场景输出指标重点看混合灵敏度峰值、鲁棒稳定裕度、姿态 RMS、姿态峰值、力矩饱和比例以及频域和时域通过判据。这样才能把“鲁棒性是否提高”和“代价是否可接受”放在同一张证据表里看。

该入口用于训练 $H_\infty$ 鲁棒控制的基本报告口径：必须说明不确定集合、权重、最坏频率增益、鲁棒稳定裕度和时域执行机构代价。这里采用单轴俯仰、频率网格和加权 PD 搜索，只能作为混合灵敏度概念的最小可运行案例；它不是完整 $H_\infty$ 综合控制器、结构奇异值分析或高保真六自由度鲁棒飞控验证。

## 刚柔耦合俯仰入口

simulation/flexible_pitch_demo.py 对应第二卷第 23 章：

- **刚体俯仰轴**：单个柔性模态 eta_dot；角速度测量污染 q_measured=q+c_f eta_dot；柔性引起的等效输入效能变化；姿态跟踪、模态能量、舵速和带宽比指标

运行命令：python -m hgv_control.simulation.flexible_pitch_demo --metrics-output hypersonic_control_tutorial\code\python\results\flexible_pitch_metrics.csv --trace-output hypersonic_control_tutorial\code\python\results\flexible_pitch_trace.csv --sweep-output hypersonic_control_tutorial\code\python\results\flexible_pitch_sweep.csv

默认场景输出指标重点看姿态误差、测量污染、模态幅值、力矩速率、等效输入效能、带宽比以及各类通过判据。这样可以把刚柔耦合的主要风险点一次讲清。

扫描输出额外记录灵活模态场景、频率与阻尼参数、传感器模态速率增益、效能模态增益以及最小效能这些信息，用来比较不同柔性条件下的敏感性差异。

该入口用于训练刚柔耦合报告口径：必须把姿态误差、柔性模态、测量污染、舵速、带宽、参数扫描和失败样本一起报告。这里采用单轴低阶教学模型，不包含有限元模态、真实气动弹性导数、热结构耦合或颤振边界验证。

## INDI 姿态控制入口

simulation/indi_attitude_control.py 对应第五卷第 58 章 C06，也支撑第三卷第 34 章：

- **单轴俯仰姿态对象**：角速度测量噪声、角加速度差分和低通滤波、实际力矩反馈和命令延迟、INDI 增量力矩命令、跟踪、扰动恢复和饱和指标

运行命令：python -m hgv_control.simulation.indi_attitude_control --output hypersonic_control_tutorial\code\python\results\indi_attitude_metrics.csv

默认场景输出指标重点看姿态 RMS、扰动后 RMS、角速度峰值、角加速度估计误差、命令增量、饱和比例、力矩速率以及各类通过判据。这样可以直接说明 INDI 的反馈链是否真正发挥了作用。

该入口用于说明 INDI 不是“不要模型”，而是把完整模型依赖转化为控制效能、角加速度反馈、滤波和执行机构反馈问题。这里仅覆盖单轴俯仰力矩通道，尚未接入三轴控制分配、舵面实际位置反馈、多执行机构约束或六自由度姿态/平动耦合。

## 简化控制分配入口

simulation/control_allocation.py 对应第 15 章“六自由度模型、三维制导与控制分配扩展”中的第 3 步：

- **期望三轴力矩**：控制效能矩阵、阻尼伪逆分配、舵偏幅值和速率限制、实际力矩和分配残差指标

运行命令：python -m hgv_control.simulation.control_allocation --output hypersonic_control_tutorial\code\python\results\control_allocation_trace.csv；python -m hgv_control.plotting.make_allocation_figures --trace-csv hypersonic_control_tutorial\code\python\results\control_allocation_trace.csv --output-dir hypersonic_control_tutorial\code\python\figures

默认场景输出指标重点看分配残差、舵偏峰值、舵偏速率峰值、饱和比例、速率受限比例以及通过判据。它们共同决定“期望力矩”能否在执行机构层真正实现。

控制分配图表包括：

- allocation_moments.svg：期望三轴力矩和实际分配力矩；
- allocation_actuators.svg：四个教学执行机构偏角和偏角上限；
- allocation_residual.svg：分配残差范数。

它的作用是训练控制分配的基本口径：即使姿态控制器给出期望力矩，也必须检查执行机构是否能实现、是否触及幅值/速率限制、残差是否可接受。这里采用阻尼伪逆加限幅限速，不是正式 QP 控制分配器，也还没有接入姿态内环闭环。

## 制导到姿态/分配接口入口

simulation/guidance_attitude_interface.py 对应第 15 章“六自由度模型、三维制导与控制分配扩展”中的第 4 步：

- **三维制导误差**：航迹角/航向角速率命令、滚转/俯仰/偏航参考、姿态力矩命令、控制分配残差和执行机构指标

运行命令：python -m hgv_control.simulation.guidance_attitude_interface --output hypersonic_control_tutorial\code\python\results\guidance_attitude_interface_trace.csv；python -m hgv_control.plotting.make_interface_figures --trace-csv hypersonic_control_tutorial\code\python\results\guidance_attitude_interface_trace.csv --output-dir hypersonic_control_tutorial\code\python\figures

默认场景输出指标重点看制导误差、姿态接口误差、分配残差、参考幅值、路径约束违反积分以及各层通过判据。这样可以同时回答“制导命令是否合理”和“姿态与分配是否能接住命令”。

接口图表包括：

- interface_guidance_errors.svg：航迹角误差和航向误差；
- interface_attitude_refs.svg：滚转、俯仰、偏航参考与姿态响应；
- interface_allocation_residual.svg：接口链路中的分配残差范数。

它的作用是训练 IGC 接口契约：制导层输出的命令必须能被姿态层和分配层接受。平动状态仍由三维质点制导直接推进，姿态和分配链路只做命令兼容性检查，因此不能写成完整 IGC 闭环。

## IGC 教学指标汇总入口

simulation/igc_step_summary.py 对应第 15 章“六自由度模型、三维制导与控制分配扩展”中的第 5 步：

- **第 1 步三维质点制导指标**：第 2 步姿态内环指标、第 3 步控制分配指标、第 4 步制导-姿态接口指标、第 5 步教学级耦合骨架指标、证据边界和六自由度闭环缺口

运行命令：python -m hgv_control.simulation.igc_step_summary --metrics-output hypersonic_control_tutorial\code\python\results\igc_step_summary.csv --gaps-output hypersonic_control_tutorial\code\python\results\igc_gap_audit.md

输出文件包括 igc_step_summary.csv 和 igc_gap_audit.md。前者逐项记录第 1 步到第 5 步的关键指标、通过状态和证据边界，后者列出仍缺的六自由度闭环证据。两者合起来，才构成 IGC 证据链的总览。

这个入口的作用不是给出“IGC 已经完整建立”的结论，而是训练读者把证据分层。formal_igc_claim 明确为 not_supported，因为气动表仍是合成教学表，闭环传播仍是三维质点和教学姿态骨架；虽然已有独立教学六自由度刚体传播入口，但还没有把验证气动数据库、制导、控制分配和刚体传播组成闭环 IGC。

## 教学级耦合六自由度骨架入口

simulation/coupled_six_dof_skeleton.py 对应第 15 章第 5 步中的“先闭合教学反馈链”：

- **三维制导**：姿态参考；姿态内环力矩命令；控制分配舵面；教学气动表气动力/力矩；姿态响应和三维质点速率；三维质点航迹角/航向角速率；四元数/DCM 健康字段

运行命令：python -m hgv_control.simulation.coupled_six_dof_skeleton --output hypersonic_control_tutorial\code\python\results\coupled_six_dof_skeleton_trace.csv；python -m hgv_control.plotting.make_coupled_six_dof_figures --trace-csv hypersonic_control_tutorial\code\python\results\coupled_six_dof_skeleton_trace.csv --output-dir hypersonic_control_tutorial\code\python\figures

默认场景输出指标重点看终端误差、路径约束、姿态误差、控制分配残差、气动反馈误差、四元数与 DCM 健康字段、攻角/侧滑角峰值、舵偏峰值以及各层通过判据。这样才能判断教学级耦合链路到底在哪一层还存在明显缺口。

配套图表对应 coupled6dof_ground_track.svg、coupled6dof_guidance_rates.svg、coupled6dof_attitude_refs.svg、coupled6dof_allocation_residual.svg、coupled6dof_aero_angles.svg、coupled6dof_aero_moments.svg、coupled6dof_force_projection.svg、coupled6dof_kinematics_health.svg 和 coupled6dof_force_transform.svg，分别展示教学耦合链路下的地面航迹、制导层速率命令与实现速率、姿态参考和实际姿态、控制分配残差、攻角和侧滑角、姿态力矩命令和教学气动力矩、气动力投影、四元数和 DCM 正交性误差，以及风轴载荷转换差值。

这个入口比第 4 步多了两条反馈：分配后的舵面进入教学气动表生成气动力和力矩，姿态和气动力又反过来影响三维质点航迹。它还记录教学级四元数、方向余弦矩阵健康字段和 DCM 力转换差值，但仍然是教学级六自由度骨架，气动表是合成教学表，四元数、DCM 和力转换只用于教学检查，因此不能写成完整六自由度 IGC 验证。

其中 models/frames.py 使用本教程的 ENU 坐标约定，把风轴升力、阻力和侧力投影到速度切向、航迹角法向和航向侧向，再由投影加速度计算教学级航迹角变化率和航向角变化率。这一步补上了“速度坐标基力方向”训练。

其中 models/rigid_body_kinematics.py 使用标量在前的四元数，把欧拉角和机体系角速度转换成 body-to-ENU 四元数与 DCM，并输出四元数模长误差、方向余弦矩阵正交性误差和机体轴向东、北、天分量。它还把升力、阻力和侧力转为机体系力分量，再经 DCM 得到东、北、天向力分量，并记录力变换差值。这一步用于训练姿态表示和力转换方向检查，不替代完整刚体姿态动力学或六自由度力传播。

## 教学级六自由度刚体传播入口

simulation/six_dof_rigid_body_demo.py 对应第 15 章中的独立刚体传播支撑模块：

- **机体系力与力矩**：机体系到 ENU 的方向余弦矩阵力变换；ENU 加速度；位置与速度传播；四元数传播；机体系角速度传播

运行命令：python -m hgv_control.simulation.six_dof_rigid_body_demo --output hypersonic_control_tutorial\code\python\results\six_dof_rigid_body_trace.csv；python -m hgv_control.plotting.make_six_dof_rigid_body_figures --trace-csv hypersonic_control_tutorial\code\python\results\six_dof_rigid_body_trace.csv --output-dir hypersonic_control_tutorial\code\python\figures

默认场景输出指标重点看高度变化、速度变化、最小高度、体轴角速度峰值、加速度 RMS，以及四元数和 DCM 健康字段。这样可以判断这条独立传播链是否数值上稳定、物理上合理。

配套图表对应 sixdof_rigidbody_position_speed.svg、sixdof_rigidbody_body_rates.svg 和 sixdof_rigidbody_kinematics_health.svg，分别展示 ENU 位置、高度和速度、机体系角速度，以及四元数范数和 DCM 正交性误差。

这个入口把六自由度状态传播落到代码，但它仍是独立教学例子：输入力和力矩来自预设剖面，不来自制导、控制分配和验证气动数据库。因此它只能支撑“六自由度刚体传播骨架可运行”，不能支撑“闭环 IGC 已验证”。

## 气动表插值入口

simulation/aero_table_demo.py 对应第 15 章“气动力和力矩的三维表达”中的教学代码支撑：

- **马赫数、攻角**：二维气动系数表插值、侧滑角和舵面效能修正、升力、阻力、侧力和三轴力矩、气动表符号和约束指标检查

运行命令：python -m hgv_control.simulation.aero_table_demo --output hypersonic_control_tutorial\code\python\results\aero_table_trace.csv；python -m hgv_control.plotting.make_aero_table_figures --trace-csv hypersonic_control_tutorial\code\python\results\aero_table_trace.csv --output-dir hypersonic_control_tutorial\code\python\figures

默认场景输出指标重点看升阻特性、最小阻力、升阻比、最大力矩、舵面效能变化以及各类通过判据。这样可以把气动表是否可用、符号是否一致、控制效能是否合理放在同一框架里判断。

配套图表对应 aero_table_coefficients.svg、aero_table_forces.svg 和 aero_table_moments.svg，分别展示升力、阻力、侧力和俯仰力矩系数，升力、阻力和侧力，以及滚转、俯仰和偏航力矩。

这个入口的作用是训练读者理解真实六自由度模型之前必须解决的气动数据接口：气动系数表不能只作为黑箱存在，必须明确插值变量、坐标约定、攻角和侧滑角定义、舵面正方向和力矩正方向。表格是合成教学表，不是风洞、CFD 或飞行试验数据；同一教学接口已接入耦合六自由度骨架，但仍不能替代高保真气动数据库。

## 输入饱和与抗积分饱和入口

simulation/input_saturation_anti_windup.py 对应第五卷第 58 章 C12：

- **不可持续速度参考**：油门饱和、速度 PI 积分项累积、参考释放后的恢复误差、抗积分饱和对比

运行命令：python -m hgv_control.simulation.input_saturation_anti_windup --controller no_anti_windup --output hypersonic_control_tutorial\code\python\results\input_saturation_no_aw_trace.csv；python -m hgv_control.simulation.input_saturation_anti_windup --controller anti_windup --output hypersonic_control_tutorial\code\python\results\input_saturation_aw_trace.csv

默认场景输出指标包括：

这一组指标重点看释放后的速度 RMS、释放后的速度超调、速度积分状态峰值、油门超限积分、饱和占比和饱和持续时间，再结合跟踪判据、输入约束判据、动压判据和综合通过判据一起判断。它们不是单独报数，而是用来说明抗积分饱和在饱和解除后是否真的把状态拉回到了可控范围。

该入口用于说明抗积分饱和的控制工程含义：限幅只能保证命令不超过物理范围，但不能自动阻止积分状态在饱和期间继续积累。这里只覆盖油门饱和和速度 PI 积分项，后续仍需扩展到舵面、多轴控制分配和更完整的抗积分饱和辅助系统。

## 再入热流约束入口

simulation/reentry_heat_rate_control.py 对应第四卷第 49 章和第五卷第 58 章 C14：

- **低阶再入质点模型**：基线下滑命令、热守护下滑命令、热流/热载荷/动压/过载/走廊指标、终端和路径组合判据

运行命令：python -m hgv_control.simulation.reentry_heat_rate_control --metrics-output hypersonic_control_tutorial\code\python\results\reentry_heat_rate_metrics.csv --trace-output hypersonic_control_tutorial\code\python\results\reentry_heat_guard_trace.csv

默认场景输出指标包括：

这一组指标需要一起读：终端横程、终端高度和终端速度误差说明任务有没有落点；最大热流率、热流超限积分和累计热载荷说明热防压力；最大动压、最大过载、走廊最小裕度和走廊超限积分说明路径是否仍在允许范围内；航迹角速率峰值和综合通过判据则把过程约束和终端约束合在一起判断。

该入口用于说明再入控制的评价不能只看终端误差。默认场景中基线终端指标可接受，但热流率和走廊约束失败；热守护下滑命令通过热流、动压、过载、走廊和终端组合判据。这里不包含倾侧角翻转、三维横程控制、真实热防模型或六自由度姿态执行层。

## 预测控制轨迹跟踪入口

simulation/mpc_trajectory_tracking.py 对应第五卷第 58 章 C18，也支撑第四卷第 44 章：

- **低维纵向预测模型**：有限候选 gamma 和油门命令、动压约束预测、滚动选择第一步控制、求解时间、回退和经验可行性记录

运行命令：python -m hgv_control.simulation.mpc_trajectory_tracking --output hypersonic_control_tutorial\code\python\results\mpc_trajectory_metrics.csv

默认场景输出指标包括：

这一组指标主要看预测时域、速度和高度 RMS、最大动压、最小动压裕度、动压违反积分、求解成功率、求解时间上界、求解时间 95 分位、回退次数、递归可行性失败次数和综合通过判据。它们共同说明预测控制教学案例是不是既能跑，又能在约束和计算代价上站得住。

该入口用于说明预测控制教学案例必须把预测时域、约束、求解时间、不可行样本和回退作为方法证据的一部分。这里用枚举有限控制集替代 QP/NMPC 求解器，没有终端集合和严格递归可行性证明；它适合训练指标口径，不适合写成工程应用闭环。

## 三维质点制导入口

simulation/guidance_3d.py 对应第 15 章“六自由度模型、三维制导与控制分配扩展”中的第 1 步：

- **三维质点模型**：终端目标、航迹角和航向角制导命令、动压和过载路径约束指标

运行命令：python -m hgv_control.simulation.guidance_3d --output hypersonic_control_tutorial\code\python\results\guidance3d_trace.csv；python -m hgv_control.plotting.make_guidance_figures --trace-csv hypersonic_control_tutorial\code\python\results\guidance3d_trace.csv --output-dir hypersonic_control_tutorial\code\python\figures

默认场景输出指标重点看终端横程误差、高度和速度误差、动压与过载峰值、路径违反积分以及终端/路径通过判据。这样可以直接判断三维制导是否同时满足任务终端和过程约束。

它的作用是训练三维制导指标和终端/路径约束的关系。它没有姿态三轴、气动表、控制分配或六自由度刚体动力学，因此不能写成“完整 IGC 仿真”。

三维制导图表包括：

对应图表分别展示地面航迹和目标位置、高度剖面和目标高度、动压曲线和动压上限、过载曲线和过载上限。它们合在一起，才构成三维制导的最小证据组。

## 示例结果文件

结果表并不是简单的运行痕迹，而是整本书里“证据如何分层”的一部分。results/mc4_metrics.csv 和 results/mc4_pairs.csv 保存了多控制器小样本的完整指标与成对差值，其余 results/mc4_lqr_*、results/mc4_ndi_*、results/mc4_backstepping_*、results/mc4_smc_* 和 results/mc4_barrier_* 文件则保存 LQR、NDI、反步、滑模和屏障安全层相对基线的对应结果。写作时，最值得抓住的是成对差值的方向含义：负值通常表示候选控制器在该指标上更低，因此更适合用来说明“安全指标改善，同时跟踪仍在阈值内”。

results/coupled_six_dof_skeleton_trace.csv、results/six_dof_rigid_body_trace.csv、results/aero_table_trace.csv、results/igc_step_summary.csv 和 results/igc_gap_audit.md 共同构成一条逐级递进的证据链：从教学级耦合骨架，到独立六自由度刚体传播，再到气动表插值和 IGC 证据边界。把这些文件放在一起看，才能判断哪些结论已经成立，哪些仍然只是教学级材料。

figures/ 下的 SVG 图表对应纵向跟踪、气动表、耦合骨架、刚体传播、控制分配、接口链、姿态内环、故障注入和三维制导等主题。它们不是目录清单，而是各章节证据进入论文图表表达的出口；也就是说，读者不该把它们当成文件列表，而应把它们当成正文结论的可视化落点。

## 故障注入教学入口

simulation/fault_injection.py 对应第 9 章“估计、故障与工程实现”。它仍是教学级实现：仿真中可直接记录真实状态、估计状态、实际输入和残差，用于训练指标口径；它不是正式机载 FDI/FTC 软件。

支持的故障类型覆盖传感器偏置、执行机构失效和输入延迟三类最常见情形，包括高度/速度/攻角偏置、升降舵效率损失或卡滞、推力损失以及输入延迟。这里的重点不是枚举故障名，而是说明不同故障会把哪些量带入证据链。

因此，输出轨迹记录除了常规状态和控制字段外，还会保留估计量、实际执行量、动压估计量以及残差信号；输出指标则重点记录故障检测延迟、误报和漏报、残差峰值、估计误差 RMS，以及故障后高度、速度和动压违反积分。它们共同用于回答三个问题：故障是否被检测、估计误差是否改变安全裕度、故障后跟踪和动压约束是否仍达标。

批量故障扫描入口：

运行命令：python -m hgv_control.simulation.fault_sweep --controllers baseline guarded --faults none alpha_sensor_bias elevator_loss input_delay --metrics-output hypersonic_control_tutorial\code\python\results\fault_sweep_metrics.csv --pairs-output hypersonic_control_tutorial\code\python\results\fault_sweep_pairs.csv

fault_sweep_metrics.csv 记录每个控制器和每个故障场景的完整指标；fault_sweep_pairs.csv 则记录同一故障场景下候选控制器相对基线的差值。写作时，最值得关注的是故障后动压违反积分、高度 RMS、速度 RMS 和检测延迟这些差值的方向，因为它们直接决定候选控制器能否支撑“故障后安全更满足、跟踪仍达标”这类结论。这个样本矩阵很小，所以它仍然只是教学级证据链模板。

故障图表生成入口使用 python -m hgv_control.plotting.make_fault_figures --metrics-csv hypersonic_control_tutorial\code\python\results\fault_sweep_metrics.csv --pairs-csv hypersonic_control_tutorial\code\python\results\fault_sweep_pairs.csv --output-dir hypersonic_control_tutorial\code\python\figures。这些图表用于把故障扫描结果接到论文图表链路中。图中的负差值通常表示候选控制器相对基线的对应指标更低，但是否可以写成“更好”仍要结合通过率、约束裕度和跟踪阈值判断。

## 可继续补充

后续可继续补如下内容：

| 扩展方向 | 说明 |
| --- | --- |
| 正式 CLF-CBF-QP、BLF-反步和预测控制器 | 把约束安全控制从教学层推进到更规范的优化框架 |
| 高阶滑模、终端滑模和自适应切换增益 | 扩展滑模控制的收敛性与鲁棒性讨论 |
| INDI 的增量式、鲁棒补偿和执行机构延迟处理 | 补足非线性动态逆的工程实现细节 |
| 反步的命令滤波、动态面和饱和补偿 | 降低微分爆炸与执行机构受限问题 |
| LQR 的增益调度、积分扩展和更严格的线性化/配平流程 | 让基线控制更适合大包线比较 |
| MATLAB 与 Simulink 控制器族对照 | 继续把已有 Python 结果迁移到两条实现线 |
| 更真实的气动数据库与完整六自由度刚体力/力矩传播 | 提升气动支撑与三维耦合真实度 |
| 更正式的 FDI/FTC、控制分配重构和故障后参考降级案例 | 把故障场景从教学扫描推进到更完整容错链路 |
