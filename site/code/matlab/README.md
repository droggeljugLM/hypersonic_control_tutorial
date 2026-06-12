# MATLAB 教学示例

本目录提供一个与 Python 对照目录并列的最小 MATLAB 纵向闭环样例。它用于支撑教程第 10 章“MATLAB/Python 仿真与论文案例复现”，不是高保真飞行器模型，也不能用于工程定型。

本目录的说明、指标和公式与正文保持同一套口径：数学表达直接用 $ 和 $$，普通术语不写成代码样式；如果某个符号在本说明中首次出现，应在就近位置把它讲清楚。

本 MATLAB 样例覆盖简化纵向动力学、动压与大气密度、执行机构二阶动态、PID 风格标称控制器、动压参考治理安全控制器、RK4 闭环积分、指标计算、结果表输出和烟测脚本等内容。这样安排的目的，是先给出一个足够小、足够稳定的比较对象，再让读者逐步把控制器、约束和结果口径放进同一个闭环里。

## 运行方式

建议按下面顺序执行：

1. 在 MATLAB 中把本目录加入路径，然后运行 run_demo。
2. 接着运行烟测脚本 run('D:\Phd\code\codex\test\hypersonic_control_tutorial\code\matlab\tests\run_matlab_smoke_test.m')。
3. 如果需要命令行入口，可直接用 matlab -batch "addpath('D:\Phd\code\codex\test\hypersonic_control_tutorial\code\matlab'); run_demo"。
4. 烟测命令可写成 matlab -batch "run('D:\Phd\code\codex\test\hypersonic_control_tutorial\code\matlab\tests\run_matlab_smoke_test.m')"。

run_demo 会生成 results/matlab_baseline_trace.csv 和 results/matlab_guarded_trace.csv 两个文件，分别记录标称和安全轨迹。这里的重点不在文件名本身，而在于它们把两条比较结果放到了同一套指标口径里。

## 适用边界

本目录只实现 MATLAB 最小闭环主线。同目录下的 Simulink 示例已经提供动压监控最小模型和教学级守护纵向闭环最小模型；它们仍不是高保真 Simulink 飞控模型，也没有覆盖 LQR、NDI、反步、滑模和屏障层控制器族。MATLAB 侧也还没有 LQR、NDI、反步、滑模、屏障层对照和 Monte Carlo 批量脚本。后续应继续把 Python 中已有的控制器和指标逐步迁移到 MATLAB/Simulink 双线。
