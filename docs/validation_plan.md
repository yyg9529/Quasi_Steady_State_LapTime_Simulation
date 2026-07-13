# 验证计划

## 分层原则

- Unit：公式、单位、符号、输入校验和单函数契约。
- Benchmark：组合链路的解析、对称性、可行性和网格收敛。
- Regression：只冻结有独立解析依据的 correctness 值。
- Reproducibility：固定配置重复运行得到相同报告；不等于物理真值。

## 版本门禁

- V0.1：Track CSV 形状；常数 μ；定半径 `sqrt(mu*g*R)`；闭环传播；空间时间积分。
- V0.2：轮荷守恒；纵横载荷转移方向；气动力平方律；抬轮诊断。
- V0.3：载荷敏感方向；combined-slip 轴端点和连续性。
- V0.4：附着/扭矩/功率/最高车速分支；制动偏置和机械极限。
- V0.5：校准限幅、速度表连续性、缺失数据错误和报告。
- V0.6：每个 Sensitivity/DOE case 重生理论 GGV；baseline 残差比例冻结；排名可追溯。
- V0.7：悬架断点插值、deg/rad、Motion Ratio 定义和越界策略。
- V0.8：7DOF 力/功率符号、低速正则、左右镜像、能量趋势和事件终止。
- V0.9：adapter/schema 配对；坐标与轮胎侧；压力条件；有效域；零轮荷 bypass；额外输出拒绝；异常不 fallback；QSS provenance 排除 `slip_force`。
- V1.0：解析 regression、直线 benchmark、曲率镜像、整圈 GGV 可行性、固定点残差、GGV 网格加密、27-case DOE、完整 limiter 报告、manifest 和重复运行一致性。

## V1.0 定量门禁

1. 定半径解析基准：
   - `R=8 m`、`mu=1.5`、`g=9.80665 m/s^2`；
   - `v=10.8480320795986 m/s`，`AbsTol=1e-6 m/s`；
   - CSV 量化长度对应圈时 `4.63360373855568 s`，`AbsTol=1e-6 s`。
2. `ay=0`、无气动、RWD 常数 μ GGV：
   - 加速 `1.161020881670534 g`；
   - 制动幅值 `1.440414507772021 g`；
   - `AbsTol=1e-6 g`。
3. 直线加速覆盖 `torque`、`power`、`top_speed`；动力系统最高车速误差不超过 `1e-5 m/s`。
4. 直线制动与 `v(s)=sqrt(2*g*(L-s))` 一致，`AbsTol=1e-8 m/s`。
5. 曲率镜像：圈时 `1e-8 s`、速度 `1e-7 m/s`、加速度 `1e-6 m/s^2` 内对称。
6. 整圈每点满足 `ax_min <= ax <= ax_max` 和 `v <= v_lateral_limit`，纵向余量 `1e-6 m/s^2`。
7. canonical GGV 的 feasible 点全部固定点收敛、残差不超过配置阈值、无意外 wheel lift、横向搜索未截断。
8. GGV 网格由 `1 m/s + 0.1 g` 加密为 `0.5 m/s + 0.05 g` 后，演示赛道圈时相对差小于 `0.2%`。
9. DOE 至少 20 case；V1 固定为 `3x3x3=27` case，全部收敛、全部输出圈时并排序。
10. limiter 按 `track.ds_m` 距离加权；纵向能力标签只在实际 `ax` 贴合 GGV 上/下界时发布，内部点标为 `unconstrained`；百分比合计 100%；拒绝空、未知和 `lateral_infeasible` 发布标签。
11. 同一代码、MATLAB 版本和平台连续两次生成的 7 个 V1 报告文件 SHA-256 一致。

## 回归冻结规则

允许冻结解析定半径和解析 `ay=0` GGV。以下结果只可记录为软件快照，不得作为物理 truth：`simple_track` 精确圈时/节点值、DOE 精确排名、synthetic GGV 校准结果、limiter 点数、7DOF 精确事件时间和 solver 迭代次数。

## 验收命令

```matlab
projectRoot = project_setup();
results = runtests(fullfile(projectRoot,"tests"), IncludeSubfolders=true);
assertSuccess(results)
run(fullfile(projectRoot,"examples","run_003_real_ggv_calibrated_lap.m"))
run(fullfile(projectRoot,"examples","run_007_v1_reproducible_study.m"))
```

验证结果和未完成项见 [v1_validation_report.md](v1_validation_report.md)。
