# 核心数据契约

## Track

`track.s_m` 和 `track.kappa_1pm` 为 `N×1` 节点量。闭环赛道 `track.is_closed=true` 时，`track.ds_m` 为 `N×1`，最后一项表示 `N → 1`；开环时为 `(N-1)×1`。CSV 每行表示一个段起点，不重复首点。当前时间积分使用分段恒加速度形式 `dt=2*ds/(v_i+v_{i+1})`，不是简单的节点 `ds/v_i`。

## GGV

```matlab
ggv.v_mps                    % Nv x 1, strictly increasing
ggv.ay_g                     % 1 x Nay, signed and increasing
ggv.ax_max_g                 % Nv x Nay
ggv.ax_min_g                 % Nv x Nay
ggv.feasible                 % Nv x Nay
ggv.ay_limit_pos_g
ggv.ay_limit_neg_g
ggv.accel_limiter
ggv.brake_limiter
ggv.solve_converged_accel
ggv.solve_converged_brake
ggv.solve_residual_accel_mps2
ggv.solve_residual_brake_mps2
ggv.lateral_limit_truncated
ggv.wheel_lift
ggv.source
ggv.gravity_mps2
ggv.provenance
ggv.options
```

第一维是速度，第二维是横向加速度。`interp_ggv` 接收 `ay_g`；`v^2*kappa` 的结果为 `m/s^2`，查询前必须除以 `g`。连续可行边界处的数值通过边界插值获得，分类标签从邻近可行格点选择，不能把相邻不可行格点的 `lateral_infeasible` 传播到可发布结果。

若提供 `models.ggv`，`run_qss_lap` 跳过模型 GGV 生成；否则由车辆/轮胎/气动/动力/制动生成。Sensitivity/DOE 会移除固定理论 GGV并为每个 case 重生；有 `ggv_real` 时只冻结 baseline 的速度相关残差比例。

## Vehicle 与模型

车辆参数按 `mass`、`geometry`、`inertia`、`load_transfer` 和 `drivetrain` 分组。核心函数不得硬编码车辆质量、轴距、轮距或重心高度。

`models` 可包含 `tire`、`aero`、`powertrain`、`brake`、`ggv`、`ggv_real`、`suspension`。

- `powertrain.max_total_wheel_torque_Nm`：所有驱动轮的轮端总扭矩，不再乘效率。
- `powertrain.max_power_W`：效率前功率，功率分支乘 `drive_efficiency`。
- `brake.front_bias`：总制动力需求的前轴比例。
- `brake.max_total_brake_torque_Nm`：7DOF 使用的四轮轮端机械制动总扭矩。

## LapResult

```text
lap_time_s, s_m, v_mps, ax_mps2, ay_mps2, limiter,
ggv_used, options, v_lateral_limit_mps,
segment_time_s, cumulative_time_s, track,
calibration_report,
solver.converged, solver.iterations, solver.max_change_mps
```

## Analysis 与 limiter 报告

`summarize_lap_result` 保留标量指标，并在 `summary.limiter_table` 返回：

```text
limiter, point_count, distance_m, percent_distance
```

百分比按 `track.ds_m` 加权，不按节点计数。可发布标签为：

```text
lateral, coasting, unconstrained, top_speed, tire, traction, torque, power,
brake_disabled, brake_traction_bias, brake_mechanical,
front_axle_lift, rear_axle_lift, measured
```

`unconstrained` 表示该段纵向加速度位于 GGV 上下界内部，当前没有纵向能力边界处于活动状态；`coasting` 仅用于近零纵向加速度。空值、未知标签、内部临时标签和 `lateral_infeasible` 会被拒绝。表中占比表示 limiter 状态覆盖（包括 `unconstrained`），不等同于某个因素造成的圈时秒差。

`run_doe` 返回 baseline、所有 case 结果、排序、最快/最慢 summary、两端 limiter 表和对比表；calibration mode 为 `theory_only` 或 `frozen_baseline`。

## V1 report manifest

`QSSLTS_V1_REPORT_V1` manifest 固定：Track 数组、Vehicle、QSS tire envelope provenance、Aero、Powertrain、Brake、完整 solver/GGV options、DOE levels、case 数和 calibration mode。JSON 中非有限标量使用 `"Inf"`、`"-Inf"` 或 `"NaN"`，避免静默变为 `null`。外部 evaluator 函数句柄不属于 QSS GGV provenance。

## Suspension lookup

`read_simscape_suspension_lookup` 只读取离线 CSV 并将 deg 转 rad；`interp_suspension_lookup` 接收 mm jounce，返回 camber、toe、Motion Ratio 和 damper stroke。默认禁止越界；只有显式 `OutOfRange="clamp"` 才钳位。缺少 lookup 时 `make_constant_suspension` 返回 `motion_ratio=NaN`。截至 V1.0，悬架 lookup 不改变 GGV。

## 7DOF 与高级轮胎适配

`tire.model_type` 描述 QSS/GGV 包络；`tire.slip_force.model_type` 描述 7DOF 滑移—力本构，二者不能互相冒充。`external_adapter` 是无状态、确定性的稳态代数接口，不是松弛长度状态模型。详细 schema、坐标、轮胎侧、压力和有效域规则见 [tire_adapter.md](tire_adapter.md)。

理论 GGV 的 `tire_envelope_provenance` 排除 `slip_force`，因为该本构不参与 QSS 包络生成，且 evaluator 句柄不可稳定序列化。
