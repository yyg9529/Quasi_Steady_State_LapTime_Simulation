# 模型与结果接口

所有生产接口使用 SI 单位；字段名携带单位后缀。除明确标为可选的输入外，生产代码不猜测缺失参数。

## 赛道输入

`read_track_csv` 接受两种 CSV schema：

1. 简化曲率：`s_m,kappa_1pm`；
2. 真实几何：`s_m,x_m,y_m,curvature_1_m,track_width_m`，其中别名映射为 `curvature_1_m → kappa_1pm`。

两种 schema 均要求 `s_m` 严格递增。段长来自文件中的弧长列 `s_m`，不是 XY 欧氏距离；`x_m,y_m` 只用于判定真实几何文件是否以重复首点精确闭合，`track_width_m` 仅保留为逐点元数据，不用于固定赛线或赛线优化。

内部赛道约定：

- 开环：`N` 个节点、`N-1` 个段；
- 闭环：`N 个唯一节点`、`N` 个段，最后一段为节点 `N→1`；
- 闭环真实 geometry 输入必须含重复闭合端点；reader 删除最后重复行，闭合段精确取 `s_end-s_last_unique`，总长取 `s_end-s_start`；
- `track.ds_m` 是段长，`track.s_m` 和 `track.kappa_1pm` 是节点量；
- 只有不含几何坐标的 synthetic 简化 schema 才允许估计闭合段，并用 `closure_is_estimated=true` 明示；真实 geometry 不得估计。

reader 保留字段为 `s_m, ds_m, kappa_1pm, x_m, y_m, track_width_m, is_closed, closure_is_estimated, length_m, source_file`。

真实文件 `data/track/tianji_kart_QSS_track_closed.csv` 是进入 Git 并随仓库提供的正式验证输入。验收目标约为总长 `857.461445744691` m、闭合段 `1.461445744691` m；这些值用于保护文件内容和闭环几何契约。

## 组合动力系统输入

`models.powertrain` 启用时必须提供九个冻结顶层字段，生产接口仅定义这些字段：

```text
enabled, motor_count, layout, gear_ratio,
drivetrain_efficiency, motor, battery, inverter, rules
```

- 当前生产主线要求 `motor_count=1`；`layout` 与车辆驱动桥布置一致。
- `gear_ratio = omega_motor / omega_wheel`，是电机转速与车轮转速比，不是倒数。
- `drivetrain_efficiency` 是电机轴至驱动轮的机械效率。
- 唯一轮胎滚动半径来源是 `tire.rolling_radius_m`，动力配置不复制半径。
- 禁用动力约束的合法最小 sentinel 是空/缺失配置或精确 `struct("enabled",false)`（文档短写 `enabled=false`）。
- 启用配置缺字段或只使用扁平 `max_power_W/max_wheel_torque` 配置时拒绝，并报 `QSSLTS:PowertrainConfig`。

`rules` 冻结 Formula Student Rules 2026 v1.1 的 `max_ts_voltage_V=600`、`max_ts_power_W=80000`、`max_ts_current_A=500` 与 `regen_enabled_in_model=false`。DOE 不得通过改变规则来模拟部件设计变化。

## 完整 PAC2002 四轮 handling

`load_pac2002_tire(tirFile)` 保留完整 section、SHA-256、模型侧和输入范围，并返回可直接传入四轮求解器的 `tireModel.evaluate`。四轮轮位顺序固定为 `[FL, FR, RL, RR]`；调用输入至少包括 `Fz_N,kappa,alpha_rad,gamma_rad,turn_slip_1pm,Vx_mps,mount_side`，输出包括 `Fx_N,Fy_N,Mx_Nm,My_Nm,Mz_Nm,effective_radius_m,within_range`。

`generate_ymd` 在固定 `speed/beta/steer` 网格上求横向力平衡对应的 yaw rate，保留非零质心横摆力矩；`solve_steady_state_cornering` 在固定速度/半径下同时求解横向力与横摆力矩平衡；`calc_understeer_gradient` 输出经典小角度线性化 `linear_fit_gradient_deg_per_g`，以及有限半径真实四轮曲线的 `steady_curve_gradient_deg_per_g` 和 `local_gradient_deg_per_g`。正值表示 understeer，负值表示 oversteer。

完整 PAC2002 handling 是稳态模型，不包含 relaxation dynamics。原始 `.tir` 位于忽略的 `data/tire/local/`，不作为仓库分发资产；圈速/GGV 主线仍只使用降阶包络。

## GGV 与耐久输入

`options.v_grid_mps` 与 `options.ay_grid_g` 定义 GGV 网格。未提供预建 GGV 时，`generate_model_ggv` 在各 `(v, Ay)` 网格点调用动力、轮胎和制动内核；前后向传播随后以节点 `actual Ay_mps2=v^2*kappa`（m/s²）重建横向加速度，并在调用 `interp_ggv` 前除以 `gravity_mps2` 转成 g，因此弯中 Ax 能力不是直线能力。前向加速段对入口和段中代表速度重复该查询，并迭代候选出口速度至分段可达；组合动力配置还在段中状态直接重算固定电压 TSAC 功率/电流上限，避免 GGV 速度网格插值的非保守误差。闭环同样处理 `N→1` 段。

`models.endurance` 独立于 `models.powertrain`，接口为正整数 `num_laps` 和不小于 1 的 `safety_factor`。它只在速度剖面收敛后的能量后处理中使用，不进入 GGV。

## 基础 `LapResult`

每次成功求解至少返回 `lap_time_s, s_m, v_mps, ax_mps2, ay_mps2, limiter, ggv_used, options, v_lateral_limit_mps, segment_time_s, cumulative_time_s, track, calibration_report, solver`。启用组合动力系统后再增加 `result.active_constraints`、`result.powertrain` 和 `result.energy`。

## `result.limiter` 与 `result.active_constraints`

`result.limiter` 是每个节点的唯一主标签；可能值包括 `lateral, brake, traction`、`motor_torque`、`motor_power`、`motor_speed`、`motor_voltage`、`rule_power`、`rule_current`、`battery_power`、`battery_current`、`inverter_power`、`inverter_current`、`top_speed`、`coasting`、`unconstrained`。

`result.active_constraints` 是与节点数等长的 14 个逻辑向量，字段严格为：

```text
lateral, brake, traction, motor_torque, motor_power, motor_speed,
motor_voltage, rule_power, rule_current, battery_power, battery_current,
inverter_power, inverter_current, top_speed
```

同一点可以多重激活；主标签的完整优先级为：

```text
lateral > brake > top_speed > rule_power > rule_current >
battery_power > battery_current > inverter_power > inverter_current >
motor_speed > motor_voltage > motor_power > motor_torque > traction >
coasting > unconstrained
```

激活容差按物理量分别冻结：`lateral` 与 `top_speed` 使用绝对 `0.01 m/s`，`brake` 使用绝对 `0.01 m/s²`，`motor_speed` 使用绝对 `0.01 rpm`；四者均叠加相对 `1e-3`。候选 torque 与 traction utilization 使用相对 `1e-3`。没有主约束时再归为 `coasting` 或 `unconstrained`。

## `result.powertrain`

长度为 `N` 的逐点向量为：

```text
motor_speed_rpm, motor_stop_speed_rpm,
motor_torque_available_Nm, motor_torque_used_Nm,
wheel_force_available_N, wheel_force_used_N,
tsac_power_cap_W, tsac_power_used_W, tsac_dc_current_used_A,
battery_dc_current_used_A, inverter_dc_current_used_A,
motor_phase_current_used_Arms, inverter_phase_current_used_Arms,
V_bus_V, effective_voltage_V,
traction_force_available_N, powertrain_force_available_N
```

`candidate_names` 与 `candidate_limiters` 是全局 `1x12` 标签，`candidate_torque_available_Nm` 是 `Nx12` 候选矩阵。`thermal_feasibility_evaluated`、`regen_enabled`、`voltage_scenario` 和 `endurance_energy_feasible` 是全局 scalar；这后四项不是逐点 active constraint。

TSAC、电池和逆变器的母线电流是 DC A；电机/逆变器相电流是 `phase Arms`，二者不可直接比较或互换。

## `result.energy`

精确段字段为 `segment_ax_mps2, segment_speed_mean_mps, segment_drive_force_N, segment_wheel_power_W, segment_tsac_power_W, segment_tsac_power_cap_W, segment_tsac_power_margin_W, segment_energy_ts_kWh, segment_energy_stored_kWh`。功率裕量严格定义为 `segment_tsac_power_cap_W-segment_tsac_power_W`；允许的负值只限冻结数值容差 `max(1 W, 1e-5*cap)`。`cumulative_energy_ts_kWh` 与 `cumulative_energy_stored_kWh` 长度均为 `Nsegment+1`，首项固定为 0。汇总字段为：

```text
E_lap_ts_kWh, E_lap_stored_kWh, E_endurance_stored_kWh,
E_nominal_required_kWh, SOC_end_estimated,
can_finish_endurance_estimated
```

闭环能量包含最后 `N→1` 段。通用耐久储能需求为单圈储能侧能量乘 `models.endurance.num_laps * models.endurance.safety_factor`，再除以 `SOC_init-SOC_min` 得到 `E_nominal_required_kWh`；正式示例传入 `26` 和 `1.10`。

## 分析与绘图接口

DOE 的 `inverter_power_W -> models.powertrain.inverter.P_dc_peak_W`，`gear_ratio -> models.powertrain.gear_ratio`；flat powertrain 扫描已退役。任何 `rules.*` 变更报 `QSSLTS:AnalysisRulesImmutable`。

`plot_track_speed_map(result)`、`plot_powertrain_energy_result(result)` 与 `plot_ggv_surface(result)` 都是 result-only API；图中的 80 kW/500 A 线是冻结的 FSG rule reference，不是从可变 `rules` 动态生成的设计参数。

三个 API 合计覆盖冻结的 14 项工程视图：

1. `plot_track_speed_map`：XY 赛道按速度着色；
2. `plot_powertrain_energy_result` 的 5x2 tiles：速度/Ax/Ay、曲率、主 limiter、14 项 active constraints、motor speed、used/available torque、TSAC used/cap/reference power、TSAC current/reference、累积单圈能量、limiter 距离占比（第 2–11 项）；
3. `plot_ggv_surface`：GGV 加速/制动 surfaces、速度 slices、actual Ax/Ay lap overlay（第 12–14 项）。
