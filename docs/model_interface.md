# 核心数据契约

## Track

`track.s_m` 和 `track.kappa_1pm` 为 `N×1` 节点量。闭环赛道 `track.is_closed=true` 时，`track.ds_m` 为 `N×1`，第 `i` 项表示节点 `i` 到下一节点的弧长，最后一项表示 `N → 1`；开环数据的 `ds_m` 为 `(N-1)×1`。

V0.1 CSV 的每行代表一个闭环段的起点，最后一行不要重复第一点。若只有 `s_m,kappa_1pm`，读取器默认闭环，并用代表性采样间距补齐最后一段。

## GGV

```matlab
ggv.v_mps       % Nv x 1, strictly increasing
ggv.ay_g        % 1 x Nay, strictly increasing and signed
ggv.ax_max_g    % Nv x Nay, acceleration, positive
ggv.ax_min_g    % Nv x Nay, braking, negative
ggv.feasible    % Nv x Nay, lateral feasibility
```

矩阵的第一维对应速度，第二维对应横向加速度。查询接口接收 `ay_g`；由 `v^2*kappa` 得到的是 `m/s^2`，必须除以 `g` 后查询。

## Vehicle 与模型

车辆参数按 `mass`、`geometry`、`inertia`、`load_transfer` 和 `drivetrain` 分组。任何核心函数不得硬编码车辆质量、轴距、轮距或重心高度。

`models` 可包含 `tire`、`aero`、`powertrain`、`brake`、`ggv`、`ggv_real`、`suspension`。若没有 `models.ggv`，`run_qss_lap` 生成理论 GGV；若存在实车能力数据，只在车辆能力层校准。

`powertrain.max_total_wheel_torque_Nm` 表示经过齿比后的所有驱动轮轮端扭矩总和，因此扭矩分支不再乘传动效率；`max_power_W` 视为效率前功率，功率分支乘 `drive_efficiency`。`powertrain.layout` 是动力模型的单一真源，并在生成 GGV 时与 `vehicle.drivetrain.layout` 校验一致。

`brake.front_bias` 是总制动力需求的前轴比例。制动附着上限同时满足前轴和后轴容量，而不是简单使用四轮容量之和。`max_decel_g_mechanical` 是概念阶段的质量比例限制；质量 DOE 若需要固定硬件能力，应改用 `max_total_brake_force_N`。

## LapResult

结果至少包含 `lap_time_s`、`s_m`、`v_mps`、`ax_mps2`、`ay_mps2`、`limiter`、`ggv_used`、`options`、逐段时间和求解收敛信息。

## Suspension lookup

`read_simscape_suspension_lookup` 只读取离线导出的 CSV，并将角度从 deg 转为 rad；它不会打开或执行 Simscape 模型。`interp_suspension_lookup` 接收 mm 表示的 jounce，并返回 `camber_rad`、`toe_rad`、`motion_ratio` 和 `damper_stroke_mm`。默认禁止越界；只有显式指定 `OutOfRange="clamp"` 才会钳位。

缺少 lookup 时使用 `make_constant_suspension`。该 fallback 不虚构 Motion Ratio，返回 `motion_ratio=NaN` 和 `motion_ratio_available=false`。V0.7 尚未定义轮荷到 jounce 的垂向平衡，因此悬架 lookup 不能改变 GGV。
