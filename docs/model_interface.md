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

## LapResult

结果至少包含 `lap_time_s`、`s_m`、`v_mps`、`ax_mps2`、`ay_mps2`、`limiter`、`ggv_used`、`options`、逐段时间和求解收敛信息。
