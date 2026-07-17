# Quasi-Steady-State Lap Time Simulation

面向 Formula Student / FSAE 的 MATLAB 固定赛线准稳态圈速仿真（fixed-raceline QSS）。当前主线从赛道曲率、车辆、轮胎、气动、制动和组合动力系统生成 GGV，求解闭环速度剖面，并给出 limiter、单圈能量与耐久容量的一阶估算。结果适合设计相对筛选，不代表实车绝对精度。

## 环境与边界

- 当前验收环境：Windows、MATLAB R2026a；核心运行只依赖基础 MATLAB。
- 内部使用 SI 单位，车辆坐标为 `X` 前、`Y` 左、`Z` 上。
- `reference/` 是本地参考目录、被 Git 忽略且不进入生产运行路径。
- 真实赛道 `data/track/tianji_kart_QSS_track_closed.csv` 保持本地未跟踪，不随仓库分发；正式示例不提供替代赛道或静默回退。
- 模型是固定赛线 QSS，不包含自由赛线、驾驶员模型、全赛道瞬态整车或热状态模型。

## 正式入口

从仓库根目录启动 MATLAB：

```matlab
projectRoot = project_setup();
run(fullfile(projectRoot, "examples", ...
    "run_qss_emrax228_hvcc_energy_demo.m"))
```

该入口直接读取 `data/track/tianji_kart_QSS_track_closed.csv`，使用 `vehicle_baseline`、载荷敏感轮胎、基准气动、EMRAX 228/HVCC 动力系统和基准制动，在线生成 GGV 并运行 QSS。示例输出圈时、最高车速、最高电机转速、最大 TSAC 功率/电流、limiter 分布、单圈 TSAC/储能侧能量及耐久所需标称容量。

正式示例的显式工程假设为：传动比 `4.369334602435052`、传动效率 `0.90`、演示用固定 600 V 电池/逆变器参数、耐久 `26` 圈、安全系数 `1.10`；无再生制动、无热降额。参数来源和失效边界见 [动力与能量模型](docs/powertrain_energy_model.md)。

规则基线是 Formula Student Rules 2026 v1.1 的 600 V、80 kW、正向 +500 A 限制；EMRAX 数据表点为 `104 kW@4500 rpm` 且约需 830 V，故正式示例在 600 V 下采用文档所述的保守降额包络。

## 结果与可视化

核心结果包括：

- 圈时、速度、纵横向加速度、GGV 与求解收敛信息；
- `result.limiter` 和可多重激活的 `result.active_constraints`；
- `result.powertrain` 的电机、轮端、TSAC、电池和逆变器逐点量；
- `result.energy` 的单圈能量、重复圈耐久估计、SOC 与容量可行性。

三个绘图 API 只接受求解结果，不重新计算模型：

```matlab
trackFigure = plot_track_speed_map(result);
powertrainFigure = plot_powertrain_energy_result(result);
ggvFigure = plot_ggv_surface(result);
```

## 测试

```matlab
projectRoot = project_setup();
results = runtests(fullfile(projectRoot, "tests"), ...
    IncludeSubfolders=true);
assertSuccess(results)
```

仓库不在 README 中冻结测试数量；应以当前 MATLAB 运行结果为准。模型范围、架构和数据契约分别见 [project-scope.md](docs/project-scope.md)、[architecture.md](docs/architecture.md) 和 [model_interface.md](docs/model_interface.md)。
