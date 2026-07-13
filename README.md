# Quasi-Steady-State Lap Time Simulation

面向 Formula Student / FSAE 的 MATLAB 固定赛线准稳态圈速仿真项目。

当前版本：V1.0 软件可复现基线。项目已具备完整 QSS/GGV 主链、四轮轮荷与载荷转移、气动、载荷敏感轮胎包络、动力/制动约束、外部 GGV 残差校准、Sensitivity/DOE、离线悬架 lookup、独立 7DOF 关键事件验证，以及高级轮胎稳态代数适配契约。

“V1.0 稳定”仅表示接口、测试和确定性实验已形成软件基线，不表示圈速或轮胎模型已经实车/外部物理验证。

## 核心输出

- 圈时、速度剖面、纵横向加速度和横向速度上限；
- 每段 limiter 状态（含无活动约束）及按赛道距离加权的分布；
- 理论或校准后的 GGV、求解收敛诊断和输入 provenance；
- 单参数敏感性与 27-case 全因子 DOE 排名；
- 最快/最慢 DOE case 的 limiter 对比和可复现实验 manifest。

## 目录

```text
config/         配置文件
data/           输入数据、演示夹具和解析回归基准
docs/           范围、接口、架构和验证文档
examples/       可运行示例
preprocessing/  轮胎、悬架和赛道数据预处理
reference/      冻结的研究报告和旧项目快照
src/            MATLAB 主程序
tests/          unit、benchmark 和 regression 测试
```

`reference/` 不进入生产运行路径。`data/track/simple_track.csv` 和 `data/ggv_real/synthetic_real_ggv.csv` 是软件演示夹具，不是实车性能证据。

## 环境与约定

- 已在 Windows、MATLAB R2025b Update 5 验证；其他版本尚未逐一验证。
- 核心 QSS 链只依赖基础 MATLAB；Simscape 仅用于离线数据预处理，不是运行时依赖。
- 内部使用 SI 单位和 `X` 前、`Y` 左、`Z` 上的车辆坐标约定。

## 快速开始

在 MATLAB 中切换到仓库根目录：

```matlab
projectRoot = project_setup();

% 解析定半径基准
run(fullfile(projectRoot,"examples","run_001_constant_mu_skidpad.m"))

% 任务书最终入口：synthetic GGV 校准、profile、limiter、sensitivity
run(fullfile(projectRoot,"examples","run_003_real_ggv_calibrated_lap.m"))

% V1.0 确定性 27-case DOE 报告
run(fullfile(projectRoot,"examples","run_007_v1_reproducible_study.m"))
```

运行全部测试：

```matlab
results = runtests(fullfile(projectRoot,"tests"), IncludeSubfolders=true);
assertSuccess(results)
```

## V1.0 报告文件

`run_007` 在 `results/v1/` 生成：

- `experiment_manifest.json`
- `baseline_summary.csv`
- `baseline_profile.csv`
- `baseline_limiter.csv`
- `doe_ranking.csv`
- `doe_case_metrics.csv`
- `doe_extreme_limiter_comparison.csv`

`results/` 被 Git 忽略，报告通过固定入口重生。manifest 记录赛道数组、车辆、轮胎包络 provenance、气动、动力、制动、求解选项、GGV 网格、DOE levels、MATLAB/platform，以及 65 个生产输入文件的 SHA-256；`Inf/NaN` 采用显式字符串编码。

## 已交付里程碑

1. V0.1：常数 μ、固定赛线闭环 QSS 和解析定半径基准。
2. V0.2：四轮轮荷、纵横载荷转移和气动。
3. V0.3：载荷敏感轮胎与 combined-slip 包络。
4. V0.4：动力系统、制动偏置和机械约束。
5. V0.5：外部 GGV 读取、速度相关残差校准和报告。
6. V0.6：Sensitivity 与每 case 重生理论 GGV 的 DOE。
7. V0.7：离线悬架几何 lookup 接口。
8. V0.8：简化 slip-force 本构和 7DOF 关键事件验证。
9. V0.9：Magic Formula / UniTire placeholder 与稳态代数适配契约。
10. V1.0：解析回归、benchmark、limiter 分析、27-case 可复现实验和文档冻结。

## 当前不支持

- 自由赛线优化、驾驶员模型、全赛道瞬态仿真；
- 能耗模型、坡度/横坡、空间变化路面附着；
- raw log 到 GGV 的自动识别；
- 在线调用 Simscape，或让悬架 lookup 直接改变 GGV；
- 随仓库交付的已验证 Magic Formula / UniTire evaluator 或参数集。

详细边界见 [project-scope.md](docs/project-scope.md)，验证证据见 [v1_validation_report.md](docs/v1_validation_report.md)，轮胎契约见 [tire_adapter.md](docs/tire_adapter.md)。
