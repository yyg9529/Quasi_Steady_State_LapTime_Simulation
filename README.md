# Quasi-Steady-State Lap Time Simulation

面向 Formula Student / FSAE 的 MATLAB 准稳态圈速仿真项目。

当前版本：V0.9 开发基线。除完整 QSS/GGV 主链和实车校准外，已支持单参数敏感性、20+ case DOE、离线悬架几何 lookup、独立 7DOF 关键事件验证，以及带有效域、坐标、轮胎侧和参数 schema 检查的高级轮胎适配器契约；Simscape 和外部轮胎 evaluator 不属于默认运行时依赖。

## 项目目标

- 在固定赛线离散点上计算横向速度上限、周期前向加速传播和周期后向制动传播。
- 输出圈速、速度、纵横向加速度、限制因素和后续可扩展的能耗结果。
- 支持车辆、轮胎、气动、动力系统和悬架查表模块逐步升级。
- 主求解链不依赖 Simscape；Simscape 仅用于离线悬架预处理和数据导出。

## 目录

```text
config/         配置文件
data/           项目输入数据和基准夹具
docs/           范围、接口、架构和验证文档
examples/       可运行示例
preprocessing/  轮胎、悬架和赛道数据预处理
reference/      冻结的研究报告和旧项目快照
src/            MATLAB 主程序
tests/          单元测试与回归测试
```

`reference/` 只用于追溯和提取可复用资产。生产代码不得在运行时从该目录加载函数或数据。

## 运行环境

- MATLAB R2025b 或更新版本；核心链只使用基础 MATLAB。
- 内部全部采用 SI 单位和 ISO 8855 风格的 `X` 前、`Y` 左、`Z` 上坐标约定。

## 运行第一个示例

在 MATLAB 中将当前目录切换到仓库根目录，然后运行：

```matlab
projectRoot = project_setup();
run("examples/run_001_constant_mu_skidpad.m")
```

运行单元测试：

```matlab
results = runtests("tests/unit", IncludeSubfolders=true);
assertSuccess(results)
```

## 开发顺序

1. V0.1：常数摩擦系数 GGV、固定赛线闭环 QSS 和解析基准。
2. V0.2–V0.4：四轮载荷、气动、载荷敏感轮胎、动力与制动约束。
3. V0.5–V0.7：实车 GGV 校准、DOE、离线悬架 lookup。
4. V0.8：简化 slip-force 本构和 7DOF 关键事件验证。
5. V0.9：Magic Formula / UniTire placeholder 和稳态代数轮胎力适配契约。
6. V1.0：冻结回归基线、文档和可复现实验。

## 当前不支持

- 自由赛线优化、驾驶员模型和全赛道瞬态仿真；
- 在线调用 Simscape；
- 未经有效域、坐标、轮胎侧和来源验证的 Magic Formula / UniTire 参数直接启用；
- V0.1 中的坡度、横坡、路面空间变化与能耗计算。

详细边界见 [`docs/project-scope.md`](docs/project-scope.md)，轮胎契约见 [`docs/tire_adapter.md`](docs/tire_adapter.md)，参考资产见 [`docs/reference-inventory.md`](docs/reference-inventory.md)。
