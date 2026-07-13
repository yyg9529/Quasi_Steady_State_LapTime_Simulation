# Quasi-Steady-State Lap Time Simulation

面向 Formula Student / FSAE 的 MATLAB 准稳态圈速仿真项目。

当前版本：V0.7 开发基线。除完整 QSS/GGV 主链和实车校准外，已支持单参数敏感性、20+ case DOE、冻结 baseline 校准、距离加权 limiter 汇总、结果绘图和离线悬架几何 lookup；Simscape 不属于运行时依赖。

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
4. V0.8–V1.0：7DOF 事件验证、高级轮胎接口和稳定回归。

## 当前不支持

- 自由赛线优化、驾驶员模型和全赛道瞬态仿真；
- 在线调用 Simscape；
- 未经有效域和坐标转换验证的 Magic Formula / UniTire 数据；
- V0.1 中的坡度、横坡、路面空间变化与能耗计算。

详细边界见 [`docs/project-scope.md`](docs/project-scope.md)，参考资产见 [`docs/reference-inventory.md`](docs/reference-inventory.md)。
