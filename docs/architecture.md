# 软件架构

## 主数据流

```text
Vehicle / Tire / Aero / Powertrain / Brake
                    ↓
            Vehicle Capability (GGV)
                    ↓
Track curvature → closed-loop QSS propagation
                    ↓
LapResult / limiter / sensitivity
```

QSS core 只查询 GGV，不直接调用轮胎或 Simscape。理论 GGV 的生成属于车辆能力层；实车 GGV 只能校准该能力层。悬架几何由离线工具导出 CSV，运行时仅做读取和插值。

## 层级依赖

1. `data/` 与 `preprocessing/`：输入和边界适配。
2. `src/vehicle|tire|aero|powertrain|brake`：可独立测试的物理模型。
3. `src/ggv`：把物理模型压缩为速度相关的整车能力边界。
4. `src/core`：固定赛线速度传播与时间积分。
5. `src/analysis`：汇总、绘图、敏感性和 DOE。
6. `src/dof7`：关键事件验证，不参与主圈速求解。

`reference/` 是冻结快照，生产代码不得将其加入路径或在运行时读取其中资产。

## 闭环求解更正

圈速赛道是周期边界问题。单次 forward/backward 会依赖数组起点，并可能漏掉末点到首点的约束。本项目把每个节点到下一节点（含 `N → 1`）视为一段，反复施加加速与制动可达性约束，直到速度剖面成为周期不动点。

时间积分使用每段常加速度恒等式：

```text
dt_i = 2 ds_i / (v_i + v_{i+1})
```

而不是左端点近似 `ds_i / v_i`。
