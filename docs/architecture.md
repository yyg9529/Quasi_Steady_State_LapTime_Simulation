# 软件架构

## 主数据流

```text
track CSV + presets + options
            |
            v
      read_track_csv
            |
            v
 generate_model_ggv -- 每个 (v, Ay) 状态调用 calc_drive_limit / 制动与轮胎内核
            |
            v
       run_qss_lap
    lateral limit -> forward pass -> backward pass -> 速度剖面收敛后
            |
            +--> result.limiter / result.active_constraints
            +--> result.powertrain
            +--> 速度剖面收敛后计算 result.energy
            +--> result-only plots / Sensitivity/DOE
```

`run_qss_lap` 是固定赛线主编排器。若没有预建 GGV，它调用 `generate_model_ggv`；前向和后向传播均以当前节点速度和曲率重建 `actual Ay = v^2*kappa`，并在该 Ay 处查询纵向加速或制动能力。前向加速段对入口和段中代表状态执行隐式可达性修正，避免把入口功率能力外推到整段；组合动力系统同时把段中固定电压功率上限作为内部可达性约束传入，后向制动传播保持原接口。GGV 和圈速后处理使用同一冻结配置，避免动力参数与预建图 provenance 不一致。

## 模块职责

- `preprocessing/track`：赛道输入，负责 schema、单位、闭环弧长与 provenance。
- `preprocessing/tire`：只做 PAC2002 标量解析与 QSS 离线降阶；生产求解器不读取 `reference/`。
- `src/vehicle`、`src/tire`、`src/aero`：车辆状态、四轮载荷、轮胎包络和气动力。
- `src/powertrain`：组合动力配置校验、600 V 电机包络、电池/逆变器/规则候选约束、逐点使用量和能量。
- `src/ggv`：在速度/横向加速度网格上生成或插值 GGV；`plot_ggv_surface` 只消费 result。
- `src/core`：固定赛线速度传播、圈时积分、active constraint 分类和求解收敛诊断。
- `src/analysis`：Sensitivity/DOE、limiter 汇总和 result-only 工程图。
- `data/` 各模型目录：车辆、轮胎、气动、动力和制动的可复现配置。

`models.endurance` 是独立于 `models.powertrain` 的重复圈配置，只含圈数和安全系数。能量仅在速度剖面收敛后后处理，不进入 GGV 生成，也不反向限制本圈速度。

## 闭环与收敛

闭环赛道以 `N` 个唯一节点和 `N` 个段表示；第 `N` 段是精确 `N → 1` 闭合段。前向/后向传播循环处理闭合边界，直至速度剖面变化低于容差；圈时、能量和距离加权 limiter 统计均包含最后一段。开环为 `N` 节点、`N-1` 段。

## 依赖方向与禁止项

主线依赖从输入/预设流向物理内核、GGV、圈速与分析；绘图不回调求解器，不允许用图形函数重算或修改结果。DOE 只复制并修改分析参数，规则对象保持冻结；若尝试更改 `rules.*`，应报 `QSSLTS:AnalysisRulesImmutable`。主线不再依赖专属瞬态 DOF 目录或适配器。
