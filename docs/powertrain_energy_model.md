# 动力系统、能量与约束模型

## 来源与冻结值

规则源为 **Formula Student Rules 2026 v1.1**（`https://www.formulastudent.de/fileadmin/user_upload/all/2026/rules/FS-Rules_2026_v1.1.pdf`）：TS 最大电压 `600 VDC`、TSAC 出口最大功率 `80 kW`、正向最大电流 `+500 A`。preset 的 `rule_ids` 冻结为 power=`EV2.2.1`、current=`EV2.2.2`、voltage=`EV4.1.1`；EV2.2.3 明确允许 regen，但本模型禁用再生。代码冻结 `rules.version="1.1"`。

电机源为 EMRAX 228 datasheet v1.6（`https://emrax.com/wp-content/uploads/2025/03/EMRAX_228_datasheet_v1.6.pdf`）的 HV combined-cooling 数据。数据表额定点包括 `104 kW@4500 rpm`，实现以约 `830 V` 作为达到该峰值功率所需电压；还保留 220 Nm 峰值转矩、6500 rpm 机械转速上限和 phase Arms 数据。75 kW 连续功率与 130 Nm 连续转矩只作为指定 combined-cooling 边界下的诊断数据；没有热模型时不能据此给出热可行性结论。数据表没有提供本项目所需的完整 600 V 转矩-转速图，因此下面的 600 V 包络是保守工程推断，不是供应商认证曲线。

正式示例 `powertrain_emrax228_hvcc_demo` 的工程假设为：单电机 RWD；`gear_ratio=4.369334602435052` 与 `tire.rolling_radius_m=0.2286` m 将 120 km/h 映射到约 6084 rpm；传动效率 0.90、电机效率 0.94。电池参数为 `V_max/V_nominal/V_min/V_bus=600/540/450/600 V`、8 kWh、`SOC_init=0.95`、`SOC_min=0.10`、100 kW/300 A 峰值、放电效率 0.98 和 500 W TS auxiliary。逆变器假设为 600 V、100 kW、300 DC A、250 phase Arms、效率 0.97。Formula Student Rules 2026 v1.1 `D7.1.3` 给出约 22 km 耐久距离；除以本地赛道约 0.857461 km 得 25.66 圈，故取 26 圈；1.10 是显式工程安全系数。它不代表实际赛车的已确认硬件。

## 保守 600 V 电机 T-N 包络

对电机转速 `n`（rpm）和母线电压 `V_bus`（V）：

```text
V_motor_eff = min(V_bus, rules.max_ts_voltage_V,
                  inverter.V_dc_max_V,
                  motor.required_voltage_peak_power_V)

P_voltage = motor.physical_peak_power_W
            * min(V_motor_eff / motor.required_voltage_peak_power_V, 1)
n_peak_load = motor.Kv_peak_load_rpm_per_V * V_motor_eff
n_stop = min(motor.max_mechanical_speed_rpm,
             motor.Kv_no_load_rpm_per_V * V_motor_eff)
omega = n * 2*pi/60                                  [rad/s]

T_peak = motor.peak_torque_Nm                         [N m]
T_power = P_voltage / max(omega, 1 rad/s)             [N m]
T_linear = motor.peak_torque_Nm
           * clamp((n_stop-n)/max(n_stop-n_peak_load,eps),0,1) [N m]
T_available = min(T_peak, T_power, T_linear)           [N m]
```

在 600 V 下，峰值功率按 `104 kW * 600/830` 线性降额，并在峰值负载速度之后线性收缩至停止速度；`n>=n_stop` 时可用转矩为 0。`motor_voltage` 主标签只可能出现在 `n_peak_load<n<n_stop` 的线性收缩区。`1 rad/s` 只用于零速数值正则化。`voltage_limited_engineering_inference` 标签只用于低于约 830 V 的降额区；达到数据表所需电压时标为 `datasheet_peak_power_voltage`。

失效边界：该包络只用于固定正转速、正驱动力、固定 DC 电压的概念级 QSS；不描述 dq/弱磁控制、调制极限、瞬态过载、温升/冷却、SOC 与内阻导致的母线跌落、负转速或再生象限。超出这些条件不能把结果解释为部件能力保证。

## actual Ay 下的驱动力能力

GGV 每个 `(v, Ay)` 网格点和圈速结果重建均采用实际横向状态（actual Ay）。轮胎 combined-slip 先扣除横向力需求，再得到驱动轮纵向抓地上限；`calc_drive_limit` 将其与动力候选转矩共同取最小值。因此弯中 `Ax` 能力不会错误复用 `Ay=0` 的直线结果。

候选约束包括电机转矩/功率/电压/转速、FSG 规则功率/电流、电池功率/电流、逆变器 DC 功率/DC 电流/相电流和轮胎牵引。TSAC 上限及其机械功率链为：

```text
V_TSAC_eff = min(V_bus, rules.max_ts_voltage_V,
                 battery.V_max_V, inverter.V_dc_max_V)
P_TSAC_cap = min(P_rule, V_TSAC_eff*I_rule,
                 P_battery, V_TSAC_eff*I_battery,
                 P_inverter, V_TSAC_eff*I_inverter)
P_inverter_in_cap = max(P_TSAC_cap-P_ts_aux, 0)
P_motor_mechanical_cap = P_inverter_in_cap*eta_inverter*eta_motor
```

前向传播不再把入口 `Ax` 当作整段恒定可用能力。候选出口速度按入口和段中平均速度查询 GGV，每个查询都保持 `actual Ay=v^2*kappa`；组合动力配置还在段中状态用同一固定母线电压、效率和 `calc_ts_power_cap` 直接重算可用 `Ax`，消除 1 m/s GGV 速度网格在线性插值功率曲线时的非保守误差。若所需分段加速度超过任一状态能力，则迭代降低出口速度。该约束发生在速度剖面求解层，节点/分段功率结果没有使用 `min(used,cap)` 截断。

电池 `eta_discharge` 只用于从 TSAC 能量换算储能侧能量，不再乘进瞬时驱动力链。轮端关系为：

```text
n_motor = v / tire.rolling_radius_m * gear_ratio * 60/(2*pi)
F_wheel = T_motor * gear_ratio * drivetrain_efficiency
          / tire.rolling_radius_m
```

## 功率与电流域

驱动时逐点使用量采用：

```text
F_drive = max(m*Ax + F_drag, 0)          (F_rolling=0)
P_TSAC = F_drive*v /
         (eta_drivetrain*eta_inverter*eta_motor) + P_ts_aux
I_TSAC_DC = P_TSAC / V_bus
I_motor_phase_Arms = T_motor / Kt
```

`tsac_dc_current_used_A` 与 `battery_dc_current_used_A` 按包含 auxiliary 的 `P_TSAC/V_bus` 计算；`inverter_dc_current_used_A` 先扣除 auxiliary，再除以母线电压。它们都是 DC A；`motor_phase_current_used_Arms` 与 `inverter_phase_current_used_Arms` 是 phase Arms。DC 电流与相 RMS 电流属于不同电气域，不能用同一个数值上限替代。规则 `500 A` 是 TSAC 出口正向 DC 电流；逆变器 `I_phase_peak_Arms` 独立参与候选约束。

## active constraints 与主 limiter

`result.active_constraints` 具有 14 个字段，可同时为真：`lateral, brake, traction, motor_torque, motor_power, motor_speed, motor_voltage, rule_power, rule_current, battery_power, battery_current, inverter_power, inverter_current, top_speed`。主 `result.limiter` 的精确优先级为 `lateral > brake > top_speed > rule_power > rule_current > battery_power > battery_current > inverter_power > inverter_current > motor_speed > motor_voltage > motor_power > motor_torque > traction > coasting > unconstrained`。激活容差按量纲冻结：`lateral` 与 `top_speed` 使用绝对 `0.01 m/s`，`brake` 使用绝对 `0.01 m/s²`，`motor_speed` 使用绝对 `0.01 rpm`；四者均叠加相对 `1e-3`。候选 torque 与 traction utilization 使用相对 `1e-3`。完整数据契约见 [model_interface.md](model_interface.md)。

## 单圈和耐久能量

能量只在速度剖面收敛后计算，不反馈到 GGV。每段以首末速度均值和段加速度计算 TSAC 与储能侧能量：

```text
E_segment_TS = P_TSAC * dt / 3.6e6                         [kWh]
E_segment_stored = E_segment_TS / battery.eta_discharge   [kWh]
E_lap_stored = sum(E_segment_stored)
E_endurance_stored = E_lap_stored * num_laps * safety_factor
E_nominal_required = E_endurance_stored / (SOC_init-SOC_min)
```

每段还输出 `segment_tsac_power_cap_W` 和 `segment_tsac_power_margin_W=cap-used`。功率验收容差冻结为 `max(1 W,1e-5*cap)`；这只吸收浮点误差，不能接受原先约 48 W 节点超限或 3.04 kW 分段超限。

## P1 旧参数数值基线

使用同一概念车辆/轮胎/制动/EMRAX-HVCC 参数和本地 Tianji 闭合赛道，仅改变分段可达性离散，R2026a 结果为：

| 指标 | 修复前 | P1 修复后 | 变化 |
|---|---:|---:|---:|
| 圈时 | 41.945736448148 s | 42.097939257058 s | +0.152202808910 s |
| 单圈 TSAC 能量 | 0.373934098748 kWh | 0.369764068982 kWh | -0.004170029766 kWh |
| 最大节点 TSAC 功率 | 80047.930791 W | 77924.949499 W | 无超限 |
| 最大分段 TSAC 功率 | 83040.483881 W | 80000.074822 W | 在 1 W 容差内 |
| 最小分段功率裕量 | -3040.483881 W | -0.074822 W | 在 1 W 容差内 |

修复后节点功率、节点轮端力和分段功率超限计数均为 0，求解 2 次外迭代收敛。该表只隔离 P1 数值影响；2026 实车滚动半径、传动比和制动输入尚未确认，未混入本基线。

闭环积分包含最后 `N→1` 段。正式演示传入 `models.endurance.num_laps=26`、`safety_factor=1.10`；通用接口并不硬编码这两个值。模型重复同一个已求得的最小圈速剖面，因此是 First-order endurance estimate。

冻结假设是固定 DC 母线电压、常数效率、无回生、无热状态、`F_rolling=0`。`thermal_feasibility_evaluated=false` 与 `regen_enabled=false` 是显式失效边界；容量可行性只比较估算 SOC 窗口，不替代电芯、BMS、内阻、温度和寿命验证。

## DOE 与工程图

DOE 的 `inverter_power_W` 映射到 `models.powertrain.inverter.P_dc_peak_W`，`gear_ratio` 直接映射到 `models.powertrain.gear_ratio`；旧扁平动力扫描已退役，改变 `rules.*` 报 `QSSLTS:AnalysisRulesImmutable`。

以下 API 只读取 `result`：

```matlab
plot_track_speed_map(result)
plot_powertrain_energy_result(result)
plot_ggv_surface(result)
```

动力图中的 80 kW/500 A 线是冻结的 **FSG rule reference**，不是跟随 DOE 或任意动态规则输入改变的设计参数。图和数值用于同一假设集下的相对筛选，不代表热、电气或实车绝对精度已验证。
