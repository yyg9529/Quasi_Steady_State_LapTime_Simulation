# 动力与能量模型冻结契约

## 范围

组合电驱接口已经接入 `calc_drive_limit`、模型 GGV 与圈速积分链路。
旧的扁平动力接口暂时保留给尚未迁移的 legacy/DOE 调用；新旧接口不互设别名，
组合配置不完整时直接报错，不回退到旧模型。

V1 仅支持 `motor_count=1`，其他值抛出
`QSSLTS:PowertrainMotorCount`。齿比定义为
`gear_ratio = omega_motor / omega_wheel`，滚动半径只允许来自
`tire.rolling_radius_m`。`models.endurance` 是独立模型，不嵌入
`models.powertrain`。

## 冻结配置

组合配置顶层严格为：

```text
enabled, motor_count, layout, gear_ratio, drivetrain_efficiency,
motor, battery, inverter, rules
```

规则字段：

```text
rules.name
rules.season
rules.version                    % "1.1"
rules.max_ts_voltage_V           % 600 VDC
rules.max_ts_power_W             % 80 kW
rules.max_ts_current_A           % 500 A
rules.regen_enabled_in_model     % false
rules.source, rules.source_url, rules.rule_ids
```

电机计算字段：

```text
motor.name                       % "EMRAX_228_HV_CC"
motor.source, motor.source_url
motor.mass_kg
motor.max_mechanical_speed_rpm
motor.physical_peak_power_W
motor.physical_peak_power_rpm
motor.physical_cont_power_W
motor.peak_torque_Nm
motor.cont_torque_Nm
motor.required_voltage_peak_power_V
motor.peak_phase_current_Arms
motor.cont_phase_current_Arms
motor.Kv_no_load_rpm_per_V
motor.Kv_nominal_load_rpm_per_V
motor.Kv_peak_load_rpm_per_V
motor.Kt_Nm_per_Arms
motor.eta_const                  % 0.94, engineering assumption
```

电池与逆变器字段只使用以下名称：

```text
battery.V_max_V
battery.V_nominal_V
battery.V_min_V
battery.V_bus_assumed_V
battery.E_nominal_kWh
battery.SOC_init
battery.SOC_min
battery.P_discharge_peak_W
battery.I_discharge_peak_A
battery.eta_discharge
battery.P_ts_aux_W

inverter.V_dc_max_V
inverter.P_dc_peak_W
inverter.I_dc_peak_A
inverter.I_phase_peak_Arms
inverter.eta_const
```

demo 是 1 motor、RWD、`gear_ratio=4.369334602435052`、传动效率
0.90 的概念配置。该齿比把 600 V 推断截止点 6084 rpm、
`tire.rolling_radius_m=0.2286 m` 映射到 120 km/h；半径不存入动力配置。
电池 600/540/450 V、8 kWh、SOC 0.95/0.10、100 kW、300 A、效率
0.98、500 W 辅助功率，以及逆变器 600 V、100 kW、300 A DC、
250 Arms、效率 0.97，均为 concept demo 假设，不代表实车。

## 官方依据与建模边界

[Formula Student Rules 2026 v1.1](https://www.formulastudent.de/fileadmin/user_upload/all/2026/rules/FS-Rules_2026_v1.1.pdf)：

- EV2.2.1：TSAC 出口功率不超过 80 kW；
- EV2.2.2：TSAC 出口电流不超过 500 A；
- EV2.2.3：允许能量回收；
- EV4.1.1：主牵引系统最大 600 VDC。

80 kW 与 500 A 均是 TSAC 出口 DC 限制。规则允许回生，
`regen_enabled_in_model=false` 是 V1 模型选择，不是规则禁止回生。

[EMRAX 228 datasheet v1.6](https://emrax.com/wp-content/uploads/2025/03/EMRAX_228_datasheet_v1.6.pdf)
给出 HV 电机在 830 V 下 104 kW@4500 rpm（S2 2 min）、峰值扭矩
220 Nm、限制转速 6500 rpm，以及 CC 条件下 75 kW/130 Nm 连续额定。
75 kW/130 Nm 是依赖规定冷却边界的目录额定，当前模型没有热状态，不能据此得出实车热连续能力结论。
官方峰值效率为 96%；`motor.eta_const=0.94` 是 demo 的保守常效率假设。
`Kt=0.94 Nm/Arms` 与效率 0.94 数值相同但物理意义不同。

## 电压包络

对给定母线电压和电机转速：

```text
V_eff = min(V_bus,
            rules.max_ts_voltage_V,
            inverter.V_dc_max_V,
            motor.required_voltage_peak_power_V)

P_voltage = motor.physical_peak_power_W
            * min(V_eff / motor.required_voltage_peak_power_V, 1)
n_peak_load = motor.Kv_peak_load_rpm_per_V * V_eff
n_stop = min(motor.max_mechanical_speed_rpm,
             motor.Kv_no_load_rpm_per_V * V_eff)

T_linear = motor.peak_torque_Nm
           * clamp((n_stop-rpm) / max(n_stop-n_peak_load, eps), 0, 1)

T_envelope = min(motor.peak_torque_Nm,
                 P_voltage / max(omega, omegaReg),
                 T_linear)
```

实现使用 `omegaReg=1 rad/s` 作零速数值正则化。`rpm>=n_stop` 时能力为
0，主标签按冻结优先级记为 `motor_speed`；`motor_voltage` 只在
`n_peak_load<rpm<n_stop` 的电压降额区参与主标签。低速区即使
`T_linear` 数值钳位到 220 Nm，也不把电压误报为 active/primary。

600 V 锚点：

```text
P_voltage = 104000 * 600 / 830 = 75180.7229 W
n_peak_load = 5.65 * 600 = 3390 rpm
n_stop = 10.14 * 600 = 6084 rpm
```

该连续包络是保守工程推断，不是厂家发布的 600 V 曲线。830 V 单调性测试使用放宽到 830 V 的纯数学场景；FSG demo 仍受 600 V 规则和逆变器上限约束。

## TSAC 功率链与电流域

以有效 TS 电压 `V` 计算：

```text
P_tsac_cap = min(rules.max_ts_power_W,
                 V * rules.max_ts_current_A,
                 battery.P_discharge_peak_W,
                 V * battery.I_discharge_peak_A,
                 inverter.P_dc_peak_W,
                 V * inverter.I_dc_peak_A)

P_mechanical_cap = max(P_tsac_cap - battery.P_ts_aux_W, 0)
                   * inverter.eta_const
                   * motor.eta_const
```

顺序必须是先限制 TSAC 出口功率，再扣除辅助功率，最后乘逆变器和电机效率。
`battery.eta_discharge` 留给电池能量账本，不在该 TSAC 出口能力链中重复相乘。

规则、电池和逆变器 DC 电流均使用 A；电机与逆变器相电流均使用 Arms。
严禁把 DC A 与 phase Arms 直接取最小值。相电流通过
`motor.Kt_Nm_per_Arms` 转成扭矩候选；TS DC 限制先转成功率，再转成机械扭矩候选。

`evaluate_powertrain_constraints` 返回每个候选扭矩、候选名称、对应发布标签和主标签。
候选覆盖电机峰值扭矩/相电流、逆变器相电流、规则/电池/逆变器的功率与
DC 电流、motor power、motor voltage 和 speed cutoff。轮端力为：

```text
F_wheel = T_motor * gear_ratio * drivetrain_efficiency
          / tire.rolling_radius_m
```

该力尚未与轮胎 traction 候选合并。

## 结果与 active constraint 契约

后续圈速集成的 `limiter` 必须是 `N x 1`。`active_constraints` 只允许
14 个字段：

```text
lateral, brake, traction,
motor_torque, motor_power, motor_speed, motor_voltage,
rule_power, rule_current,
battery_power, battery_current,
inverter_power, inverter_current,
top_speed
```

active 判定：利用率 `util>=1-1e-3`；速度使用绝对 0.01 加相对
`1e-3` 容差；加速度使用绝对 0.01 加相对 `1e-3` 容差；`brake`
仅在贴合 `ax_min` 时成立。

并列主标签优先级严格为：

```text
lateral > brake > top_speed >
rule_power > rule_current >
battery_power > battery_current >
inverter_power > inverter_current >
motor_speed > motor_voltage > motor_power > motor_torque >
traction > coasting > unconstrained
```

圈速集成按实际节点 `v/ax/ay` 重算轮载、combined-slip 轮胎能力和动力能力；
`lateral/brake/traction/top_speed` 由该集成层与 GGV 边界共同判定。内点负加速度
只有贴合 `ax_min` 时才是 `brake`，不能按加速度符号直接分类。

功率结果必须区分 capability 与 usage。每个节点至少记录：电机转速、可用/已用电机扭矩、可用/已用轮端力、TSAC 功率上限/实际使用、各 DC 电流、各 phase Arms、
`V_bus`、`voltage_scenario`、`thermal_feasibility_evaluated=false` 和
`regen_enabled=false`。输入配置中的 `motor.thermal_model_enabled=false` 与
`rules.regen_enabled_in_model=false` 是模型开关；它们不是对外结果字段。
不能用“可用能力”替代“实际使用量”进行能量积分。

## 能量与耐久契约

节点级 `result.powertrain` 是能力/使用量快照；能量账本使用相邻节点重构的分段量。
闭合赛道有 `N` 段并显式包含 `N→1`，开放赛道有 `N-1` 段。分段加速度与均速为：

```text
ax_segment = (v_next^2 - v_start^2) / (2*ds)
v_mean = 0.5 * (v_start + v_next)
```

当前没有滚阻模型，因此 `F_rolling=0`。驱动力取
`max(m*ax_segment + F_drag, 0)`；回生关闭，制动段不会形成负能耗。
TSAC 功率按传动、逆变器和电机常效率链换算，并在每一段加入辅助功率。
辅助功率计入 TSAC 与电池侧 DC 功率/电流，不计入逆变器 DC 电流、
电机/逆变器 phase Arms、轮端力或电机扭矩。
模型采用固定 DC 母线电压、常数效率、无回生、无热状态；
`battery.eta_discharge` 仅将 TSAC 能量换算为电池储能消耗。

段与累计字段严格为：

```text
segment_energy_ts_kWh, segment_energy_stored_kWh
cumulative_energy_ts_kWh, cumulative_energy_stored_kWh
```

累计数组长度为 `N_segment+1` 且首项为 0。标量输出为
`E_lap_ts_kWh`、`E_lap_stored_kWh`、`E_endurance_stored_kWh`、
`E_nominal_required_kWh`、`SOC_end_estimated` 和
`can_finish_endurance_estimated`。耐久储能需求等于单圈储能消耗乘圈数和安全系数；
这里假定每圈都重复同一个已求得的最小圈速剖面，不模拟圈间 SOC、温度或性能变化。
标称容量需求再除以 `SOC_init-SOC_min`。结束 SOC 由耐久储能需求除以标称容量估算，
可行性判定在 `SOC_min` 边界使用浮点容差。

`models.endurance` 与 powertrain 独立。D7.1.3 规定完整耐久约 22 km。
冻结的真实赛道换算结果为 `ceil(...)=26` 圈；本提交只记录任务书给定结果，未读取或运行真实赛道。
`safety_factor=1.10` 是工程假设，不是规则值。

`endurance_energy_feasible`、`thermal_feasibility_evaluated`、
`regen_enabled` 和 `voltage_scenario` 是全局状态，不属于点级 limiter。
对外提供的预生成 GGV 若启用组合动力，必须带有匹配的动力和轮胎 provenance；
当前 GGV 未保存完整气动参数 provenance，因此启用气动的预生成 GGV 被保守拒绝，
应由当前输入现场重新生成。
