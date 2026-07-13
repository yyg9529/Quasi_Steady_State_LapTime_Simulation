# 高级轮胎适配器契约

## V0.9 边界

V0.9 提供 Magic Formula / UniTire 的显式 placeholder 和可替换适配缝，但不把参考目录中的参数文件作为已验证模型启用。当前参考资产存在 evaluator 依赖缺失、参数版本来源不唯一、轮胎侧与压力定义不完整等问题；直接启用会产生不可审计的假精度。

本接口是 `QSSLTS_TIRE_FORCE_V1` 稳态代数轮胎力契约，不是带松弛长度或内部状态的通用瞬态轮胎模型。evaluator 必须无状态、确定性；7DOF 的轮荷固定点可能在同一仿真时刻重复调用它。

## 与 QSS 的隔离

- `tire.model_type` 定义 QSS/GGV 使用的轮胎能力包络。
- `tire.slip_force.model_type` 定义 7DOF 使用的滑移—力本构。
- `src/core/` 只消费 GGV，不调用本适配器。
- GGV provenance 只保存 `tire_envelope_provenance(tire)`，不保存含函数句柄的 `slip_force`。

因此替换 7DOF evaluator 不修改 QSS core，也不会改变理论 GGV；若未来用高级轮胎生成能力边界，应新增独立的离线 GGV/envelope 预处理层。

## 配置结构

```matlab
tire.slip_force = make_unitire_placeholder("simple");
% 或：tire.slip_force = make_magic_formula_placeholder();

tire.slip_force.evaluator = @myCanonicalEvaluator;
tire.slip_force.parameters = parameterBundle;
tire.slip_force.metadata.source = "参数来源说明";
tire.slip_force.metadata.evaluator_id = "稳定且可追溯的 evaluator ID";
tire.slip_force.metadata.native_tire_side = "LEFT"; % RIGHT/SYMMETRIC
```

`adapter_contract` 固定为 `QSSLTS_TIRE_FORCE_V1`。`adapter_type` 与 `model_schema` 必须成对匹配：

| adapter_type | model_schema |
|---|---|
| `unitire_simple` | `unitire_simple_v1` |
| `unitire_full` | `unitire_full_v1` |
| `magic_formula_pac2002` | `pac2002_v1` |

`parameters.model_schema` 还必须与 metadata 一致，避免简化 UniTire、完整 UniTire 或 PAC2002 参数误载。

## Canonical request 与输出

`tire_force_from_slip` 的第六个可选参数为 `operatingPoint`。7DOF 按固定轮序 `[FL, FR, RL, RR]` 传入：

```text
wheel_vx_mps        每轮轮胎坐标系有符号纵向速度
wheel_omega_radps   每轮角速度
wheel_side          LEFT / RIGHT
pressure_Pa         仅声明 requires_pressure=true 时必需
```

evaluator 每次接收一个 canonical request：

```text
kappa, alpha_rad, Fz_N, camber_rad,
wheel_vx_mps, wheel_omega_radps, wheel_side[, pressure_Pa]
```

其中 `Fz_N` 是非负载荷幅值；`kappa>0` 表示轮胎周向速度高于纵向接地点速度，`alpha_rad>0` 表示轮胎坐标中的接地点速度指向左侧。evaluator 只能返回项目内部轮胎坐标的有限标量 `Fx_N/Fy_N`（X 前、Y 左）。对无偏移、对称模型，零点附近通常表现为正 `kappa` 增加正 `Fx`、正 `alpha` 增加负 `Fy`；带水平偏移、外倾推力或非对称项时，总力不保证始终满足该符号，契约不据此拒绝输出。具体 MF/UniTire wrapper 负责把原生输入和输出转换为 canonical 语义；SAE Y-right 力可用 `convert_tire_force_to_internal` 集中换号，不允许 generic adapter 与 evaluator 重复转换。

同一次调用会把非零 `kappa/alpha` 一起传给 evaluator，高级模型自行处理 combined slip；generic adapter 不再做 p-norm 二次投影。

## 有效域和显式失败

`valid_domain` 至少包含 `slip_ratio`、`slip_angle_rad`、`Fz_N`、`camber_rad` 和 `wheel_vx_mps` 的 `[min,max]`。需要压力时还必须包含 `pressure_Pa`。默认 `extrapolation_policy="error"`；显式设为 `"allow"` 时才执行外推，并返回 `force.is_extrapolated=true`，不做静默 clamp。7DOF 继续把该状态写入 `diagnostics.tire_extrapolated` 和轨迹 `trace.tire_extrapolated`，避免事件仿真吞掉越域信息。

简化 UniTire placeholder 记录的是冻结参考拟合数据各维度的轴对齐边界：`Fz=222–1112 N`、`kappa≈[-0.2176,0.1817]`、`alpha≈[-0.2135,0.2133] rad`、零 camber、带速 `40.193/3.6 m/s`。这些边界只提供必要的越界保护，不证明边界盒内任意 `kappa × alpha × Fz` 组合都被试验覆盖，也不是函数可计算范围。`Fz<=0` 的点在调用 evaluator 前直接置零，混合轮荷时也不会把零载荷送入外部模型。

主要错误标识：

- `QSSLTS:TireParameterSchema`：契约、参数 schema 或 metadata 不一致；
- `QSSLTS:TireAdapterUnavailable`：placeholder 尚未配置 evaluator；
- `QSSLTS:TireDomain`：运行点缺失或越出有效域；
- `QSSLTS:TireCamberUnsupported`：模型不支持当前 camber；
- `QSSLTS:TireSideUnsupported`：原生轮胎侧与当前角点不兼容；
- `QSSLTS:TirePressureRequired`：模型要求压力但未提供；
- `QSSLTS:TireAdapterOutput`：evaluator 输出缺失或非有限。

不会静默退回 `simple_saturated`。

## 当前不支持

V1 契约尚不接收/应用 `Mz`、`Mx/My`、动态有效半径、滚阻、turn slip、胎温、磨损、压力动态和松弛长度。当前 7DOF 仍使用固定滚动半径，偏航方程只累加轮胎力对重心的力矩；为防止使用者误以为额外物理量已经生效，evaluator 返回 `Fx_N/Fy_N` 之外的字段会被 `QSSLTS:TireAdapterOutput` 拒绝。
