# 7DOF 关键事件验证

## 用途与状态

V0.8 的 7DOF 模块用于直线加速、直线制动和受控定半径事件的交叉一致性检查，不参与 QSS 圈速求解。状态顺序为：

```text
[vx, vy, yaw_rate, omega_FL, omega_FR, omega_RL, omega_RR]
```

内部坐标继续使用 X 前、Y 左、Z 上，左转横向加速度和 yaw rate 为正。车身方程为纵向、横向、横摆 3DOF，另加四个车轮转动自由度。

## 轮胎本构更正

`tire_envelope` 只定义可用力上限，不能单独为 ODE 产生轮胎力。因此 V0.8 新增独立的 `tire.slip_force` 接口：简化模型先用线性纵向/侧偏刚度产生试算力，再投影到逐轮 p-norm combined-slip 包络。

滑移量采用平滑低速参考速度：

```text
Vref  = sqrt(Vx_wheel^2 + v_regularization^2)
kappa = (R*omega - Vx_wheel) / Vref
alpha = atan2(Vy_wheel, Vref)
```

概念刚度和正则速度没有经过轮胎试验拟合，不能作为轮胎真实性结论。V0.9 的高级轮胎适配器将替换该瞬态本构，而不会改变 QSS core。

## 载荷与扭矩

ODE 每次求值都迭代 `ax/ay -> Fz -> tire force -> ax/ay`，并在 diagnostics 中返回收敛状态、迭代次数和残差。轮荷仍使用准静态模型；悬架 lookup 尚未接入垂向平衡，camber 暂为零。

`powertrain.max_total_wheel_torque_Nm` 是所有驱动轮的总轮端扭矩；只在功率分支乘传动效率。`brake.max_total_brake_torque_Nm` 是四轮总机械制动扭矩；实际命令同时受 `max_total_brake_force_N * R` 与 `max_decel_g_mechanical * m * g * R` 约束，再按 `front_bias` 分轴、同轴均分。V0.8 不支持再生制动，启用时显式报错。

## 事件与 GGV 对比

- 加速和制动使用 `ode15s`，因为高滑移刚度使轮速方程呈刚性。
- 终止事件分别只接受速度上穿和下穿，并返回明确的 `termination_reason`；制动在正车速终止，禁止把反向轮速当正常结果。
- 当前 ODE 接口没有 aero，因此只允许与 aero-off、未校准的理论 GGV 对比；GGV 必须携带完全一致的 vehicle/tire/powertrain/brake provenance 和标准重力，事件未到达时不进行比较。
- 超过可配置差异阈值时发出 `QSSLTS:DOF7Mismatch`。

定半径函数不是单次固定转角开环。它对目标速度进行扫描，用简单纵向速度保持器和偏航角速度反馈分别维持速度与目标曲率。末段窗口使用速度/曲率峰值误差以及偏航角加速度/横向速度导数 RMS，避免正负振荡被均值抵消。只有速度网格同时包含稳定点及其后的失稳点、从而 bracket 横向极限时，才继续二分细化该区间；最高稳定点始终标记为 lower bound，GGV 对比使用细化后上下界的速度中点作为横向极限估计。若不能形成或细化 bracket，则不进行 GGV 对比。这仍只是事件控制器，不是全赛道驾驶员模型。

## 验证含义与限制

GGV 与 7DOF 共用轮胎包络，因此二者一致只能发现符号、轮荷、扭矩分配和动态实现错误，不能替代轮胎试验或实车验证。独立测试另检查零输入平衡、左右镜像、低速有限性、机械能符号，以及关闭载荷转移后的 RWD 解析加速范围。

当前仍未包含轮胎松弛长度、回正力矩、差速器、Ackermann、滚阻、气动、悬架垂向平衡和全赛道驾驶员模型。
