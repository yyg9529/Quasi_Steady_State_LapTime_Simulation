# 悬架 lookup 接口

## 范围

Simscape 悬架模型只作为离线几何预处理工具。生产运行路径读取其导出的 CSV，不加载 `.slx`，因此基础 MATLAB 环境仍可运行 QSS。

## 输入与单位

CSV 必须包含：

```text
jounce_mm, damperStroke_mm, motionRatio_local,
toe_deg, camber_deg, caster_deg, kpi_deg,
scrubRadiusRaw_mm, casterTrailRaw_mm, statusFlag
```

读取器只保留 `statusFlag == 1` 的行，要求有效断点唯一、递增且所有必需值有限。角度进入运行时结构后转换为 rad；jounce、减振器行程及几何诊断长度保留 mm，以匹配离线文件的显式接口。

## Motion Ratio 定义

本项目冻结的 CSV 字段定义为局部导数：

```text
MR = d(damper length) / d(wheel jounce)
```

正值表示车轮正向上跳时减振器长度增加。它不是“从静态点计算的总行程比”，也不是其倒数。虽然局部线性模型常写成 `k_wheel ≈ k_spring * MR^2`，当前导出件尚未完成减振器压缩方向、弹簧/摇臂以及整车左右角点的统一确认，因此结构中的 `wheel_rate_usage_confirmed=false`；V0.7 不允许使用该值计算轮端刚度。

## 插值和 fallback

- lookup 内使用一维线性插值，断点返回原值。
- 默认越界抛出 `QSSLTS:SuspensionRange`。
- 只有调用者显式传入 `OutOfRange="clamp"` 才钳位，并通过 `was_clamped=true` 暴露该行为。
- 无 lookup 时使用 constant suspension；camber/toe 保持常数，Motion Ratio 明确不可用。

## 当前物理边界

仅有 `jounce -> geometry` 映射不足以求整车姿态或轮胎载荷。要让悬架改变 GGV，还需要至少加入静态参考、轮率/弹簧、ARB、四角垂向平衡以及左右角点符号转换。完成这些前，本接口只用于数据验证和后续模型接缝。
