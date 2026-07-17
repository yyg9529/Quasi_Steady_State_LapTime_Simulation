# FS 2026 车辆输入

`vehicle_fs_2026()` 继承 `vehicle_baseline()`，但不修改通用解析基准。
当前唯一经用户确认的 2026 实车参数是轴距。

| 参数 | 当前值 | 状态 | 来源 / 测量状态 |
| --- | ---: | --- | --- |
| `geometry.wheelbase_m` | 1.560 m | confirmed | User-confirmed 1560 mm |
| `mass.total_kg` | 300 kg | provisional | baseline-derived / not yet measured |
| `mass.front_static_frac` | 0.40 | provisional | baseline-derived / not yet measured |
| `mass.cg_height_m` | 0.250 m | provisional | baseline-derived / not yet measured |
| `geometry.track_front_m` | 1.200 m | provisional | baseline-derived / not yet measured |
| `geometry.track_rear_m` | 1.200 m | provisional | baseline-derived / not yet measured |
| `inertia.Iz_kgm2` | 120 kg·m² | provisional | baseline-derived / not yet measured |
| `load_transfer.front_lateral_distribution` | 0.50 | provisional | baseline-derived / not yet measured |
| `drivetrain.layout` | RWD | provisional | baseline-derived / not yet measured |

预设内的 `provenance` 冻结以下来源信息：

- `season = 2026`
- `wheelbase_source = "User-confirmed 1560 mm"`
- 所有列出的继承参数统一标记为 `provisional`、`baseline-derived`、
  `not yet measured`

在车队完成称重、质心、轮距、横摆惯量和载荷转移参数测量前，不应将这些
继承值解释为 2026 实车实测值。

## 尚未解锁的实车集成

当前仍缺少：加载状态有效滚动半径及胎压/载荷/车速工况、目标最高车速、
实际主动/从动齿数或总传动比，以及制动计算 Excel。获得这些输入前不生成
`powertrain_fs_2026()`、`brake_fs_2026()` 或 FS 2026 正式赛道结果；现有
EMRAX/HVCC 入口继续明确作为概念基准。
