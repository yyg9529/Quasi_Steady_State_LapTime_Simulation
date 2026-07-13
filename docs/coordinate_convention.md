# 坐标、符号与单位约定

项目内部采用右手、Z-up 的 ISO 8855 风格车体坐标：

- `+X`：车辆前进；
- `+Y`：车辆左侧；
- `+Z`：竖直向上；
- `ax > 0`：向前加速；
- `ay > 0`：向左加速；
- `yaw rate > 0`：俯视逆时针；
- `curvature > 0`：左转；
- 轮位顺序固定为 `[FL, FR, RL, RR]`。

内部单位全部为 SI：速度 `m/s`、长度 `m`、质量 `kg`、力 `N`、加速度 `m/s^2`、角度 `rad`。只有文件字段明确带 `_g`、`_deg` 或 `_mm` 时才使用相应单位，进入核心模型前必须转换。

外部 SAE J670 Z-down 数据、轮胎坐标或 Simscape 局部坐标必须在适配器中转换，不允许在核心函数里隐式换号。

高级轮胎 evaluator 的输出统一为轮胎局部 `+X` 前、`+Y` 左。具体外部模型 wrapper 负责输入/输出转换；`convert_tire_force_to_internal` 可把 SAE J670 `+Y` 右的 `Fx/Fy` 转为内部符号。generic adapter 不再二次换号。
