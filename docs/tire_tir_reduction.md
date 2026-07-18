# PAC2002 TIR 离线降阶

## 边界

Hoosier PAC2002 文件只用于离线参数提取，不是圈速求解器的运行时轮胎模型。生产预设
`tire_hoosier_16x75_10_r20_qss` 保存已冻结的标量和源文件 SHA-256，不读取
原始 `.tir` 或 `.mat` 文件。

当前 QSS 轮胎仍采用载荷敏感的纯滑移峰值包络和 p-norm 组合约束，不包含外倾角、
温度、胎压动态、磨损或 PAC2002 瞬态状态。

## 离线工作流

```matlab
parsed = read_pac2002_tir(tirPath);
reduced = derive_qss_tire_parameters(parsed);
```

`read_pac2002_tir` 识别 section，解析有效 `KEY = VALUE`，去除 `$` 行注释和
行尾注释，并校验：

- `[UNITS]` 使用 `meter/newton/radians/kg/second`；
- `MODEL.PROPERTY_FILE_FORMAT` 为 `PAC2002`；
- 降阶所需的 `FNOMIN/LFZO/LMUX/LMUY/PDX1/PDX2/PDY1/PDY2` 均为有限标量；
- 源文件 SHA-256 由原始字节计算。

固定零外倾角、纯滑移峰值映射为：

```matlab
Fz_ref_N = FNOMIN * LFZO;
mu_x_ref = PDX1 * LMUX;
mu_y_ref = PDY1 * LMUY;
load_sensitivity_x = PDX2 / PDX1;
load_sensitivity_y = PDY2 / PDY1;
```

本地 Hoosier 源文件的冻结 SHA-256 为
`6AB8AA1219B7910A1660966AAD1B6C8FC7EF2FDB46F40BF0FA9B6FC172E53A2A`，对应生产值：

| 字段 | 值 |
|---|---:|
| `Fz_ref_N` | 667 N |
| `mu_x_ref` | 1.334655 |
| `mu_y_ref` | 1.804320 |
| `load_sensitivity_x` | -0.0168396327140722 |
| `load_sensitivity_y` | -0.0693256185155627 |
| `combined_n` | 2 |

`combined_n = 2` 是 **QSS p-norm model-reduction assumption**，不是从 TIR 中直接
提取的 PAC2002 系数；该假设记录在 `tire.provenance.combined_n_basis`。

单元测试在 300、667、1000、1500 N 四个代表载荷点，逐点比较 PAC2002 纯滑移峰值
`(PDX1 + PDX2*dfz)*LMUX`、`(PDY1 + PDY2*dfz)*LMUY` 与
`tire_load_sensitive_mu` 的输出，其中 `dfz=(Fz-Fz_ref)/Fz_ref`。

## 有效滚动半径契约

TIR 中的 `UNLOADED_RADIUS = 0.20066 m` 是自由半径，不等于加载、指定胎压和速度下
的有效滚动半径，因此降阶函数不生成 `rolling_radius_m`。

生产预设必须显式接收完整测量/模型工况：

```matlab
rollingRadius.value_m = 0.195;       % 示例值，不是已确认实车值
rollingRadius.load_N = 667;
rollingRadius.pressure_kPa = 82.7;
rollingRadius.speed_mps = 10;
rollingRadius.source = "loaded rolling-circumference measurement";

tire = tire_hoosier_16x75_10_r20_qss(rollingRadius);
```

五个字段均为必填。半径、载荷和胎压必须为正有限标量，速度必须为非负有限标量，
来源必须为非空文本。预设拒绝缺字段、`NaN` 和无效工况，并把完整输入保存在
`tire.provenance.rolling_radius`。在有效滚动半径得到实测或可信 PAC2002 计算确认前，
不得把示例值用于最终传动比与最高车速结论。

`source` 必须以以下受控方法之一开头，可在其后追加试验编号或报告标识：

- `loaded rolling-circumference measurement`
- `validated PAC2002 effective rolling radius`
- `validated tire-test fit`

`TIR UNLOADED_RADIUS`、`free radius` 或其他未验证文本会被拒绝，不能仅靠改写数值绕过来源契约。
