# V1.0 验证与可复现实验报告

日期：2026-07-13
验证环境：Windows，MATLAB R2025b Update 5

## 1. 声明范围

本报告区分三类证据：

- **解析验证**：独立公式可推导，作为 correctness regression。
- **实现一致性**：对称性、约束可行性、网格加密和重复运行。
- **外部物理验证**：当前未完成；需要实车、台架或独立高保真模型。

因此，`simple_track`、synthetic GGV、DOE 和 7DOF 结果是确定性软件证据，不是实车圈速精度声明。

## 2. 可复现配置

`run_v1_reproducible_experiment` 使用：

- `data/track/simple_track.csv`，闭环 120 m、12 个 10 m 段；
- 300 kg baseline vehicle、概念级 load-sensitive tire、aero、RWD powertrain 和 brake baseline；
- GGV 速度网格 `[0:2:44,45] m/s`，横向网格 `-4:0.2:4 g`；
- 质量 `[280,300,320] kg`、功率 `[60,80,100] kW`、轮胎 μ scale `[0.9,1.0,1.1]` 的 `3×3×3=27` case；
- `calibration_mode="theory_only"`。

完整数组、参数、选项和非有限值编码位于 `results/v1/experiment_manifest.json`。manifest 同时记录 MATLAB release、version、platform，以及 `src/`、`preprocessing/`、参数函数和赛道输入共 65 个文件的逐文件 SHA-256，因此报告本身携带可审计的环境与代码内容身份。

## 3. 解析 regression

| 测试 | 独立参考 | 容差 | 当前结果 |
|---|---:|---:|---|
| 常数 μ、8 m 定半径速度 | 10.8480320795986 m/s | `1e-6 m/s` | 通过 |
| CSV 量化圆周圈时 | 4.63360373855568 s | `1e-6 s` | 通过 |
| `ay=0` RWD 加速 | 1.161020881670534 g | `1e-6 g` | 通过 |
| `ay=0`、60% 前偏置制动幅值 | 1.440414507772021 g | `1e-6 g` | 通过 |

对应测试：`tests/regression/v1BaselineRegressionTest.m`。

令 `q=h/L`、前轴静载比例为 `f`、制动前偏置为 `beta`，解析参考为：

```text
RWD acceleration: ax/g = mu_x*(1-f)/(1-mu_x*q)
braking magnitude: min(
    mu_x*f/(beta-mu_x*q),
    mu_x*(1-f)/((1-beta)+mu_x*q))
```

## 4. Benchmark 与数值一致性

- 直线加速覆盖 torque、power、45 m/s powertrain top-speed 分支。
- 直线 1 g 制动与 `v(s)=sqrt(2*g*(100-s))` 的最大允许误差为 `1e-8 m/s`。
- 曲率全反号后圈时/速度/纵向加速度不变，横向加速度反号。
- 完整闭环逐点满足 GGV 纵横向可行性；canonical GGV feasible 点固定点全部收敛，无横向搜索截断或意外 wheel lift。
- GGV 网格 `1 m/s + 0.1 g` 得到 7.287274972609034 s；加密至 `0.5 m/s + 0.05 g` 得到 7.279090597361369 s，相对差 0.112437%，通过 0.2% 门槛。

这些圈时只用于网格一致性，不作为物理真值。

最终全量测试结果：119 passed、0 failed、0 incomplete。7 个 example 已分别按正序和逆序连续执行通过，未发现 base workspace 残留导致的配置串扰。

## 5. 任务书最终入口 `run_003`

synthetic GGV 校准示例当前输出：

- 圈时 7.694481988 s；
- 最大速度 22.666 m/s；
- 最大纵/横向加速度 1.023 g / 1.591 g；
- `calibrated(model_load_sensitive_v0.4)` GGV；
- speed、ax、ay、curvature、limiter 图；
- 距离加权 limiter 表；
- 280/300/320 kg 质量敏感性表。

当前 limiter 分布为：`unconstrained` 50%、traction 33.333%、brake traction/bias 16.667%。其中 `unconstrained` 明确表示纵向加速度位于 GGV 上下界内部，不能把它计为制动或驱动能力受限。

该输入文件名中的 `real_ggv` 表示外部能力表接口；仓库所带 CSV 是 synthetic fixture，不是实测证据。

## 6. V1 baseline 与 DOE 报告

V1 theory-only baseline：

- 圈时 7.29985328830874 s；
- 最大速度 25.8773540419222 m/s；
- limiter 距离占比：traction 41.667%、`unconstrained` 33.333%、lateral 16.667%、brake traction/bias 8.333%。

27 个 case 全部收敛并具有有限圈时。当前软件快照中：

- fastest：case 25，280 kg、100 kW、μ scale 1.1，6.87902042362683 s；
- slowest：case 3，320 kg、60 kW、μ scale 0.9，7.80581200854484 s。

最快/最慢 limiter 对比已写入 `doe_extreme_limiter_comparison.csv`。占比反映 limiter 状态覆盖（包括 `unconstrained`），不表示因果秒差；参数因果判断应结合 sensitivity/DOE。

## 7. 运行命令

```matlab
projectRoot = project_setup();
results = runtests(fullfile(projectRoot,"tests"), IncludeSubfolders=true);
assertSuccess(results)
run(fullfile(projectRoot,"examples","run_003_real_ggv_calibrated_lap.m"))
run(fullfile(projectRoot,"examples","run_007_v1_reproducible_study.m"))
```

PowerShell 报告哈希：

```powershell
Get-ChildItem '.\results\v1' -File |
    Sort-Object Name |
    Get-FileHash -Algorithm SHA256
```

只保证同一代码、MATLAB 版本和平台的重复运行字节一致；不同 MATLAB 版本的 CSV/JSON 格式不保证一致。

本次连续运行前后的 7 个文件哈希完全一致：

| 文件 | SHA-256 |
|---|---|
| `baseline_limiter.csv` | `61C6314E5CFCD5FDD04DAFD7F2BEBCE535DE380BA10DEDAB472B6BF23F472E52` |
| `baseline_profile.csv` | `2B3123BEB4701B233D693F5A6D064F1ABC02ACB99400343A428CC85A1965E4F1` |
| `baseline_summary.csv` | `72D5993E17DD1D7A089788191E46D77561BE48D1B5BCA2EDDB3F18F5A6E60C8F` |
| `doe_case_metrics.csv` | `CBAD53B3C46CA75CFDBE737685FEF6CE160E5A65F53349BEA64365671EEE01E0` |
| `doe_extreme_limiter_comparison.csv` | `E77D2D1660CC43FE8A0582C46F9C88A0DC6BAF30561A7FF624A5E0971570DF05` |
| `doe_ranking.csv` | `42FE28ADAADE88D72E977AC0C1ED24CFB5B1119FDB80E7F432A550772C06CA43` |
| `experiment_manifest.json` | `710151AB02789BBADCED87E0A5748CB58D5D2DB2B779DA8AF93CABEC7393D1C5` |

## 8. 未完成验证

- 没有实车圈速、轮胎台架或独立高保真全车模型的定量误差对比；
- synthetic GGV 只验证校准链；
- 7DOF 使用的 simple-saturated 本构与 QSS 共享包络，属于实现一致性；
- 未提供已验证 MF/UniTire evaluator、能耗、赛线优化、坡度/横坡或空间 μ；
- Track 离散加密尚未形成正式自动化门禁，因此当前不能宣称赛道离散误差或更紧的物理精度。

回归更新时只能修改有独立解析依据的冻结值；其余软件快照应重新生成和审查，不能直接晋升为 truth。
