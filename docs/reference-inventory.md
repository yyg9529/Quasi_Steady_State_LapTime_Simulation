# Reference 资产清单

记录日期：2026-07-13。

## `deep-research-report.md`

- SHA-256：`DA21A466EABE050C5BE1641689F4CA35DEED4722093A6B9A7832C0B5975B2144`
- 用途：项目分层、坐标与数据规范、求解路线、测试和里程碑参考。
- 注意：报告中的研究引用标记需要在正式技术文档中重新核实，不能直接视为可发布引用。

## `FSAE_Simscape_Suspension`

- 快照规模：23 个工作树文件。
- 原目录中的 `.git` 为空，无法恢复有效提交号或远程地址。
- 可复用内容：
  - hardpoints CSV schema 与 SI 单位转换；
  - corner 参数构造和局部坐标系；
  - jounce/bump sweep 工作流；
  - camber、toe、caster、KPI、Motion Ratio 后处理；
  - suspension lookup CSV/MAT 输出样例。
- 集成原则：仅作为离线悬架预处理来源；主圈速求解器只读取标准化 lookup 数据。

## `FSAE-VD-Personal-Scripts-main`

- 外层快照提交：`a99a25af19e0d6941e715d9cbe93fbfdbf8e3e81`。
- 内层项目来源提交：`05a0a6bc4a09631106ebda80ceeab428fb68d6ef`。
- 原始远程：
  - `https://github.com/yyg9529/FSAE-VD-Personal-Scripts.git`
  - upstream：`https://github.com/Alphabet1671/FSAE-VD-Personal-Scripts.git`
- License：MIT，版权声明为 `Copyright (c) 2025 Alphabet1671`。
- 快照状态：`Yaw Dynamics/Full Vehicle Model/Base Models/SuspensionModelBase.slx` 相对来源提交存在本地修改；本仓库保留的是该修改后的工作树文件。
- 原项目 `.gitignore` 保存为 `gitignore.snapshot.txt`，避免其规则继续影响主仓库；`.tir` 轮胎文件作为参考数据保留跟踪，`.slxc` 和 `slprj` 生成缓存不跟踪。
- 可复用内容：
  - 简化 UniTire 求解器、拟合代码和拟合参数；
  - 简化车辆、动力系统和气动模型思路；
  - 弹簧/阻尼器表格及预处理函数；
  - 四角垂向悬架参数、符号约定、测试工况和日志接口；
  - 赛道一维曲线与瞬时速度上限计算思路。
- 集成原则：优先提取方程、数据 schema、测试夹具和经过验证的纯函数；不直接复制未完成类结构或把旧工程加入运行路径。

## 复用检查清单

任何资产进入 `src/`、`data/` 或 `preprocessing/` 前都需要确认：

1. 来源和许可证允许复用。
2. 输入输出、单位、坐标系和符号明确。
3. 与 Phase 0 接口一致，或通过边界适配器转换。
4. 不依赖 `reference/` 的相对路径或旧项目工作区变量。
5. 有最小复现测试，并与旧结果或解析结果进行对照。
