# 实车 GGV 校准约定

实车数据只用于识别理论基线的速度相关比例：横向、加速和制动三个 scale。比例需限幅、平滑并输出报告。

DOE 时必须为每个参数组合重新生成理论 GGV。推荐从 baseline 理论模型与对应实车数据识别一次校准 scale，然后把该 scale 冻结应用到各 DOE case；若每个 case 都重新对同一实车 GGV 拟合，校准会抵消待研究参数的物理敏感性。

`run_sensitivity_sweep` 和 `run_doe` 已执行该规则：有 `models.ggv_real` 时先生成 baseline 理论 GGV，识别一次 `v_mps,scale_lat,scale_acc,scale_brake`，随后删除 case 中的实车数据并对每个新理论 GGV调用 `apply_ggv_calibration_scales`。

任何来自日志点云的“最大值”都必须记录样本数量、分位数方法、速度分箱、滤波和有效域，避免把噪声峰值当成能力边界。
