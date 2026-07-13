# 参考资产迁移说明

- TUM QSS 与其他开源项目只用于理解固定赛线、能力边界和前后向传播，不复制其 Python 架构。
- `reference/FSAE_Simscape_Suspension` 只用于导出悬架 lookup。现有脚本中的 `motionRatio_local` 明确定义为 `d(damper length)/d(wheel vertical travel)`；符号取决于局部几何，轮端刚度使用平方前仍需核对行程正方向。
- `reference/FSAE-VD-Personal-Scripts-main` 中的 UniTire 资产使用自己的轮胎坐标和有效域，进入生产代码前必须单独适配、验证许可证与坐标符号。
- `reference/` 不加入 MATLAB path，不在运行时读取。
