# FS 2026 制动 Excel 映射

当前状态：**blocked pending source workbook**。工作区尚未收到用户制动计算 Excel，因此没有可冻结的文件 SHA-256、sheet、命名单元格、字段地址、单位或公式缓存。本项目不会根据单元格颜色或位置猜测含义，也不会生成 `brake_fs_2026()` 占位预设。

收到文件后先只读记录：绝对路径、SHA-256、sheet 名称、命名单元格、输入区、公式区、输出区、单位及公式缓存有效性。随后独立复算最大踏板力、踏板比、前后主缸、液压分配、卡钳活塞面积、摩擦系数、盘有效半径、前后轴转矩、前制动力比例、总制动力和机械减速度。

只有表格能够等价表示为固定前后液压比例时，才允许映射到：

```matlab
brake.enabled
brake.front_bias
brake.max_decel_g_mechanical
brake.max_total_brake_force_N
brake.max_total_brake_torque_Nm
```

如果前后轴存在独立饱和转矩，当前 `front_bias + total torque` 接口不充分，必须先评审是否增加 `max_front_brake_torque_Nm` 和 `max_rear_brake_torque_Nm`，不得用不等价总转矩替代。生产仿真最终只读取已验证 MATLAB 预设，不依赖 Excel、COM 或 Microsoft Excel 安装。
