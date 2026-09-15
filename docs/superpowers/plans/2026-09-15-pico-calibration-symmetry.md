# PICO 标定收尾自动骨长对称化

用户已确认实施。顺序实施，不使用子智能体、不提交；不启动设备。

## 设计与边界

保留原始双侧测量文件及质量证据。每次 geometry candidate 成功激活后，
默认 symmetric_max 检查两侧完整标定，再生成独立版本目录；上臂和前臂分别取最大值。
缺少另一侧时报告 pending，不阻断当前侧测量保存；完整但无效的标定拒绝派生。
新目录完整生成后原子更新 `runtime_symmetric` 符号链接，旧版本保留。
拒绝覆盖普通文件、目录或指向非本工具版本目录的链接。
校准入口 `--geometry-policy original` 保留只保存测量的模式。
遥操仍显式使用 `--pico-calibration-dir .../runtime_symmetric`，不改变旧默认入口。
禁止在派生目录内采集或修改标定。此任务不改变驱动、裸手路线、IK、C 键行为。

## 实施与验证

- [x] tests/test_pico_calibration_finalize.py：先验证原始模式不写入、缺件 pending、
  双侧校验失败不更新、版本保留及固定链接更新、路径冲突拒绝。
- [x] vendor/pico_tracker/src/pico_bridge/scripts/pico_calibration_finalize.py：
  复用 create_profile，提供 finalize(source, policy) 和命令行入口。
- [x] vendor/pico_tracker/scripts/calibrate_pico_arm.sh：解析策略；成功激活 geometry
  后调用收尾模块；status 展示固定入口；派生目录禁止标定。
- [x] vendor/pico_tracker/src/pico_bridge/test/test_pico_calibration_menu.py：
  无设备验证参数、派生目录保护、成功激活后的调用顺序。
- [x] 更新 README 的标定及固定运行入口说明；运行 unittest、vendor pytest、bash -n、
  git diff --check。现场设备验收另行进行，不把离线通过当作现场通过。

## 本轮验证记录

- `pixi run python -m unittest discover -s tests -p test_pico_calibration_finalize.py -v`：4 项通过。
- 对称骨长单元测试：2 项通过。
- vendor 环境 pytest：calibration_artifact、calibration_menu、skeleton_filter_core/node 共 110 项通过。
- 用实际六份标定的临时副本连续生成两版本；固定链接切换、旧版本保留、原始 SHA256 不变。
- 从工程根目录执行 README 中标定命令的 `--help` 通过；没有调用设备采样。
- 自动收尾尚未经新的真人标定操作验收；既有对称快照的现场遥操效果已获用户肯定，二者区分。
