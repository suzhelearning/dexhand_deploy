# Mapped Palm＋Manus 实施计划

按已确认的 `../specs/2026-09-13-mapped-palm-manus-design.md` 顺序执行，不使用子智能体、不提交代码。

## 工作清单

- [x] 固定 tag 源码闭包、正式 bandwidth 配置和模型，生成 source manifest；验证摘要。
- [x] 新建独立 C++ mapped-palm 周期与 IPC worker；保留原版控制器和参考生成器。
- [x] 构建并验证 mapped 目标、Headroom、臂角、QP、模型参考状态和显式 reset。
- [x] 独立原版 Viewer/oracle 短轨迹 5430 周期对照，包含运动与同 tick 无新输入。
- [x] 用户 Eggbeat 基准 10,682 帧/24,001 周期独立对照；修复 reset TCP 缓存滞后一周期与绘图加速度比较口径。详见 `../../mapped-palm-porting.md`。
- [x] 本工程专项验证 stale/hold、epoch 切换、显式 reset；不将其表述为原版 oracle 全场景覆盖。
- [x] 输入合同、配置解析、后端工厂和实时会话支持新名称，默认 SPARK 不变。
- [x] Manus/仿真/录制接入及新后端身份、来源参数记录。
- [x] 老 SPARK、PICO2 与退出清理回归；更新操作文档和未完成验收项。
- [ ] 真实 PICO＋控制器＋Manus 现场验收（用户接入后执行）。

原生后端路径为 `src/tianji_teleop/src/ik/mapped_palm/`，构建产物为
`build/mapped-palm-native/mapped_palm_native_worker`。依赖隔离在
`tools/mapped_palm_native/`，运行时不访问参考 checkout。Python 会话的默认参数
保持原 SPARK，新增 backend 参数必须经过输入形状/仿真能力校验。

验证命令：

```bash
PYTHONPATH=src/tianji_teleop pixi run python -m unittest discover -s tests -p 'test_mapped_palm*.py' -v
pixi run --manifest-path tools/mapped_palm_native/pixi.toml configure
pixi run --manifest-path tools/mapped_palm_native/pixi.toml build
PYTHONPATH=src/tianji_teleop pixi run python -m unittest discover -s tests -q
```

只对已执行并通过的项目打勾。真实设备测试由用户接入设备后执行，离线结果不冒充硬件验收。
