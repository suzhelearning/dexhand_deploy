# 01 — G0 预检

## 适用前置

操作者、值守安全员、设备管理员必须在场；确认物理急停可触及，机器人工作区无人且无松动物体。真实 case 需要 robot IP/model、Marvin SDK/firmware、Wuji 型号/firmware（如启用）、Motive rigid IDs（如使用动捕）、H5 输入 SHA256。不得通过修改 limit、timeout、freshness 或关闭 ACL 解决预检失败。

## 完整命令

```bash
# 终端 0：唯一 router（采集仓库）
cd /home/current/syz/mocap/acquisition
TIANJI_ROUTER_ENDPOINT=tcp/127.0.0.1:7447 pixi run start-router

# 终端 1：teleop 仓库
cd /home/current/xxl/tianji_teleop
export TIANJI_ROUTER_ENDPOINT=tcp/127.0.0.1:7447
pixi run validation-run -- --list
pixi run doctor
```

若未使用默认端口，所有终端显式设置同一 endpoint。不得从 session YAML 复制 endpoint，不得在采集端或 teleop 端再启动第二 router。

## 步骤与预期

1. 运行 `validation-run -- --list`，确认 18 个固定 case ID 全部存在，矩阵 profile、capability、active side、hand mode、比例、duration、prerequisite、stop criteria 可解析。
2. 从 doctor 输出记录唯一 router ZID；检查 ACL default-deny、`mocap/**`/`tianji/**` data rule、`tj/live/**` liveliness rule。任意未授权 key 被允许都算失败。
3. 核对 robot/hand 型号、IP、SDK/firmware、Home 姿态和当前硬件急停状态；执行连接前 readiness，不能要求 policy observation 已 ready。
4. 记录 source、producer、coordinator、executor、recorder instance，以及 runtime/config/ACL hash。确认同 logical ID 没有第二 live instance；发现重复 authority 立即危险停止。
5. 对 H5 记录 SHA256、Motive rigid ID；对真实手部确认模式是 retarget 或 direct 且只有一个 hand command publisher。
6. 建立结果根目录并记录预检事件：
   ```bash
   pixi run validation-run -- --case pico_sim --output ROOT --fake --headless \
     --operator-event preflight='operator checked physical E-stop and workspace'
   ```
   此命令只验证 bundle 链路，不能作为设备 pass。

## 立即停止条件

router ZID 不唯一或运行中变化、ACL 放行未授权 key、急停不可用、Home/limit 不明确、型号/IP/firmware 不匹配、SDK 连接失败、第二 authority、instance/token 不一致，都必须保持急停并停止；不要 return，不要自动 clear。Marvin 的 reconnect race、recorder teardown、headless config、calibration instance、overlay 第二 authority 若无法观察到明确安全行为，按失败处理。

## 记录与通过判据

记录 `manifest.yaml`、`status.jsonl`、`operator_events.jsonl`、doctor 输出、机器型号/OS、两个仓库 commit+dirty、runtime/config/ACL hash、router endpoint/ZID、设备照片或资产编号（不得含敏感凭据）。G0 仅在所有字段完整、唯一 router/authority、急停和 Home 可用、ACL default-deny 通过时通过；fake/headless 的 `operator_result=aborted` 不能升级为 pass。
