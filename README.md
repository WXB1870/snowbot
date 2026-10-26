# Snowbot

Snowbot 是基于 Bindu 通用机器人框架的扫雪应用场景。本目录维护本轮118现场联调的专属资料：底层源码快照、补丁、现场问题、恢复备份与验证记录；Bindu公共框架和可复用适配接口仍在 `../galbot/bindu/`。

双臂遥操作是Snowbot的定制应用，可复用的能力和接口改进在Bindu维护，定制操作流程、按键方案、场景参数、现场配方与验证记录在Snowbot维护。现有双臂运行实现仍在Bindu，本轮补充归属和修改规则。

- [代理规范](AGENTS.md)：本项目协作与执行边界。
- [当前进度](当前进度.md)：原版恢复、改进版保全及未完成项。
- [位置遥操作修改目标](位置遥操作修改目标.md)：能力复用、阻抗隔离边界、修改目标与完成条件。
- [运行与验证](运行与验证.md)：本机准备工具、现场流程和验收边界。
- [底层驱动问题记录](底层驱动问题记录.md)：接口差异、问题证据和修复历史。
- [当日过程记录](过程记录/2026-10-21.md)：118联调与资料迁移记录。

底层原始源码基线位于 [baseline](artifacts/position-session-20261021/baseline/)，包含上层、中层、servo、消息、头腰、力传感器和模型包；配套位置会话源码由[prepare_position_session.py](tools/deployment/prepare_position_session.py)生成，补丁与共用头在`tools/deployment/snowbot_vendor/position_session/`，离线测试在`tests/`。早期`development-src`、完整恢复归档与现场证据仅保留在本地，不随Git分发；旧`prepare_snowbot_vendor.py`用于复现历史补丁。

118已完成本轮位置会话软件部署：Bindu `5ee3868`／底层`f02b5de`叠加保留4处同事修改；后续初始化诊断版、现场运行／反馈时效限制及备份配方统一见[当前进度](当前进度.md)。此前原版恢复、JSON、源码归档和bundle保留为历史证据，其中旧绝对路径及当时的Bindu路径只用于溯源。

本目录不自动连接、同步或启动机器人。机器人运动禁令及本机修改后同步的协作边界见[本项目规范](AGENTS.md#3-机器人软件工程约束)。

## 源码与离线检查

- Snowbot源码：[WXB1870/snowbot](https://github.com/WXB1870/snowbot)。
- Bindu框架与原生执行实现：[WXB1870/bindu](https://github.com/WXB1870/bindu)。

两个仓库可克隆在同一父目录。以下在Snowbot仓库根目录执行，显式指定Bindu源码路径；只生成本机源码副本，不连接设备、不构建或启动驱动。已有输出目录不会覆盖。

```bash
git rev-parse HEAD
python3 -m tools.deployment.prepare_position_session \
  artifacts/position-session-20261021/baseline \
  --execution-source ../bindu/src/control/bindu_execution/native \
  --output /tmp/snowbot-position
python3 -m tests.integration.validate_position_invariants \
  --baseline artifacts/position-session-20261021/baseline \
  --source /tmp/snowbot-position/src
python3 -m tests.integration.validate_position_native --source /tmp/snowbot-position/src
python3 -m tests.integration.validate_position_model --source /tmp/snowbot-position/src
```

原始基线按现有完整文件清单纳入Git，保留原目录及文件；准备器逐文件校验SHA后应用补丁，并记录所选Bindu原生文件指纹。生成结果包含配套位置会话消息，原消息保持不变。完整servo构建另需匹配的ROS、Linux、Pinocchio及外部`kinematic_pose_msgs`等依赖；源码生成和内存检查覆盖离线准备流程，完整构建、现场部署与运动验证需分别完成。部分厂商包许可字段未完整声明，第三方源码沿用原许可证。

## 现场条件与待确认项

- 118为Ubuntu 24.04.5／ROS 2 Jazzy／Python 3.12.3／x86_64；现场工作区分别为 `/home/cc/snowbot_ws` 和 `/home/cc/bindu`。`/home/sun/snowbot`为历史环境路径，118使用上述工作区。
- 25电机与20逻辑轴对应、物理轴序／零点、夹爪／握点／六维力标定和持载停止仍待现场确认。
- 恢复前曾读到完整20轴反馈；当前底层已回到原版，历史通过结果以[当前进度](当前进度.md)限定时间和版本。
