# Snowbot

Snowbot 是基于 Bindu 通用机器人框架的扫雪应用场景。本目录维护Snowbot应用包、底层准备工具、专属配置和现场记录：源码基线、补丁、问题、恢复备份与验证证据；Bindu公共框架和可复用适配接口仍在 `../galbot/bindu/`。

双臂遥操作是Snowbot的定制应用，可复用的能力和接口改进在Bindu维护，定制操作流程、按键方案、场景参数、现场配方与验证记录在Snowbot维护。通用双臂求解与执行仍在Bindu；型号适配、PICO扫雪操作、模型与配方在本工作区维护。

- [代理规范](AGENTS.md)：本项目协作与执行边界。
- [当前进度](当前进度.md)：原版恢复、改进版保全及未完成项。
- [TODO](TODO.md)：待实施问题、建议方向与完成条件。
- [位置遥操作修改目标](位置遥操作修改目标.md)：能力复用、阻抗隔离边界、修改目标与完成条件。
- [运行与验证](运行与验证.md)：本机准备工具、现场流程和验收边界。
- [底层驱动问题记录](底层驱动问题记录.md)：接口差异、问题证据和修复历史。
- [当前过程记录](过程记录/2026-10-27.md)：夹爪位置、运行入口及文档收敛；整理前历史仍保留在对应日期记录。

底层原始源码基线位于 [baseline](vendor/snowbot/original/)，包含上层、中层、servo、消息、头腰、力传感器和模型包；配套位置会话源码由[prepare_position_session.py](tools/deployment/prepare_position_session.py)生成，补丁与共用头在`tools/deployment/snowbot_vendor/position_session/`，离线测试在`tests/`。早期`development-src`、完整恢复归档与现场证据仅保留在本地，不随Git分发；旧`prepare_snowbot_vendor.py`用于复现历史补丁。

选定本机候选、最近已有部署回执、未完成项及验收边界统一见[当前进度](当前进度.md)。此前原版恢复、各轮部署、JSON、源码归档和bundle保留为历史证据；旧版本和绝对路径只用于溯源，不代表当前在线设备状态。

本目录不自动连接、同步或启动机器人。机器人运动禁令及本机修改后同步的协作边界见[本项目规范](AGENTS.md#3-机器人软件工程约束)。

## 当前工程结构

```text
snowbot/
├── src/
│   ├── snowbot_bringup/       launch、部署配方与机器人配置
│   ├── snowbot_description/   URDF、CAD及来源声明
│   ├── snowbot_hardware/      厂商消息、会话、反馈、夹爪及设备节点
│   └── snowbot_teleop/        PICO准备、装卸、操作状态机及协调节点
├── vendor/snowbot/            original/原始输入、baseline.json与SHA身份
├── tools/deployment/         统一部署入口、底层准备器与补丁维护源
├── tests/                    底层离线检查；跨框架回归沿用Bindu tests
├── releases/                 唯一命名的生成候选与index.json（本地）
├── deployments/118/          已有部署批次与回执；index.json标明记录状态（本地）
├── artifacts/                archive/冻结历史、exports/交付物（本地）
└── backups/                  本机恢复材料（本地）
```

配置模板的唯一维护目录为 `src/snowbot_bringup/config/`：`deployments/` 选择装配，`robots/snowbot/` 保存本体索引、profile、工具、PICO、夹爪及可选底盘参数。默认实机参数按最后部署证据整理，单机覆盖保留在现场配方。部署配方用 `robot_package=snowbot_bringup` 选择应用 launch，资源通过 `package://snowbot_bringup/...` 和 `package://snowbot_description/...` 解析。现场 `site.json` 保存实际选择与网络、设备、标定条件；未知字段及无效组合仍拒绝。

厂商原始基线已逐字节迁至 `vendor/snowbot/original/`，由 `vendor/snowbot/baseline.json` 固定其清单身份；日常修改在准备器、补丁与公共头文件维护，生成目录只作输出，不再作为第二个维护源。原始文件与同事资产不改写，`artifacts/`、`vendor/snowbot/`、`releases/`、`deployments/`、`backups/`中的 `COLCON_IGNORE` 防止外层构建误装配历史源码或原始驱动。Bindu原导入入口为兼容转发，新实现只维护一次。

生成与历史产物按[产物存放约定](运行与验证.md#基线与可复现边界)维护。当前本机候选见[发布索引](releases/index.json)，最近一次有回执的118版本见[部署索引](deployments/index.json)，历史位置映射见[产物索引](artifacts/index.json)。索引标注盘点日期及证据状态；历史回执不证明机器人当前在线版本。生成目录只作输出，不直接修改。

## 部署与使用入口

从[运行指南](运行与验证.md#首次准备与构建)获取选定Snowbot版本后，执行`python3 -m tools.deployment.deploy prepare`；统一入口按`dependencies.repos`获取锁定Bindu，构建包含servo的vendor和两仓上层。后续`build`复用环境，`start`复用现场配置及既有预检，人工顺序启动底层。支持条件及现场标定边界见运行指南。

默认控制行为以118最后部署的df2645a及配套底层、现场覆盖为对照；旧单仓安装保留原样，不把本次软件整理视为118已更新。现场配置、私钥及构建产物保持本地。

## 未实现项

姿态优化保存在两仓的`xavier/posture-optimization`开发分支。它未部署、未完成实体效果验收，不作为main或实机运行指南中的已实现能力。

## 现场条件与待确认项

- 118为Ubuntu 24.04.5／ROS 2 Jazzy／Python 3.12.3／x86_64；现场工作区分别为 `/home/cc/snowbot_ws` 和 `/home/cc/bindu`。`/home/sun/snowbot`为历史环境路径，118使用上述工作区。
- 25电机与20逻辑轴对应、物理轴序／零点、夹爪／握点／六维力标定和持载停止仍待现场确认。
- 恢复原版及其后改进部署均有历史记录；反馈能力与通过结果须按具体版本和采集时间核对，不据旧恢复记录推断当前底层状态。
