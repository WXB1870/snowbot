# Snowbot

Snowbot是基于[Bindu通用机器人框架](https://github.com/WXB1870/bindu)的扫雪机器人应用，也是完整机器人的部署入口。本仓维护型号实现、场景参数、厂商底层维护源和现场流程；Bindu维护通用契约、求解、执行与ROS节点。两个仓库分别保留源码，通过`dependencies.repos`锁定配对，不要求它们处于固定的相邻目录。

新用户从[运行与验证](运行与验证.md#首次准备与构建)开始：获取Snowbot后执行`python3 -m tools.deployment.deploy prepare`，由工具获取匹配Bindu并完成底层与上层构建。支持条件、首次现场配置和人工启动顺序由该文档统一维护。

- [代理规范](AGENTS.md)：本项目协作与执行边界。
- [当前进度](当前进度.md)：原版恢复、改进版保全及未完成项。
- [位置遥操作修改目标](位置遥操作修改目标.md)：能力复用、阻抗隔离边界、修改目标与完成条件。
- [运行与验证](运行与验证.md)：本机准备工具、现场流程和验收边界。
- [底层驱动问题记录](底层驱动问题记录.md)：接口差异、问题证据和修复历史。
已有工作区可另有本地`TODO.md`和未入库的`过程记录/`条目，保存待办及现场证据；这些材料不作为新克隆的部署前提。

底层原始源码基线位于 [baseline](vendor/snowbot/original/)，包含上层、中层、servo、消息、头腰、力传感器和模型包；配套位置会话源码由[prepare_position_session.py](tools/deployment/prepare_position_session.py)生成，补丁与共用头在`tools/deployment/snowbot_vendor/position_session/`，离线测试在`tests/`。早期`development-src`、完整恢复归档与现场证据仅保留在本地，不随Git分发；旧`prepare_snowbot_vendor.py`用于复现历史补丁。

选定本机候选、最近已有部署回执、未完成项及验收边界统一见[当前进度](当前进度.md)。此前原版恢复、各轮部署、JSON、源码归档和bundle保留为历史证据；旧版本和绝对路径只用于溯源，不代表当前在线设备状态。

本目录不自动连接、同步或启动机器人。机器人运动禁令及本机修改后同步的协作边界见[本项目规范](AGENTS.md#3-机器人软件工程约束)。

## 两仓职责

| 内容 | Bindu | Snowbot |
|---|---|---|
| 公共契约与控制 | 消息、控制权、动作期限、动态限制、插值、停止及故障语义 | 选择设备后端并提供场景参数 |
| 算法与输入 | 通用FK／IK、双臂QP、VR输入与映射、通用ROS节点 | PICO准备、普通／耦合切换、虚握及夹爪操作流程 |
| 型号与配置 | 通用本体／设备接口及资源解析 | 模型、profile、准备位、工具几何、夹爪标定与配方模板 |
| 厂商底层 | 提供准备器使用的通用原生执行代码 | 原始vendor基线、专属补丁、公共头及生成工具；构建servo、中层、topic上层与消息 |
| 部署与使用 | 框架开发构建，以及供应用复用的`x86_real`上层构建工具 | 锁定Bindu、组织完整构建、现场配置、日常启动与设备问题记录 |

`snowbot_hardware`负责厂商协议、会话和反馈适配；真正的servo程序来自vendor源码与补丁。Bindu中的旧Snowbot导入和launch仅保留兼容转发，正式应用实现维护在本仓四个包中。生成树、现场配置、证书私钥及构建产物保持本地。

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
├── artifacts/                dependencies/锁定依赖、robot/部署产物及历史归档（本地）
└── backups/                  本机恢复材料（本地）
```

配置模板的唯一维护目录为 `src/snowbot_bringup/config/`：`deployments/` 选择装配，`robots/snowbot/` 保存本体索引、profile、工具、PICO、夹爪及可选底盘参数。默认实机参数以最后部署证据为基础，跟踪与回零使用2026-10-29确认的较宽口径，具体数值见[现场配置](运行与验证.md#首次现场配置)；单机覆盖保留在现场配方。部署配方用 `robot_package=snowbot_bringup` 选择应用 launch，资源通过 `package://snowbot_bringup/...` 和 `package://snowbot_description/...` 解析。现场 `site.json` 保存实际选择与网络、设备、标定条件；未知字段及无效组合仍拒绝。

厂商原始基线已逐字节迁至 `vendor/snowbot/original/`，由 `vendor/snowbot/baseline.json` 固定其清单身份；日常修改在准备器、补丁与公共头文件维护，生成目录只作输出，不再作为第二个维护源。原始文件与同事资产不改写，`artifacts/`、`vendor/snowbot/`、`releases/`、`deployments/`、`backups/`中的 `COLCON_IGNORE` 防止外层构建误装配历史源码或原始驱动。Bindu原导入入口为兼容转发，新实现只维护一次。

生成与历史产物按[产物存放约定](运行与验证.md#基线与可复现边界)维护。已有本地工作区的候选、部署及历史位置分别由`releases/index.json`、`deployments/index.json`和`artifacts/index.json`索引；这些机器资料不随Git分发。索引标注盘点日期及证据状态；历史回执不证明机器人当前在线版本。生成目录只作输出，不直接修改。

## 部署与使用入口

统一入口位于[deploy.py](tools/deployment/deploy.py)，配对版本以[dependencies.repos](dependencies.repos)为准。

| 阶段 | 从Snowbot执行 | 作用 |
|---|---|---|
| 首次准备 | `python3 -m tools.deployment.deploy prepare` | 获取锁定Bindu、准备依赖，构建包含servo的vendor及两仓上层 |
| 更新后构建 | `python3 -m tools.deployment.deploy build` | 按新依赖锁选择Bindu，复用环境与构建缓存 |
| 日常上层启动 | `python3 -m tools.deployment.deploy start --enable-commands` | 由现场人员先按指南启动servo→中层→topic上层，再复用现场配方及既有预检启动上层 |

准备和构建不启动设备。以上命令的系统前提、源码保护、产物路径与完整启动顺序见[运行指南](运行与验证.md)，不在README另维护一套操作步骤。

默认控制行为以118最后部署的df2645a及配套底层、现场覆盖为对照；当前跟踪／双臂回零判据是用户要求的明确差异，以`baseline.json`中的`default_policy`及运行指南为准，历史部署身份保留在`deployment_reference`。旧单仓安装保留原样，本机软件整理未更新118，也未完成目标环境完整构建或实机验收。现场配置、私钥及构建产物保持本地。

## 未实现项

姿态优化保存在两仓的`xavier/posture-optimization`开发分支。它未部署、未完成实体效果验收，不作为main或实机运行指南中的已实现能力。

## 现场条件与待确认项

- 118为Ubuntu 24.04.5／ROS 2 Jazzy／Python 3.12.3／x86_64；现场工作区分别为 `/home/cc/snowbot_ws` 和 `/home/cc/bindu`。`/home/sun/snowbot`为历史环境路径，118使用上述工作区。
- 25电机与20逻辑轴对应、物理轴序／零点、夹爪／握点／六维力标定和持载停止仍待现场确认。
- 恢复原版及其后改进部署均有历史记录；反馈能力与通过结果须按具体版本和采集时间核对，不据旧恢复记录推断当前底层状态。
