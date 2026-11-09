# Snowbot

Snowbot 是基于 [Bindu](https://github.com/WXB1870/bindu) 开发的扫雪机器人应用，提供机器人模型、双臂遥操作、底层驱动适配、仿真和部署工具。通用运动求解与执行能力由 Bindu 提供，配对版本通过 [dependencies.repos](dependencies.repos) 固定。

## 快速开始

完整部署环境为 Ubuntu 24.04、ROS 2 Jazzy、Python 3.12 和 x86_64。依赖安装见[运行指南](docs/运行与验证.md#支持条件)。

```bash
git clone https://github.com/WXB1870/snowbot.git
cd snowbot
python3 -m tools.deployment.deploy prepare
```

`prepare` 获取匹配的 Bindu，生成底层源码并构建底层和上层软件。后续更新使用 `python3 -m tools.deployment.deploy build`。

仅在改动涉及阻抗会话时，按需使用 `python3 -m tools.deployment.deploy build --validate session`（另支持 `rt`、`nominal`、`all`）。入口直接使用本仓库配套测试，不增加逐文件夹具哈希门槛；缺模型会在耗时编译前报错。验证与构建均复用增量产物，并显示阶段耗时；验证或编译失败修正后重跑原命令即可。普通更新继续用 `build`，不默认运行额外测试，不启动设备。

Orin 首次选择 `python3 -m tools.deployment.deploy prepare --with-impedance --machine orin`；已有部署可在下一次 `build` 时指定 `--machine orin`。选择保存在生成的部署元数据中，后续构建和 `run_robot.sh` 自动复用。机器配方 [orin.json](src/snowbot_bringup/config/deployments/machines/orin.json) 固定网卡、默认 ROS 域 88、阻抗模式、VR 端口及 TLS 路径引用；证书与私钥内容留在机器上。显式设置的 `ROS_DOMAIN_ID` 仍优先。不选机器配方时沿用原现场配置。

完成构建后，按[现场配置](docs/运行与验证.md#首次现场配置)设置设备、标定和 PICO 网络，再按[启动顺序](docs/运行与验证.md#日常启动)运行。开发和仿真见[仿真与浏览器操作台](docs/运行与验证.md#仿真与浏览器操作台)。

终端2可选 `bash tools/run_robot.sh vr --home` 或 `bash tools/run_robot.sh replay --home`：先确认原位使能保持，再通过现有轨迹生成器平滑将双臂和腰部回关节零位，头部／夹爪不参与；到位、停稳后开放业务。回零时双臂速度≤0.25 rad/s，腰部两轴速度≤0.05 rad/s（由 `runtime_parameters.json` 的 `execution.startup_home_waist_max_velocity_rad_s` 调整）；加速度≤0.5 rad/s²、jerk≤162 rad/s³，同时服从关节配置，不改业务限值。默认不回零，Ctrl+C继续走原有停止流程；replay随后仍执行原有Prepare。

VR 长按 A 的动态准备回位默认时长为15秒，由 `teleop.json` 的 `modes.real.prepare_duration` 调整；实际时长仍可被腰部同步约束延长，已认证动作库转场不使用此参数。

扫雪耦合模式因双手相对握持误差锁定后，可在停止已确认、双侧握及扳机均松开时长按 X（默认1秒）请求触地抬升。软件用 replay `g099-curved` 的1.2米握点到杆端安装关系、杆半径及地平面估计杆体距地高度，排除已过时的铲盒包围盒；杆体最低点不高于地面2厘米时，验证双臂共同竖直抬升5厘米的路径，腰部固定，设定笛卡尔速度上限2厘米/秒、关节速度上限0.05 rad/s。持续按住 X 执行单段，松开 X 或按 Y 停止；段末仍锁定，松开 X 后按 B 复核，若仍触地可再次明确请求。恢复保留原握持参考，不回 Home、不自动重试；设备Fault、停止未确认、反馈无效或抬升过程中误差增大仍停止。参数在真实工具配置的 `ground_recovery` 中；几何只用于此功能，不恢复已删除的旧杆显示／碰撞代理。该高度是模型估计，不能证明接触原因或工具滑移；日志保留估计高度及恢复结果，实际脱困效果需现场验收。

直连和远程都用 `bash tools/run_robot.sh vr` 启动。手柄输入有效期和换源静默时间均为 1 秒，双手配对窗口为 50 毫秒，到帧间隔告警阈值为 1 秒。统一在 [runtime_parameters.json](src/snowbot_bringup/config/robots/snowbot/runtime_parameters.json) 的 `pico` 和 `diagnostics` 中调整。已有现场配置中的 `remote_vr` 覆盖值对所有 VR 连接生效。

默认诊断在本次会话目录的 `timing-summary.jsonl` 每5秒汇总输入／反馈／命令年龄、采样间隔、求解／提交／服务及Action等待耗时，以及可用的网络RTT，退出时补写尾段。每项含次数、P50、P95、最大值和超限次数；期限采用本次配置或请求值，无对应门槛时超限数为null。统计来自观察端收到的消息、在日志抽样前收集，队列丢失仍以诊断health为准。浏览器每5秒最多一个HEAD请求，4秒超时，结果随手柄消息带回；HTTP往返含浏览器／服务调度，TCP平滑RTT仅到连接对端，两者都不是采集到硬件动作的完整延时。可加 `--input-timing` 补充输入管线窗口最大值；分位数最多保留每项每窗8192个样本，实际用于分位数的数量单列，异常及超时次数保留。

终端直接显示电脑只读界面和头显入口的实际局域网 IP。同一局域网的电脑可打开打印的 `http://机器IP:8013/`，头显使用打印的 HTTPS 入口；内部显示通信仍仅限本机。

只需松开持杆夹爪时，先可靠支撑杆／铲，保持 `base` 运行并退出 VR／replay 等上层，再执行 `bash tools/run_robot.sh open-grippers`。该入口独占当前 ROS 域和命名空间，禁止双臂／腰部写权限、启动保持和回零，只按现有配置打开双爪；实测到位、停稳并释放夹爪租约后退出，不自动闭爪。反馈过期、故障、手臂未停稳或控制权冲突会拒绝／中止，未到位不报成功。执行入口存在不代表已部署或通过真机验收。

正式位置会话固定为`session_v3`，日常菜单和配方无需选择协议。旧现场JSON的`arguments.position_protocol`、节点的`position_protocol`及servo YAML的`position_reference_mode`均须删除；加载时会明确拒绝。内部`reference_mode=2`、普通目标配方version 2、刚性预测version 3和准备回放version 4/5保持原语义。位置／阻抗仍为执行方式，旧Follower的`position_max_velocity/position_max_acceleration`须显式迁移到`trajectory_max_velocity/trajectory_max_acceleration`，`bindu_guard`须删除，旧键均会拒绝加载。`main` 同时支持 `position` 与 `impedance`；编入阻抗后端仍由已有 `--with-impedance` 选择，现场版本不自动升级。

## 维护分支

Bindu 与 Snowbot 均维护以下两条分支。两仓 main 保持现有架构，精简分支承载配套重构；Snowbot 通过 `dependencies.repos` 分别固定对应 Bindu 分支的提交，应用与框架配套选版。

| 分支 | 定位 | 使用边界 |
|---|---|---|
| `main` | 保持现有架构的真机部署维护线 | 真机更新从此选版，部署与运动验收由现场执行 |
| `xavier/session-v3-position-impedance` | 通信链路、模块分层及 session_v3 精简的重构候选 | 尚未经过真机测试，暂不替换 main |

两仓共享功能与修复先进入 main，再同步到精简分支；尚未验收的框架及应用重构保留在精简分支，完成真机验收后再考虑合入 main。其他专题、部署及实验分支转为本地 `archive/` 标签，保留提交和工作树，不再作为维护入口；远端旧引用未随本地整理删除。

## 功能

- **双臂遥操作**：PICO 手柄输入、独立双臂与工具耦合模式、准备位和夹爪操作。
- **运动执行**：通过 Bindu 统一处理运动求解、控制权、目标期限、动态限制和停止反馈。
- **设备适配**：底层位置会话、关节与设备反馈、夹爪及型号配置。
- **仿真与可视化**：轻量运动学显示、Isaac 物理后端和浏览器操作台。
- **部署工具**：依赖锁定、底层补丁生成、增量构建和配置清单。

## 目录结构

```text
snowbot/
├── src/
│   ├── snowbot_bringup/       launch、配置、仿真工具与浏览器页面
│   ├── snowbot_description/   URDF、CAD 与模型资源
│   ├── snowbot_hardware/      设备协议、位置会话与反馈适配
│   └── snowbot_teleop/        PICO 操作流程与协调节点
├── vendor/snowbot/           底层驱动源码与基线清单
├── tools/deployment/         部署入口、源码准备器与补丁
├── tests/                    单元测试与集成检查
├── docs/                     运行文档与本地工作记录
├── artifacts/                本地产物（不入库）
│   ├── field/                按设备、批次保存现场回执、日志和分析
│   ├── experiments/          离线实验数据、图表和视频
│   ├── validation/           软件验证结果
│   ├── exports/              导出文档与参数
│   └── archive/              历史归档
├── releases/                 工具生成的版本快照（不入库）
└── dependencies.repos        Bindu 版本锁
```

**常用入口**：调参数看 [机器人配置](src/snowbot_bringup/config/robots/snowbot/)，改手柄流程看 [遥操作源码](src/snowbot_teleop/snowbot_teleop/)，查现场记录看本地 `artifacts/field/118/`，找实验结果看本地 `artifacts/experiments/`。`config/deployments/` 是启动配方，与现场记录分开。

运行入口在 `snowbot_bringup.runtime`，显示与诊断在 `snowbot_bringup.visualization`，仿真在 `snowbot_bringup.simulation`；页面位于该包的 `share/web/`。遥操作业务逻辑和可复用的预设规划／回放在 `snowbot_teleop`，设备协议和反馈会话在 `snowbot_hardware`。

## 配置与模型

[配置目录](src/snowbot_bringup/config/)随 `snowbot_bringup` 安装。按要调整的内容选择入口。

| 要调整的内容 | 配置位置 |
|---|---|
| 真机或位置仿真启动哪些组件 | [deployments/](src/snowbot_bringup/config/deployments/) 下的 real、sim 两份配方 |
| VR 有效期、双手配对、求解超时和执行时序 | [runtime_parameters.json](src/snowbot_bringup/config/robots/snowbot/runtime_parameters.json)。直连与转发共用同一套输入参数 |
| 准备姿态、按钮流程、握持确认；夹爪和底盘参数 | [teleop.json](src/snowbot_bringup/config/robots/snowbot/teleop.json)、[grippers.json](src/snowbot_bringup/config/robots/snowbot/grippers.json)、[snowbot_base.json](src/snowbot_bringup/config/robots/snowbot/snowbot_base.json) |
| 本体、关节限制、双臂工具几何和求解权重 | [本体入口](src/snowbot_bringup/config/robots/snowbot/robot_snowbot_real.json)引用关节配置和[双臂工具配置](src/snowbot_bringup/config/robots/snowbot/bimanual_snowbot_real.json)。独立加载双臂工具时，从同目录的运行参数读取默认时序 |
| 动作回放数据、雪铲物理仿真参数 | [primitive-library/](src/snowbot_bringup/config/robots/snowbot/primitive-library/) 和 [snowbot_shovel_physics.json](src/snowbot_bringup/config/robots/snowbot/snowbot_shovel_physics.json) |
| 单臂、独立适配和演示配置；默认 launch／节点及相关测试仍会使用 | [examples/snowbot/](src/snowbot_bringup/config/examples/snowbot/)；当前双臂 VR 配方使用上面的 `robots/snowbot/` |

`teleop.json` 和 `grippers.json` 的公共字段位于顶层，`modes.real`／`modes.simulation` 保存各环境差异，运行时按配方的 `runtime_mode` 合并。

资源使用 `package://snowbot_bringup/...` 和 `package://snowbot_description/...` 路径。选择机器配方后，每次 prepare/build 从 [真机模板](src/snowbot_bringup/config/deployments/snowbot_vr_real.json)、[公共底层参数](tools/deployment/snowbot_vendor/position_session/target_stream.yaml) 和机器覆盖值生成 `upper/site.json`、`vendor-position.yaml`，长期修改应进入这些维护源。上层 VR 限速 1.0 rad/s，replay 执行器与底层上限 1.25 rad/s，分别由模板和部署入口／公共底层参数维护；未改变现有配套值。生成文件、安装路径及证书密钥不入库。

[D 型把手雪铲](src/snowbot_description/urdf/d_handle_snow_shovel.urdf)包含 28×45 cm 浅铲、22 cm 横把和名义间距 34 cm 的左右握点，右手握杆位置可用于不同握距评估。`left_grip_frame` 的 +Z 沿横把，`right_grip_frame` 的 +Z 沿杆。固定握点坐标用于标定，离线双轴握持通过 Bindu 的 `left_axial_grasp`、`right_axial_grasp` 求解，工具跟踪完整六维目标；CAD 与物理形状共用该 URDF。

机器人模型的生成器维护在配套 Bindu 的 `tools/models/prepare_snowbot_description.py`，输入来源见 [资源说明](src/snowbot_description/SNOWBOT_ASSETS_NOTICE)。各文件用途如下：

| 模型位置（位于 `snowbot_description` 包内） | 用途 |
|---|---|
| `sources/snowbot.urdf` | 用户提供的关节模型输入 |
| `urdf/snowbot.urdf` | 包含几何资源的完整模型 |
| `urdf/snowbot_display.urdf` | 固定轮关节的显示模型 |
| `urdf/snowbot_control.urdf` | 用于求解的关节与坐标模型，当前真机／仿真配方共用 |
| `urdf/snowbot_control_real.urdf` | 既有真机限位变体，当前配方未引用 |
| `urdf/d_handle_snow_shovel.urdf` | D 型把手雪铲的模型与握点 |

## 开发

通用契约、求解与执行逻辑在 Bindu 维护；型号配置、底层适配和操作流程在 Snowbot 维护。底层代码以 `vendor/snowbot/original/` 为基线，在 `tools/deployment/snowbot_vendor/position_session/` 维护补丁与公共头，通过 [prepare_position_session.py](tools/deployment/prepare_position_session.py) 生成构建源码。

准备器显式传入 `--with-impedance` 时叠加已固定的118关节阻抗控制器与参数，来源和源码哈希见 [impedance_118.json](vendor/snowbot/impedance_118.json)。双臂阻抗复用会话目标流，腰部保持位置控制；上层与servo须配对选择 `execution_mode=impedance`，默认仍为位置模式、标定未确认。阻抗会话从本部署安装目录读取参数（可用已有 `IMPEDANCE_PARAMS_YAML` 显式覆盖），不读取同事工作区；独立参考、旧阻抗开关、双臂模式及直接力矩测试订阅不注册。导入参数未经本项目真机验收，真实标定完成后才能确认 `impedance_calibration_confirmed`；本地原生检查不代表完整ROS构建或实体跟踪／停止通过。

应用测试从仓库根目录执行，例如 `python -m unittest tests.unit.test_pico_control`。ROS 与原生依赖的加载方式、仿真构建及集成检查见[运行指南](docs/运行与验证.md#仿真与浏览器操作台)。

## 文档

- [运行与验证](docs/运行与验证.md)：依赖、构建、配置、PICO 操作与仿真。
- [Bindu](https://github.com/WXB1870/bindu)：通用框架与模块文档。

本地工作资料也集中在 `docs/`，不随 Git 分发：

| 资料 | 用途 |
|---|---|
| [TODO](docs/TODO.md) | 待完成任务 |
| [当前进度](docs/当前进度.md) | 当前状态、边界与下一步 |
| [底层驱动问题记录](docs/底层驱动问题记录.md) | 已知问题、措施和证据 |
| [位置遥操作修改目标](docs/位置遥操作修改目标.md) | 遥操作需求与目标 |
| [过程记录](docs/过程记录/) | 按日期追溯修改与验证 |
